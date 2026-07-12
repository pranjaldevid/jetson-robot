# Companion ROBO: ROS 2 Node Architecture and Gazebo Simulation Plan

**Platform:** Jetson Orin Nano 8GB, JetPack 6.2.1, ROS 2 Humble
**Cameras:** 2x Pi Camera Module 2 (IMX219) fixed-baseline stereo, CAM0/CAM1 CSI
**Actuation:** Pico 2 W + Kitronik 5329 over `/dev/ttyACM0` serial (DRIVE/STOP/PING, 0.5 s watchdog). Pico never runs ROS.
**Sensor hub (planned):** ESP32 + VL53L5CX ToF + 1-2 ultrasonics over USB serial.
**Simulator:** Gazebo Fortress (the officially supported pairing for Humble via `ros\_gz`). Gazebo Classic 11 reached EOL in January 2025 and must not be used for new work.

Design rule enforced throughout: **the topic contract in Section 3 is identical in sim and on hardware.** Only the nodes that touch physical devices (camera driver, serial bridges, IMU) are swapped for Gazebo equivalents. Everything between the camera topics and `/cmd\_vel` never changes.

\---

## 1\. ROS 2 Package Structure

Workspace: `/ssd/robot/ros2\_ws/src` on the Jetson (mirrored in the repo under `ros2\_ws/src`).

### 1.1 Custom packages (we write these)

|Package|Type|Contents|Depends on|
|-|-|-|-|
|`robo\_interfaces`|CMake (msg-only)|Custom messages: `PersonTrack.msg`, `PersonTrackArray.msg`, `GestureCommand.msg`|`std\_msgs`, `geometry\_msgs`, `sensor\_msgs`|
|`robo\_description`|ament\_cmake|`urdf/robo.urdf.xacro` (chassis, wheels, stereo mount, sensor frames), Gazebo `<gazebo>` blocks in a separate `urdf/robo.gazebo.xacro`, RViz config|`xacro`, `robot\_state\_publisher`|
|`robo\_sim`|ament\_python|Gazebo Fortress world (`worlds/living\_room.sdf`), spawn launch, `ros\_gz\_bridge` config YAML, fake Pico serial endpoint (`fake\_pico.py`), sim-side launch files|`robo\_description`, `ros\_gz\_sim`, `ros\_gz\_bridge`|
|`robo\_drivers`|ament\_python|`motor\_bridge` node, `esp32\_bridge` node, `imu\_mpu6050` node, udev rules file (`99-robo-serial.rules`)|`rclpy`, `robo\_interfaces`, `pyserial`, `smbus2`|
|`robo\_perception`|ament\_python|`face\_detect\_node`, `face\_id\_node`, `pose\_node`, `gesture\_node`, `perception\_manager` (models per the vision-stack report: SCRFD + MobileFaceNet, YOLOv8n-pose TensorRT, MediaPipe Hands)|`rclpy`, `robo\_interfaces`, `cv\_bridge`, `image\_transport`|
|`robo\_navigation`|ament\_python|Nav2 parameter YAMLs (sim + real variants of *tuning values only*, same plugins), Nav2 launch wrapper, `twist\_mux` config|`nav2\_bringup`, `twist\_mux`|
|`robo\_bringup`|ament\_python|Top-level launch: `sim.launch.py`, `real.launch.py`, `perception.launch.py`, `slam.launch.py`, `nav.launch.py`. Each accepts `use\_sim:=true/false` where relevant|all of the above|

### 1.2 External packages (pull as-is, do not modify)

|Package|Source|Role|
|-|-|-|
|`isaac\_ros\_argus\_camera`|Isaac ROS (apt/Docker, JP6 line)|Real dual-CSI IMX219 driver, time-paired left/right frames|
|`isaac\_ros\_image\_proc`|Isaac ROS|GPU rectification (NITROS)|
|`isaac\_ros\_stereo\_image\_proc`|Isaac ROS|GPU SGM disparity -> `PointCloud2` for costmaps|
|`isaac\_ros\_visual\_slam`|Isaac ROS|cuVSLAM stereo VSLAM (chosen framework, see 2.3)|
|`nav2\_bringup` + Nav2 stack|apt (Humble)|Planning, control, costmaps, behavior server|
|`twist\_mux`|apt|Priority mux: gesture stop > Nav2 > teleop|
|`robot\_state\_publisher`, `joint\_state\_publisher`|apt|URDF static TF|
|`ros\_gz\_sim`, `ros\_gz\_bridge`|apt (`ros-humble-ros-gz`)|Gazebo Fortress integration|
|`teleop\_twist\_keyboard`|apt|Manual testing|
|`camera\_calibration` (image\_pipeline)|apt|One-time stereo calibration tooling|
|`rtabmap\_ros`|apt|**Fallback VSLAM only**; not launched by default|

Custom message definitions:

```
# PersonTrack.msg
uint32 track\_id
bool is\_primary\_user
float32 identity\_confidence
geometry\_msgs/PoseStamped position      # camera frame; z from disparity when available
float32\[] keypoints                     # 17x3 (x,y,conf) flattened, COCO order

# PersonTrackArray.msg
std\_msgs/Header header
PersonTrack\[] tracks

# GestureCommand.msg
std\_msgs/Header header
string gesture          # "stop" | "come" | "confirm" | "point" | "wave"
uint32 track\_id
bool from\_primary\_user
float32 confidence
```

\---

## 2\. Node-by-Node Responsibilities

### 2.1 Stereo camera source

**Real: `argus\_stereo\_node` (from `isaac\_ros\_argus\_camera`, launched by `robo\_bringup/real.launch.py`)**

* Responsibility: capture CAM0+CAM1 IMX219 at 1280x720@30, pair frames by Argus timestamps (Argus provides per-frame hardware timestamping with <100 us jitter; note the RPi v2 modules have no sync pin, so pairing is timestamp-based, see Risk R1).
* Inputs: none (hardware). Calibration from `robo\_description/config/stereo\_calibration/left.yaml`, `right.yaml`.
* Outputs (remapped to contract): `/stereo/left/image\_raw`, `/stereo/left/camera\_info`, `/stereo/right/image\_raw`, `/stereo/right/camera\_info`.
* Prereq: enable "Camera IMX219 Dual" in `jetson-io.py`, reboot, confirm `/dev/video0` and `/dev/video1`.

**Sim: Gazebo sensors + `ros\_gz\_bridge`**

* Two `camera` sensors on `camera\_left\_link`/`camera\_right\_link` in the URDF/SDF, same resolution/rate/FOV as real, `update\_rate: 30`, both updated in the same sim step so stamps match exactly.
* `ros\_gz\_bridge parameter\_bridge` maps Gazebo topics onto the identical contract names above, with `use\_sim\_time:=true` everywhere.
* Everything downstream cannot tell the difference.

### 2.2 Rectification: `rectify\_left`, `rectify\_right` (`isaac\_ros\_image\_proc::RectifyNode`, two instances)

* Responsibility: undistort/rectify using `camera\_info`.
* In: `/stereo/{left,right}/image\_raw` + `camera\_info`. Out: `/stereo/{left,right}/image\_rect` + `/stereo/{left,right}/camera\_info\_rect`.
* Sim difference: **none.** Gazebo cameras are distortion-free, so rectification is a near-identity operation, but we run it anyway to keep the graph identical.

### 2.3 VSLAM: `visual\_slam\_node` (`isaac\_ros\_visual\_slam`)

* **Chosen framework: Isaac ROS Visual SLAM (cuVSLAM), stereo mode.** Justification (consistent with the vision-stack report): only GPU-accelerated option, first-party on JetPack 6 + Humble, \~116 fps class on Orin Nano 8GB, leaves the CPU free for perception. It requires a stereo pair, which this rig now provides. IMU fusion stays **disabled** (MPU6050 is not hardware-synced; cuVSLAM stereo does not need it). Fallback: `rtabmap\_ros` stereo mode, same input topics.
* In: `/stereo/left/image\_rect`, `/stereo/right/image\_rect`, both `camera\_info\_rect` (remapped to the node's `visual\_slam/image\_{0,1}` / `camera\_info\_{0,1}` inputs).
* Out: TF `map -> odom` and `odom -> base\_link`; `/visual\_slam/tracking/odometry` (`nav\_msgs/Odometry`); status topics.
* Config: `base\_frame: base\_link`, `enable\_localization\_n\_mapping: true`, `enable\_slam\_visualization: false` on the robot.
* Sim difference: **none** except `use\_sim\_time:=true`. cuVSLAM consumes the simulated rectified pair unchanged.

### 2.4 Stereo depth for costmaps: `disparity\_node` + `point\_cloud\_node` (`isaac\_ros\_stereo\_image\_proc`)

* Responsibility: GPU SGM disparity from the rectified pair, then `PointCloud2` for Nav2 obstacle marking. This substitutes for the depth camera we are not buying.
* In: `/stereo/{left,right}/image\_rect` + `camera\_info\_rect`. Out: `/stereo/disparity` (`stereo\_msgs/DisparityImage`), `/stereo/points2` (`sensor\_msgs/PointCloud2`), throttled to 10 Hz for the costmap.
* Sim difference: **none.**

### 2.5 Perception (all subscribe to the left rectified feed only)

|Node|Responsibility|In|Out|Rate|
|-|-|-|-|-|
|`pose\_node`|YOLOv8n-pose TensorRT, person keypoints + ByteTrack IDs|`/stereo/left/image\_rect`|`/perception/tracks\_raw` (`PersonTrackArray`, `is\_primary\_user` unset)|10-15 Hz|
|`face\_detect\_node`|SCRFD-500MF TensorRT on head ROIs from tracks|`/stereo/left/image\_rect`, `/perception/tracks\_raw`|`/perception/faces` (`vision\_msgs/Detection2DArray`)|5-10 Hz|
|`face\_id\_node`|MobileFaceNet/ArcFace embedding + cosine match vs enrolled templates; event-driven (new/unconfirmed tracks only)|`/perception/faces`, `/stereo/left/image\_rect`|`/perception/identity` (`vision\_msgs/Classification2D`-style custom mapping onto track ids via `PersonTrackArray` update)|event|
|`gesture\_node`|MediaPipe Hands (CPU) on wrist ROIs of the primary user + landmark MLP; pose-heuristic gestures as milestone A|`/stereo/left/image\_rect`, `/perception/tracks`|`/perception/gesture` (`GestureCommand`)|10-15 Hz|
|`perception\_manager`|Fuses tracks + identity + gestures; owns identity gating; converts "stop" gesture into a zero-velocity override; publishes person goal for "come"|`/perception/tracks\_raw`, `/perception/identity`, `/perception/gesture`|`/perception/tracks` (`PersonTrackArray`, gated), `/cmd\_vel\_gesture` (`geometry\_msgs/Twist`, only on stop/hold), `/perception/goal` (`geometry\_msgs/PoseStamped`, on "come")|15 Hz|

* Sim difference: **none in code.** In Gazebo these nodes run against simulated images (they will simply detect nothing unless a human actor model is placed in the world; a `walking actor` SDF is included in `robo\_sim` for smoke-testing the pipeline plumbing, not accuracy).

### 2.6 ESP32 sensor bridge: `esp32\_bridge` (`robo\_drivers`)

* Responsibility: read newline-delimited JSON from the ESP32 (`{"t":<ms>,"tof":\[64 x mm],"us":\[mm,mm]}` at 10-15 Hz), validate, stamp, publish.
* In: serial `/dev/robo\_esp32` (udev-pinned symlink). Out: `/sensors/tof/points` (`sensor\_msgs/PointCloud2` in `tof\_link`, 8x8 zones projected via the VL53L5CX 45 deg FOV), `/sensors/us\_left` and `/sensors/us\_right` (`sensor\_msgs/Range` in their frames).
* Sim difference: node **not launched** in sim. Gazebo equivalents: a `gpu\_lidar` sensor constrained to the ToF FOV publishing to the same `/sensors/tof/points` topic via the bridge, plus two Gazebo ultrasonic (narrow `gpu\_lidar` or `sonar`) sensors bridged to `/sensors/us\_left`/`us\_right`.

### 2.7 IMU: `imu\_mpu6050` (`robo\_drivers`)

* Responsibility: read MPU6050 over I2C at 100 Hz, publish `sensor\_msgs/Imu` on `/imu/data\_raw` (orientation unset, gyro+accel only), frame `imu\_link`.
* Role in v1: telemetry and tilt safety only. **Not** fused into VSLAM or an EKF initially (unsynced consumer IMU; cuVSLAM stereo does not need it). A `robot\_localization` EKF is a later, optional enhancement.
* Sim difference: not launched; Gazebo `imu` sensor bridged to the same topic.

### 2.8 Navigation: Nav2 stack (`nav2\_bringup`, configured by `robo\_navigation`)

* Nodes (standard): `controller\_server` (DWB or RPP controller), `planner\_server` (NavFn/Smac2D), `bt\_navigator`, `behavior\_server`, `smoother\_server`, `velocity\_smoother`, lifecycle manager, `collision\_monitor` (optional, phase 2).
* Map strategy v1: **no static map.** Global costmap runs in rolling-window mode (10x10 m) in the `map` frame from cuVSLAM; local costmap 4x4 m in `odom`. This avoids coupling Nav2 to a mapping server while cuVSLAM's pose graph handles global consistency.
* Costmap observation sources: `/stereo/points2` (voxel layer, marking+raytrace), `/sensors/tof/points` (voxel layer), `/sensors/us\_left`/`us\_right` (`range\_sensor\_layer`). Inflation layer on both costmaps.
* In: goals via `/navigate\_to\_pose` action (from RViz, or `/perception/goal` forwarded by a 20-line `goal\_relay` in `perception\_manager`). Out: `/cmd\_vel\_nav` (`geometry\_msgs/Twist`).
* Sim difference: **none** except tuning values (sim friction differs; keep two param files with identical structure: `nav2\_params\_sim.yaml`, `nav2\_params\_real.yaml`).

### 2.9 Velocity arbitration: `twist\_mux`

* Priorities (high to low): `/cmd\_vel\_gesture` (100, timeout 0.5 s), `/cmd\_vel\_teleop` (50), `/cmd\_vel\_nav` (10). Output: `/cmd\_vel`.
* Sim difference: **none.** This is the single authoritative `/cmd\_vel` producer in both worlds.

### 2.10 Motor bridge: `motor\_bridge` (`robo\_drivers`), detailed in Section 7

* In: `/cmd\_vel` (`geometry\_msgs/Twist`). Out: serial DRIVE/STOP to the Pico; `/motor\_bridge/status` (`diagnostic\_msgs/DiagnosticArray`, 1 Hz).
* Sim difference: in Gazebo the DiffDrive plugin consumes `/cmd\_vel` directly, so `motor\_bridge` is **not required** for the robot to move in sim; it is nevertheless testable in sim against a fake Pico on a pseudo-terminal (Section 7.4).

\---

## 3\. Topic and Message Contracts (the implementation contract)

|Topic|Type|Publisher|Subscriber(s)|Rate|
|-|-|-|-|-|
|`/stereo/left/image\_raw`|`sensor\_msgs/Image`|argus\_stereo\_node (real) / ros\_gz\_bridge (sim)|rectify\_left|30 Hz|
|`/stereo/left/camera\_info`|`sensor\_msgs/CameraInfo`|same|rectify\_left|30 Hz|
|`/stereo/right/image\_raw`|`sensor\_msgs/Image`|same|rectify\_right|30 Hz|
|`/stereo/right/camera\_info`|`sensor\_msgs/CameraInfo`|same|rectify\_right|30 Hz|
|`/stereo/left/image\_rect`|`sensor\_msgs/Image`|rectify\_left|visual\_slam, disparity\_node, pose\_node, face\_detect\_node, face\_id\_node, gesture\_node|30 Hz|
|`/stereo/right/image\_rect`|`sensor\_msgs/Image`|rectify\_right|visual\_slam, disparity\_node|30 Hz|
|`/stereo/left/camera\_info\_rect`|`sensor\_msgs/CameraInfo`|rectify\_left|visual\_slam, disparity\_node|30 Hz|
|`/stereo/right/camera\_info\_rect`|`sensor\_msgs/CameraInfo`|rectify\_right|visual\_slam, disparity\_node|30 Hz|
|`/stereo/disparity`|`stereo\_msgs/DisparityImage`|disparity\_node|point\_cloud\_node|30 Hz|
|`/stereo/points2`|`sensor\_msgs/PointCloud2`|point\_cloud\_node|Nav2 costmaps|10 Hz (throttled)|
|`/visual\_slam/tracking/odometry`|`nav\_msgs/Odometry`|visual\_slam|Nav2 (odom topic), diagnostics|30 Hz|
|`/imu/data\_raw`|`sensor\_msgs/Imu`|imu\_mpu6050 (real) / bridge (sim)|(logging; optional EKF later)|100 Hz|
|`/sensors/tof/points`|`sensor\_msgs/PointCloud2`|esp32\_bridge (real) / bridge (sim)|Nav2 costmaps|10 Hz|
|`/sensors/us\_left`, `/sensors/us\_right`|`sensor\_msgs/Range`|esp32\_bridge (real) / bridge (sim)|Nav2 range layer|10 Hz|
|`/perception/tracks\_raw`|`robo\_interfaces/PersonTrackArray`|pose\_node|face\_detect\_node, perception\_manager|10-15 Hz|
|`/perception/faces`|`vision\_msgs/Detection2DArray`|face\_detect\_node|face\_id\_node|5-10 Hz|
|`/perception/identity`|`robo\_interfaces/PersonTrackArray` (id-annotated)|face\_id\_node|perception\_manager|event|
|`/perception/tracks`|`robo\_interfaces/PersonTrackArray`|perception\_manager|gesture\_node, app logic|15 Hz|
|`/perception/gesture`|`robo\_interfaces/GestureCommand`|gesture\_node|perception\_manager|event|
|`/perception/goal`|`geometry\_msgs/PoseStamped`|perception\_manager|bt\_navigator (via goal relay)|event|
|`/cmd\_vel\_nav`|`geometry\_msgs/Twist`|Nav2 velocity\_smoother|twist\_mux|20 Hz|
|`/cmd\_vel\_teleop`|`geometry\_msgs/Twist`|teleop\_twist\_keyboard (remapped)|twist\_mux|manual|
|`/cmd\_vel\_gesture`|`geometry\_msgs/Twist`|perception\_manager|twist\_mux|event (stop only)|
|`/cmd\_vel`|`geometry\_msgs/Twist`|twist\_mux|motor\_bridge (real) / gz DiffDrive via bridge (sim)|20 Hz|
|`/motor\_bridge/status`|`diagnostic\_msgs/DiagnosticArray`|motor\_bridge|diagnostics|1 Hz|
|`/tf`, `/tf\_static`|`tf2\_msgs/TFMessage`|see Section 4|all|var|

QoS notes: images and point clouds `SensorDataQoS` (best-effort, depth 1); `/cmd\_vel\*` reliable, depth 1; contract topics must not be renamed by any implementation step.

\---

## 4\. TF Tree

```
map                          (broadcast: visual\_slam\_node)
 └── odom                    (broadcast: visual\_slam\_node)
      └── base\_link          (broadcast: visual\_slam\_node, from stereo VO)
           ├── chassis\_link           (robot\_state\_publisher, static)
           ├── wheel\_{fl,fr,rl,rr}\_link  (robot\_state\_publisher via joint\_states; zeros on real robot, DiffDrive-driven in sim)
           ├── camera\_mount\_link      (static)
           │    ├── camera\_left\_link  -> camera\_left\_optical  (static, REP-103 optical rotation)
           │    └── camera\_right\_link -> camera\_right\_optical (static, baseline offset on Y)
           ├── imu\_link               (static)
           ├── tof\_link               (static, front-center, slight downward pitch)
           ├── us\_left\_link           (static)
           └── us\_right\_link          (static)
```

* `visual\_slam\_node` owns `map -> odom -> base\_link` (it publishes both; Nav2 publishes neither).
* `robot\_state\_publisher` owns every `base\_link -> \*` transform from the URDF.
* In sim, Gazebo does **not** publish any TF for the robot pose (ros\_gz odometry TF bridging is disabled) so that cuVSLAM is the single source of truth in both worlds. Ground-truth pose from Gazebo is bridged to `/sim/ground\_truth` (`nav\_msgs/Odometry`) for evaluation only, never into TF.
* cuVSLAM base frame is `base\_link`; the camera extrinsics chain reaches it through the URDF static TF, which is why the URDF baseline must match the calibrated reality (Section 5).

\---

## 5\. URDF / Robot Description Plan

`robo\_description/urdf/robo.urdf.xacro`, parameterized by xacro args so placeholder dimensions are one-line edits. **Placeholders below are guesses; correct them from the physical chassis when you measure it.**

|Element|Placeholder value (xacro property)|
|-|-|
|Chassis (box)|250 x 200 x 120 mm, mass 1.8 kg|
|Wheels|4x, diameter 60 mm, width 25 mm, track width 180 mm, wheelbase 140 mm|
|Drive model|Differential (left pair / right pair as in skid-steer; DiffDrive plugin with two joints per side)|
|Camera mount|Front-top, 100 mm above chassis top, 0 deg pitch|
|**Stereo baseline**|**80 mm** (Y offset between camera\_left\_link and camera\_right\_link; must equal the physical rig exactly once built)|
|Camera intrinsics (sim)|1280x720, horizontal FOV 1.086 rad (62.2 deg, IMX219), 30 Hz|
|IMU|chassis center, `imu\_link`|
|ToF|front bumper center, pitched down 10 deg|
|Ultrasonics|front corners, +/- 20 deg yaw|

Structure:

* `robo.urdf.xacro`: links/joints/inertials, `<xacro:arg name="use\_sim" default="false"/>`.
* `robo.gazebo.xacro` (included only when `use\_sim`): Gazebo Fortress plugins:

  * `gz-sim-diff-drive-system`: wheel joints, wheel separation/radius, subscribes to bridged `/cmd\_vel`, publishes joint states. Odometry publication and odom TF **disabled**.
  * Two `<sensor type="camera">` blocks (left/right) with matching update rate so both frames carry identical stamps.
  * `<sensor type="imu">` on `imu\_link`.
  * `gpu\_lidar` configured to 45 deg cone, 8x8-ish resolution for the ToF stand-in; two 15 deg cones for ultrasonic stand-ins.
  * `gz-sim-joint-state-publisher-system`.
* `ros\_gz\_bridge` config YAML in `robo\_sim/config/bridge.yaml` maps every Gazebo topic to the Section 3 contract names.
* The **same xacro without `use\_sim`** is what `robot\_state\_publisher` loads on the real robot, guaranteeing the TF tree is identical.

Calibration note: after the physical rig is built, run `camera\_calibration` stereo mode (checkerboard) and store the YAMLs in `robo\_description/config/stereo\_calibration/`; the URDF baseline property must then be set to the calibrated translation, not the CAD guess.

\---

## 6\. Simulation-to-Real Swap Strategy (node by node)

|Node|In Gazebo|On hardware|Changes|
|-|-|-|-|
|Stereo source|gz camera sensors + ros\_gz\_bridge|`argus\_stereo\_node`|**Swapped.** Same output topics/types/rates|
|rectify\_left/right|runs|runs|none|
|disparity + point cloud|runs|runs|none|
|visual\_slam (cuVSLAM)|runs, `use\_sim\_time:=true`|runs|none|
|pose/face/face\_id/gesture/perception\_manager|run (nothing meaningful to detect unless actor present)|run|none|
|esp32\_bridge|not launched; gz sensors bridged to same topics|launched|**Swapped source**, same topics|
|imu\_mpu6050|not launched; gz IMU bridged|launched|**Swapped source**, same topic|
|Nav2 (all servers)|runs, `nav2\_params\_sim.yaml`|runs, `nav2\_params\_real.yaml`|tuning values only, identical plugin structure|
|twist\_mux|runs|runs|none|
|motor\_bridge|optional, against fake Pico pty; gz DiffDrive consumes `/cmd\_vel`|required, against `/dev/robo\_pico`|**only the serial device path changes** (a `serial\_port` parameter)|
|robot\_state\_publisher|runs (`use\_sim:=true` xacro arg)|runs|xacro arg only|
|use\_sim\_time|true for every node|false|launch argument|

Launch discipline: `sim.launch.py` = gz world + spawn + bridge + common stack; `real.launch.py` = argus + esp32\_bridge + imu + motor\_bridge + common stack. Both include the same `common.launch.py` (rectify, disparity, VSLAM, Nav2, twist\_mux, perception). If a change is ever needed inside `common.launch.py` to make hardware work, that is an architecture bug to fix, not a fork point.

\---

## 7\. Motor Bridge Node Detail (`robo\_drivers/motor\_bridge.py`)

### 7.1 Interface

* Subscribes: `/cmd\_vel` (`geometry\_msgs/Twist`). **Confirmed recommendation: standard `Twist` via twist\_mux.** No custom message; Nav2, teleop and gesture overrides all speak Twist natively. (`TwistStamped` is unnecessary here; Nav2 Humble outputs unstamped Twist.)
* Publishes: `/motor\_bridge/status` (`diagnostic\_msgs/DiagnosticArray`): serial connected, last PING round-trip, watchdog margin, current duty cycles.
* Parameters: `serial\_port` (default `/dev/robo\_pico`), `baud`, `wheel\_separation` (0.18), `max\_linear\_mps` (calibrated, placeholder 0.35), `deadman\_hz` (10.0), `cmd\_timeout\_s` (0.4), `min\_duty\_pct` (motor stiction floor, placeholder 15).

### 7.2 Kinematics -> DRIVE conversion

1. `v\_l = v - w \* wheel\_separation / 2`, `v\_r = v + w \* wheel\_separation / 2`.
2. Normalize: `duty = clamp(v\_side / max\_linear\_mps \* 100, -100, 100)`; apply `min\_duty\_pct` floor with sign, zero below a small epsilon.
3. Emit the Pico's existing line protocol. **Claude Code must read the exact grammar from the repo's existing Pico listener and match it verbatim**; the working assumption is a newline-terminated ASCII command carrying signed left/right percentages, e.g. `DRIVE <left> <right>\\n`, plus `STOP\\n` and `PING\\n`. If the real protocol is per-motor (4 motors), map left duty to motors 1+2 and right duty to motors 3+4, respecting the known reversed wiring of motors 3 and 4.
4. There are **no wheel encoders**, so this mapping is open-loop: `max\_linear\_mps` must be measured empirically (drive at 100% for 2 m, time it) and closed-loop correction comes from cuVSLAM odometry through Nav2's controller, not from the bridge.

### 7.3 Watchdog handling (the Pico's 0.5 s failsafe)

* The Pico stops motors if no command arrives for 0.5 s. **motor\_bridge must transmit at a fixed 10 Hz regardless of input**, resending the last commanded duty (including `DRIVE 0 0`) so the watchdog acts as a true dead-man against a crashed Jetson process rather than firing during normal idling.
* Independent bridge-side timeout: if no `/cmd\_vel` arrives for `cmd\_timeout\_s` (0.4 s), decay the command to zero itself (do not coast on stale velocity).
* Lifecycle: send `STOP` on shutdown/SIGINT; send `PING` at 1 Hz and surface round-trip in diagnostics; if the serial port drops, publish an error diagnostic and attempt reconnect with backoff. Never buffer more than the latest command.

### 7.4 Simulation testing before touching the real Pico

1. **Unit tests:** pyserial `loop://` transport; assert kinematics table (pure v, pure w, mixed), deadman cadence (>= 10 msgs/s with zero input), timeout decay, STOP on shutdown.
2. **Fake Pico:** `robo\_sim/fake\_pico.py` opens a pty pair (`socat -d -d pty,raw,echo=0 pty,raw,echo=0` or Python `pty`), parses the protocol exactly like the MicroPython listener, enforces its own 0.5 s watchdog, and logs state transitions. Run the full sim stack with `motor\_bridge serial\_port:=<ptyA>` and confirm the fake Pico's duty log matches the Gazebo DiffDrive motion driven by the same `/cmd\_vel`.
3. **First hardware contact:** robot on blocks, teleop only, watch diagnostics; then floor teleop; only then Nav2.

\---

## 8\. Phased Build Order for Claude Code

Each phase ends with a concrete verification gate. Phases 1-5 need no new hardware; hardware phases are marked \[HW].

**Phase 0: Workspace + interfaces + description.** Create `ros2\_ws`, all 7 custom packages compiling empty, `robo\_interfaces` messages, `robo.urdf.xacro` with placeholder dimensions. Gate: `colcon build` clean; `ros2 launch robo\_description display.launch.py` shows the robot and full static TF tree in RViz.

**Phase 1: Gazebo world + diff drive.** `living\_room.sdf` (furniture, walls), spawn robot, DiffDrive plugin, bridge `/cmd\_vel` + `/clock` + joint states. Gate: `teleop\_twist\_keyboard` (remapped to `/cmd\_vel\_teleop`, through twist\_mux) drives the simulated robot.

**Phase 2: Simulated stereo + rectify + disparity.** Camera sensors, bridge to contract topics, launch both RectifyNodes and stereo\_image\_proc. Gate: `/stereo/points2` renders a sane point cloud of the world in RViz; left/right stamps identical.

**Phase 3: cuVSLAM in sim.** Launch `isaac\_ros\_visual\_slam` on the rectified pair, `use\_sim\_time:=true`. Gate: teleop a loop around the sim room; `map->odom->base\_link` stays consistent, drift visibly corrected on loop closure; compare against `/sim/ground\_truth`.

**Phase 4: Nav2 in sim (milestone: full pipeline).** Nav2 with rolling costmaps fed by `/stereo/points2`, twist\_mux wiring, RViz goal. Gate: **simulated camera -> VSLAM -> Nav2 -> `/cmd\_vel` -> simulated robot navigates to a goal around furniture.** This is the de-risk milestone.

**Phase 5: motor\_bridge against fake Pico.** Implement per Section 7, unit tests + pty fake, run alongside the Phase 4 sim. Gate: fake Pico log shows correct duties, 10 Hz cadence, watchdog never fires during idle, STOP on Ctrl-C.

**Phase 6 \[HW]: Dual CSI bring-up + calibration.** jetson-io "Camera IMX219 Dual", verify both `/dev/video\*`, argus\_stereo\_node publishing the contract topics, stereo checkerboard calibration, update URDF baseline. Gate: live rectified pair + epipolar-aligned disparity on a static scene.

**Phase 7 \[HW]: Real VSLAM.** cuVSLAM on the real rig, robot carried/pushed by hand first. Gate: stable tracking walking a loop; relocalization after covering the lenses.

**Phase 8 \[HW]: Real motors.** motor\_bridge -> real Pico on blocks (teleop), then floor teleop, calibrate `max\_linear\_mps`, then Nav2 short goals in a cleared area. Gate: real robot completes a 3 m goal with obstacle avoidance.

**Phase 9 \[HW]: ESP32 hub.** Firmware (JSON line protocol), `esp32\_bridge`, udev rules for stable `/dev/robo\_pico` / `/dev/robo\_esp32` names, add ToF/us layers to costmaps. Gate: hand in front of ToF marks the local costmap; low obstacle invisible to the cameras is avoided.

**Phase 10: Perception stack.** Port/build the vision-stack nodes (pose, face, id, gesture, manager), identity gating, gesture stop through twist\_mux, "come here" goal relay. Gate: robot stops on your open-palm gesture and ignores the same gesture from someone else.

Suggested Claude Code prompting: one prompt per phase, each pasting this document's Sections 2, 3 and the phase text; commit + push at every gate per your usual workflow.

\---

## 9\. Open Risks and Validation Steps

1. **R1, unsynchronized stereo (the biggest one).** Two independent RPi v2 modules have no hardware sync pin; Argus pairs frames by timestamp (<100 us jitter on capture stamps, but exposure starts are not locked). Under fast rotation this skews stereo matching and cuVSLAM/SGM quality. Validate in Phase 7 with slow motion first; measure left/right stamp deltas. Mitigations if it bites: cap angular velocity in Nav2, drop to 720p30 exposure-matched mode, or swap the rig for an Arducam Camarray-style synchronized dual-IMX219 HAT (single CSI device, minimal architecture change: still one argus/v4l2 source node).
2. **R2, open-loop base with no encoders.** Nav2's controller assumes commanded velocities roughly happen. Carpet, battery sag and stiction will violate that; cuVSLAM odometry closes the loop but at camera latency. Validate in Phase 8; tune `min\_duty\_pct`, velocity smoother limits, and consider a crude per-side gain calibration. If unacceptable, encoders become a hardware follow-up.
3. **R3, Gazebo Fortress performance on the Orin Nano.** Sim + cuVSLAM + Nav2 on one 8GB board may be tight; run Gazebo headless (`gz sim -s`), RViz on the Windows machine over the network (`ROS\_DOMAIN\_ID` shared), reduce sim camera rate to 15 Hz if needed (contract allows rate drop; do not rename topics). Alternative: run Gazebo on the Windows box via WSL2 and keep Isaac ROS nodes on the Jetson; validate DDS discovery across the LAN early (Phase 1).
4. **R4, cuVSLAM tolerance of Gazebo's idealized cameras vs the real rig.** Sim success does not prove real-world tracking (perfect sync, zero noise, textureless sim walls can even make sim *harder*). Phase 7 is the real test; keep `rtabmap\_ros` stereo as the drop-in fallback on identical topics.
5. **R5, exact Pico protocol grammar.** Section 7 assumes `DRIVE l r` semantics; Claude Code must diff against the actual MicroPython listener in the repo (including the motor 3/4 reversal handling) before writing the encoder, and the fake Pico must be generated from the same grammar.
6. **R6, watchdog timing margins.** 10 Hz keep-alive against a 0.5 s timeout gives 5x margin, but USB serial latency spikes under CPU load are unmeasured; Phase 5/8 should log worst-case inter-command gaps under full stack load, and the Pico-side timeout could be relaxed to 1 s if needed.
7. **R7, dual-CSI on JetPack 6.2.1.** The jetson-io dual-IMX219 preset is documented, but forum reports show occasional dual-cam enumeration issues on Orin Nano; validate both `/dev/video\*` nodes and simultaneous 720p30 capture (bandwidth) before building the physical mount.
8. **R8, ISP/exposure divergence between the two cameras.** Independent AE/AWB on left/right hurts stereo matching; lock exposure/gain to the left camera's values via Argus settings during Phase 6 calibration.
9. **R9, VL53L5CX frame layout and ultrasonic crosstalk.** The 8x8 zone-to-point projection and simultaneous ultrasonic firing need bench validation on the ESP32 before trusting costmap marks (stagger the ultrasonic triggers in firmware).
10. **R10, 8GB memory ceiling.** Sim phases load Gazebo alongside the full stack; if OOM appears, that is a sim-only artifact (Gazebo is absent on the real robot), addressed via R3's remote-sim option, not by changing the architecture.

\---

## Appendix: sources consulted for platform-specific claims

[Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html) | [isaac\_ros\_argus\_camera (multi-camera sync, timestamping)](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera) | [Argus stereo sync forum thread](https://forums.developer.nvidia.com/t/argus-camera-stereo-sync/267695) | [Dual IMX219 on Orin Nano (jetson-io dual preset)](https://nvidia-jetson.piveral.com/jetson-orin-nano/how-to-enable-dual-imx219-camera-connection-on-jetson-orin-nano/) | [Dual IMX219 enumeration issues thread](https://forums.developer.nvidia.com/t/jetson-orin-nano-and-imx219-dual-cam/339106) | [IMX219-83 stereo (no hardware sync)](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera) | [Arducam synchronized stereo HAT approach](https://blog.arducam.com/jetson-nano-stereo-camera-sync-issue-arducam/) | [Isaac ROS visual SLAM concepts](https://nvidia-isaac-ros.github.io/concepts/visual_slam/index.html)

