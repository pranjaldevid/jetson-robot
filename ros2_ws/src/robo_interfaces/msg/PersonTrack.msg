uint32 track_id
bool is_primary_user
float32 identity_confidence
geometry_msgs/PoseStamped position      # camera frame; z from disparity when available
float32[] keypoints                     # 17x3 (x,y,conf) flattened, COCO order
