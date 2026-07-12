import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robo_navigation"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pranjal",
    maintainer_email="pranjaltikhe05@gmail.com",
    description="Nav2 params and twist_mux config for Companion ROBO (Nav2 launch wrapper itself is scaffold only until Phase 4).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
