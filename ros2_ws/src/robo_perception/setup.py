from setuptools import find_packages, setup

package_name = "robo_perception"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pranjal",
    maintainer_email="pranjaltikhe05@gmail.com",
    description="Perception nodes for Companion ROBO (scaffold only, not implemented until Phase 10).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
