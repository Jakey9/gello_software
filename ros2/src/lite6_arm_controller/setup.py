from setuptools import find_packages, setup
import glob

package_name = "lite6_arm_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jake Tan",
    maintainer_email="jake.tan@example.com",
    description="Lite6 arm controller: subscribes to GELLO joint states and commands the robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_position_controller = lite6_arm_controller.joint_position_controller:main",
            "gripper_controller = lite6_arm_controller.gripper_controller:main",
        ],
    },
)
