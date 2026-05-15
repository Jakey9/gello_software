from setuptools import find_packages, setup
import glob

package_name = "lite6_gello_state_publisher"

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
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Jake Tan",
    maintainer_email="jake.tan@example.com",
    description="Publishes GELLO joint states for Lite6 using Zhonglin serial bus servos.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gello_publisher = lite6_gello_state_publisher.gello_publisher:main",
        ],
    },
)
