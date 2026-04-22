from setuptools import find_packages, setup


package_name = "g1_bridge"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/bridge_config.yaml"]),
        (f"share/{package_name}/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="plate",
    maintainer_email="plate@example.com",
    description="DDS-to-ROS 2 bridge for Unitree G1 low-level state and command topics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sdk_bridge_node = g1_bridge.sdk_bridge_node:main",
        ],
    },
)
