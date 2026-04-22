from setuptools import setup


package_name = "g1_bringup"


setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/full_system.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="plate",
    maintainer_email="plate@example.com",
    description="Combined bringup launch files for the Unitree G1 ROS 2 stack.",
    license="Apache-2.0",
)
