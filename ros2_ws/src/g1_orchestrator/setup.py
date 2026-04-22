from setuptools import find_packages, setup


package_name = "g1_orchestrator"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/orchestrator_params.yaml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/orchestrator.launch.py",
                "launch/phase2_stack.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="plate",
    maintainer_email="plate@example.com",
    description="Locomotion state orchestrator for Unitree G1 ROS 2 control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator_node = g1_orchestrator.orchestrator_node:main",
        ],
    },
)
