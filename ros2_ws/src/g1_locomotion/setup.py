from setuptools import find_packages, setup


package_name = "g1_locomotion"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/locomotion_params.yaml"]),
        (f"share/{package_name}/launch", ["launch/locomotion.launch.py"]),
        (
            f"share/{package_name}/policies",
            ["policies/README.md", "policies/g1_motion.pt"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="plate",
    maintainer_email="plate@example.com",
    description="Unitree G1 locomotion policy runner with RL and PD fallback control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "locomotion_node = g1_locomotion.locomotion_node:main",
        ],
    },
)
