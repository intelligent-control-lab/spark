from pathlib import Path

from setuptools import find_namespace_packages, setup


README = Path(__file__).with_name("README.md").read_text(encoding="utf-8")
RESOURCE_ROOT = Path(__file__).with_name("resources")
RESOURCE_DATA = {}
for resource in RESOURCE_ROOT.rglob("*"):
    if resource.is_file():
        destination = (
            Path("share/spark_robot/resources") / resource.relative_to(RESOURCE_ROOT).parent
        )
        source = resource.relative_to(Path(__file__).parent)
        RESOURCE_DATA.setdefault(str(destination), []).append(str(source))

setup(
    name="spark_robot",
    version="2.0.0",
    description="SPARK robot models, configurations, resources, and kinematics.",
    license="MIT",
    url="https://github.com/intelligent-control-lab/spark",
    long_description=README,
    long_description_content_type="text/markdown",
    packages=find_namespace_packages(include=["spark_robot*"]),
    include_package_data=True,
    package_data={
        "spark_robot.fanuc_lrmate200id.config": ["*.json"],
        "spark_robot.galaxea_r1lite.config": ["*.json"],
        "spark_robot.kinova_gen3.config": ["*.json"],
        "spark_robot.kuka_iiwa14.config": ["*.json"],
    },
    install_requires=[
        "numpy>=2,<3",
        "scipy",
        "pin>=4.1,<5",
        "spark_utils==2.0.0",
    ],
    python_requires=">=3.10",
    data_files=sorted(RESOURCE_DATA.items()),
)
