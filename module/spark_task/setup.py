from pathlib import Path

from setuptools import find_namespace_packages, setup


README = Path(__file__).with_name("README.md").read_text(encoding="utf-8")

setup(
    name="spark_task",
    version="2.0.0",
    description="SPARK task definitions and goal management.",
    license="MIT",
    url="https://github.com/intelligent-control-lab/spark",
    long_description=README,
    long_description_content_type="text/markdown",
    packages=find_namespace_packages(include=["spark_task*"]),
    include_package_data=True,
    install_requires=[
        "numpy>=2,<3",
        "spark_robot==2.0.0",
        "spark_utils==2.0.0",
    ],
    extras_require={
        "ros1": ["catkin-pkg", "rospkg"],
    },
    python_requires=">=3.10",
)
