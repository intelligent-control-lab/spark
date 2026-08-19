from pathlib import Path

from setuptools import find_namespace_packages, setup


README = Path(__file__).with_name("README.md").read_text(encoding="utf-8")

setup(
    name="spark_utils",
    version="2.0.0",
    description="Shared numerical, geometry, logging, and configuration utilities for SPARK.",
    license="MIT",
    url="https://github.com/intelligent-control-lab/spark",
    long_description=README,
    long_description_content_type="text/markdown",
    packages=find_namespace_packages(include=["spark_utils*"]),
    include_package_data=True,
    install_requires=[
        "numba",
        "numpy>=2,<3",
        "tensorboardX",
    ],
    python_requires=">=3.10",
)
