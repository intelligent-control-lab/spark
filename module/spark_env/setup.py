from pathlib import Path

from setuptools import find_namespace_packages, setup


README = Path(__file__).with_name("README.md").read_text(encoding="utf-8")


setup(
    name="spark_env",
    version="2.0.0",
    description="SPARK agent/task environment coordination.",
    license="MIT",
    url="https://github.com/intelligent-control-lab/spark",
    long_description=README,
    long_description_content_type="text/markdown",
    packages=find_namespace_packages(include=["spark_env*"]),
    include_package_data=True,
    install_requires=[
        "numpy>=2,<3",
        "spark_agent==2.0.0",
        "spark_task==2.0.0",
        "spark_utils==2.0.0",
    ],
    python_requires=">=3.10",
)
