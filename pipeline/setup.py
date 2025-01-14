from setuptools import setup, find_packages

setup(
    name="spark_pipeline",
    version="1.0.0",
    description="Main task runner.",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "numpy",  # Add your library dependencies here
    ],
    python_requires=">=3.8",
)