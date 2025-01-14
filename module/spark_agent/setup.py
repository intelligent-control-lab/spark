from setuptools import setup, find_packages

setup(
    name="spark_agent",
    version="1.0.0",
    description="Interface for the real robot and the simulation.",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "numpy",  # Add your library dependencies here
    ],
    python_requires=">=3.8",
)