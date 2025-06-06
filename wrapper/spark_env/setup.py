from setuptools import setup, find_packages

setup(
    name="spark_env",
    version="0.1",
    description="Env wrapper.",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "numpy",  # Add your library dependencies here
    ],
    python_requires=">=3.8",
)