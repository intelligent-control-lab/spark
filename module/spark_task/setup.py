from setuptools import setup, find_packages

setup(
    name="spark_task",
    version="1.0.0",
    description="Robot task manager.",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "numpy",  # Add your library dependencies here
    ],
    python_requires=">=3.8",
)