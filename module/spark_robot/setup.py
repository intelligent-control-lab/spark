from setuptools import setup, find_packages

setup(
    name="spark_robot",
    version="1.0.0",
    description="Robot lib for spark",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "numpy",  # Add your library dependencies here
    ],
    python_requires=">=3.8",
    package_data={
        # Include URDF and STL files in the installation
        "spark_robot": [
            "resources/g1/*.urdf",
            "resources/g1/meshes/*.STL",
        ]
    },
)
