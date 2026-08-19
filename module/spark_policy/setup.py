from pathlib import Path

from setuptools import find_namespace_packages, setup


README = Path(__file__).with_name("README.md").read_text(encoding="utf-8")

setup(
    name="spark_policy",
    version="2.0.0",
    description="Reusable SPARK planning, control, estimation, and safety policies.",
    license="MIT",
    url="https://github.com/intelligent-control-lab/spark",
    long_description=README,
    long_description_content_type="text/markdown",
    packages=find_namespace_packages(include=["spark_policy*"]),
    include_package_data=True,
    package_data={
        "spark_policy": [
            "control/whole_body/unitree_g1/sport/resources/*.pt",
            "control/whole_body/unitree_g1/wbt/runtime/LICENSE.OpenWBT",
            "control/whole_body/unitree_g1/wbt/runtime/NOTICE.OpenWBT.md",
            "control/whole_body/unitree_g1/wbt/runtime/configs/*.yaml",
            "control/whole_body/unitree_g1/wbt/runtime/ckpts/*.onnx",
            "safety/monitoring/collision/resources/onnx_model/*.onnx",
        ],
    },
    install_requires=[
        "numpy>=2,<3",
        "scipy",
        "PyYAML",
        "casadi",
        "meshcat",
        "osqp",
        "proxsuite",
        "spark_robot==2.0.0",
        "spark_utils==2.0.0",
    ],
    extras_require={
        "learned": [
            "onnx",
            "onnx2torch",
            "onnxruntime",
            "tensorboard",
            "torch",
        ],
    },
    python_requires=">=3.10",
)
