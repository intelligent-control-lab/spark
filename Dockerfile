# syntax=docker/dockerfile:1

ARG BASE_IMAGE=python:3.10-slim-bookworm

FROM ${BASE_IMAGE} AS spark

ARG SPARK_VERSION=2.0.0

LABEL org.opencontainers.image.title="SPARK" \
      org.opencontainers.image.description="Safe Protective and Assistive Robot Kit" \
      org.opencontainers.image.source="https://github.com/intelligent-control-lab/spark" \
      org.opencontainers.image.version="${SPARK_VERSION}"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    VIRTUAL_ENV=/opt/spark-venv \
    PATH=/opt/spark-venv/bin:${PATH} \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    OMP_NUM_THREADS=4 \
    MUJOCO_GL=glx

# Keep the base usable for the numerical core, MuJoCo/X11 rendering, ROS base
# images, and the optional Isaac profile. NVIDIA drivers are supplied by the
# host container runtime; the default image does not install a CUDA toolkit.
RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        bash \
        build-essential \
        ca-certificates \
        curl \
        git \
        libdbus-1-3 \
        libegl1 \
        libfontconfig1 \
        libfreetype6 \
        libgl1 \
        libglib2.0-0 \
        libglx0 \
        libosmesa6 \
        libsm6 \
        libx11-6 \
        libx11-xcb1 \
        libxcb-cursor0 \
        libxcb-xinerama0 \
        libxcursor1 \
        libxext6 \
        libxi6 \
        libxinerama1 \
        libxkbcommon-x11-0 \
        libxrandr2 \
        libxrender1 \
        libxxf86vm1 \
        pkg-config \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/spark

COPY docker/entrypoint.sh /usr/local/bin/spark-entrypoint
RUN chmod 0755 /usr/local/bin/spark-entrypoint

COPY . .

# Use the repository's installer as the single source of truth. Boolean build
# arguments map directly to the optional dependency switches documented by
# install.sh; no optional stack is silently added to the default image.
ARG SPARK_PYTHON=3.10
ARG SPARK_PROFILE=mujoco
ARG SPARK_WITH_ROS=false
ARG SPARK_WITH_TORCH=false
ARG SPARK_WITH_LEARNED=false
ARG SPARK_WITH_UNITREE_SDK=false
ARG SPARK_WITH_DEV=false
RUN install_args=( \
        --env-manager venv \
        --path "${VIRTUAL_ENV}" \
        --profile "${SPARK_PROFILE}" \
        --python "${SPARK_PYTHON}" \
    ) \
    && if [[ "${SPARK_WITH_ROS}" == "true" ]]; then install_args+=(--ros); fi \
    && if [[ "${SPARK_WITH_TORCH}" == "true" ]]; then install_args+=(--torch); fi \
    && if [[ "${SPARK_WITH_LEARNED}" == "true" ]]; then install_args+=(--learned); fi \
    && if [[ "${SPARK_WITH_UNITREE_SDK}" == "true" ]]; then \
        install_args+=(--unitree-sdk); \
    fi \
    && if [[ "${SPARK_WITH_DEV}" == "true" ]]; then install_args+=(--dev); fi \
    && ./install.sh "${install_args[@]}" \
    && python -c 'import spark_agent, spark_env, spark_pipeline, spark_policy, spark_robot, spark_task, spark_utils'

ENTRYPOINT ["/usr/local/bin/spark-entrypoint"]
CMD ["bash"]
