ARG BASE_IMAGE=ubuntu:24.04
# ARG BASE_IMAGE=osrf/ros:jazzy-simulation

####################
# 
# Base Image
# 
# Includes Miniconda, CUDA JIT, and OpenGL and system dependencies for MuJoCo.
# Defaults to Ubuntu, but can be overridden by the `BASE_IMAGE` build arg 
# for example, to use a ROS jazzy simulation image, set BASE_IMAGE=osrf/ros:jazzy-simulation
####################

FROM ${BASE_IMAGE} AS spark-base

ARG WITH_ROS=false
# Specify a miniconda3. (see versions here https://repo.anaconda.com/miniconda/)
ARG CONAD_VERSION=py313_26.1.1-1
# ARG CONAD_VERSION=latest

# Environment Variables for Conda and CUDA
ENV DEBIAN_FRONTEND=noninteractive
ARG CONDA_PREFIX=/opt/conda
ENV CONDA_PLUGINS_AUTO_ACCEPT_TOS=yes
ENV CUDA_HOME=/usr/local/cuda
ENV PATH="${CONDA_PREFIX}/bin:${CUDA_HOME}/bin:${PATH}"
# for GPU Rendering
ENV LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${CUDA_HOME}/compat"
ENV LIBGL_ALWAYS_SOFTWARE=0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility
# for MuJoCo
ENV MUJOCO_GL=glx
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
# IMPORTANT: for WSLg OpenGL support in WSL2 with NVIDIA GPU
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH}

# Install OpenGL and system dependencies for MuJoCo
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    wget \
    git \
    libgl1 \
    libglvnd0 \
    libglx0 \
    mesa-utils \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libgtk-3-0 \
    && rm -rf /var/lib/apt/lists/*
    
# Install NVIDIA CUDA toolkit Numba and CUDA JIT-compilation.
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb && \
    dpkg -i /tmp/cuda-keyring.deb && \
    rm /tmp/cuda-keyring.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
    cuda-compiler-12-6 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-${CONAD_VERSION}-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -u -p /opt/conda && \
    rm /tmp/miniconda.sh && \
    conda init --all --system


# Install ROS 2 dependencies
# Uncomment the following and add desired ROS dependencies
# 
# RUN if [ "${WITH_ROS}" = "true" ]; then \
#     apt-get update && apt-get install -y --no-install-recommends \
#     ros-${ROS_DISTRO}-rviz2 \
#     ros-${ROS_DISTRO}-rviz-default-plugins \
#     ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*; \
#     fi

# Update bash config to source ROS setup script if ROS is included in the image
RUN if [ "${WITH_ROS}" = "true" ]; then \
        echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc; \
    fi

CMD [ "/bin/bash" ]


####################
#
# Runtime Image 
# 
# Clones the Spark repository main branch and runs the installation script 
# to set up the Spark application and its dependencies.
####################

FROM spark-base AS spark
ARG WITH_ROS=false
ARG PYTHON_VERSION=3.10
ARG USE_REMOTE_SRC=false

WORKDIR /home/spark

# Copy SPARK local repo
COPY . /home/spark/spark
# Optionally, instead use the remote repo source hosted on Github to install Spark
# if USE_REMOTE_SRC is set "true"
RUN if [ "${USE_REMOTE_SRC}" = "true" ]; then \
rm -rf /home/spark/spark && \
git clone -b main https://github.com/intelligent-control-lab/spark.git spark; \
fi

# clean conda cache and install SPARK
# RUN conda clean --all -y 2>/dev/null
RUN  cd spark && \
chmod +x install.sh && \
./install.sh \
--name spark \
--python ${PYTHON_VERSION} \
# Install ROS dependencies if WITH_ROS is true
$(if [ "${WITH_ROS}" = "true" ]; then echo "--ros"; fi)

# Create a new user named 'spark'
# RUN useradd -ms /bin/bash spark
# switch to the spark user
# RUN chown -R spark:spark /home/spark /opt/conda
# USER spark

# Update bash config to activate the conda environment on login
RUN conda init --all 
RUN echo "conda activate spark" >> /root/.bashrc

WORKDIR /home/spark/spark

ENV DEBIAN_FRONTEND=readline

