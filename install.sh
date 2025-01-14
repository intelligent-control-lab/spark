#!/bin/bash

# Initialize Conda
CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"

# Function to report errors and exit
function handle_error {
    echo "Error: $1"
    exit 1
}

# Default values
INSTALL_ROS=false
ENV_NAME=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ros)
            INSTALL_ROS=true
            shift
            ;;
        --name)
            shift
            # Check if environment name is provided
            if [[ -z "$1" || "$1" == "--"* ]]; then
                handle_error "You must provide an environment name after --name."
            fi
            ENV_NAME="$1"
            shift
            ;;
        *)
            handle_error "Unknown option: $1"
            ;;
    esac
done

# Check if ENV_NAME was provided
if [[ -z "$ENV_NAME" ]]; then
    handle_error "You must provide an environment name using --name env_name"
fi

# Step 1: Create Conda environment
echo "Step 1: Creating Conda environment: $ENV_NAME"
conda create --name $ENV_NAME "python=3.10" -y || handle_error "Failed to create Conda environment $ENV_NAME using 'conda create --name $ENV_NAME python=3.10'"

# Step 2: Activate the environment
echo "Step 2: Activating Conda environment: $ENV_NAME"
conda activate $ENV_NAME || handle_error "Failed to activate Conda environment $ENV_NAME using 'conda activate $ENV_NAME'"

# Step 3: Install Pinocchio
echo "Step 3: Installing Pinocchio"
conda install -c conda-forge pinocchio=3.2.0 -y || handle_error "Failed to install Pinocchio using 'conda install -c conda-forge pinocchio=3.2.0'"

# Step 4: Install numpy=2.0
echo "Step 4: Installing numpy=2.0"
conda install numpy=2.0 -y || handle_error "Failed to install numpy=2.0 using 'conda install numpy=2.0'"

# Step 5: Install core Python packages
echo "Step 5: Installing core Python packages"

# Declare the array of core Python packages
core_packages=("scipy" "numba" "casadi" "matplotlib" "mujoco" "pyyaml" "osqp" "tensorboardX")

# Iterate through each package in the array
for package in "${core_packages[@]}"; do
    echo "Installing $package..."
    pip install "$package" || handle_error "Failed to install Python package $package using 'pip install $package'"
done

# Step 6: Install spark_* modules
echo "Step 6: Installing spark_* modules"

# Declare the array of spark modules
spark_modules=(
    "module/spark_agent"
    "module/spark_policy"
    "module/spark_robot"
    "module/spark_safe"
    "module/spark_task"
    "module/spark_utils"
    "pipeline/"
    "wrapper/spark_algo"
    "wrapper/spark_env"
)

# Iterate through each module in the array
for module in "${spark_modules[@]}"; do
    echo "Installing module: $module"
    cd "$module" || handle_error "Failed to enter directory $module"
    pip install -e . || handle_error "Failed to install module $module using 'pip install -e .'"
    cd - || handle_error "Failed to return to parent directory from $module"
done

echo "All spark_* modules installed successfully!"

# Step 7: Conditionally install ROS dependencies
if $INSTALL_ROS; then
    echo "Step 7: Installing ROS dependencies"

    # Declare the array of ROS packages
    ros_packages=("yaml" "rospkg" "catkin_pkg")

    # Iterate through each package in the array
    for package in "${ros_packages[@]}"; do
        echo "Installing $package..."
        conda install conda-forge::"$package" -y || handle_error "Failed to install ROS package $package using 'conda install -c conda-forge $package'"
    done

    echo "ROS dependencies installed successfully!"
else
    echo "Step 7: Skipping ROS dependencies installation. Use '--ros' if ROS-related utilities are needed."
fi

echo "Installation process completed successfully!"
