# Use the official ROS2 Foxy base image
FROM ros:foxy

# Set the working directory in the container
WORKDIR /workspace

# Install system dependencies
# Replace '<your-system-dependencies-here>' with any system packages you need. For example: 'git', 'vim', 'wget', etc.

# RUN apt-get update && apt-get install -y \
#     <your-system-dependencies-here> \
#     && rm -rf /var/lib/apt/lists/*

# Install ROS2 dependencies
# Replace '<your-ros2-packages-here>' with any ROS2 packages you need. For example: 'ros-foxy-tf2', 'ros-foxy-navigation2', etc.

# RUN apt-get update && apt-get install -y \
#     <your-ros2-packages-here> \
#     && rm -rf /var/lib/apt/lists/*

# Install C++ libraries
# Replace '<your-cpp-libraries-here>' with any C++ libraries you need. For example: 'libeigen3-dev', 'libboost-all-dev', etc.

RUN apt-get update && apt-get install -y \
    ros-foxy-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Copy the project into the container
COPY . .

# Install any Python dependencies
# If you have a 'requirements.txt' file for Python packages, uncomment the next two lines.
# COPY requirements.txt .
# RUN pip3 install --no-cache-dir -r requirements.txt

# Build the ROS2 project
RUN . /opt/ros/foxy/setup.sh && colcon build

# Source the ROS2 setup script
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set the default command to execute when starting the container
CMD ["bash"]
