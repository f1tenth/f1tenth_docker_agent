# Base image
FROM ros:melodic-robot-bionic

# Update apt repo and pip2
RUN apt-get update --fix-missing && \
    apt-get install -y \
    python-pip

# Install apt dependencies, add your apt dependencies to this list
RUN apt-get install -y git \
                       build-essential \
                       cmake \
                       vim \
                       ros-melodic-ackermann-msgs

# Upgrade pip
RUN pip install --upgrade pip

# Install pip dependencies, add your pip dependencies to this list
RUN pip install numpy==1.16.0 \
                scipy==1.2.0 \
                pyyaml

# Creating a catkin workspace
RUN mkdir -p /catkin_ws/src

# Clone or copy over your source code

# Copying
# COPY ./your_package /catkin_ws/src/

# Cloning
# RUN cd /catkin_ws/src/ && \
#     git clone https://github.com/your_user/your_repo.git

# Building your ROS packages
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"

# Setting entry point command that runs when the container is brought up
# CMD source /catkin_ws/devel/setup.bash; roslaunch --wait exp_pkg exp_launch.launch