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