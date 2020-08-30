# F1TENTH Competition Docker Agent Template

This repo contains a standardized template for creating your own Docker containerized agent submission to F1TENTH competitions.

## Overview
This repo serves as a guide to get participants started on creating their own agent submissions to the F1TENTH competitions.

The overall structure of a race is shown in the diagram below. The racing agents will be fully contained in their own docker containers as well as the simulation environment. A ROS bridge is created so that participants could use the ROS interface to receive sensor information and publish drive messages.

<img src="media/structure.png">

In the next few sections, we'll go over what you'll need to install to get started, how to create a docker image for your agent, and how to set up an experimental race on your own machine.

## Installing Dependencies
### Docker
Currently, only docker on Ubuntu is officially tested and supported. To install Docker Engine on Ubuntu, follow the guide [here](https://docs.docker.com/engine/install/ubuntu/). You can then follow the steps [here](https://docs.docker.com/engine/install/linux-postinstall/) to configure Linux hosts to work better with Docker. Without these post-installation steps, ```sudo``` will be needed every time the ```docker``` command is ran.

### Docker Compose
Compose is a tool for defining and running multi-container Docker applications. With Compose, you can use a YAML file to configure and bring up multiple Docker containers. This will become very handy when we need to configure and bring up the simulation container and the agent containers. You can install Compose following the steps [here](https://docs.docker.com/compose/install/) after you've installed the Docker engine.

## Creating your own agent Dockerfiles
### Base image
The template Dockerfile pulls from the ```melodic-robot-bionic``` version of the official ROS docker image, which is based on an Ubuntu Bionic docker image. So make sure your dependencies support Ubuntu Bionic.

### Adding your dependencies
There are several options for adding dependencies to your agent docker container. You could either install via ```apt```, ```pip```, or build from source.

For ```apt``` or ```pip``` dependencies, add your dependencies to the end of the existing list of packages in the template Dockerfile.

For dependencies that you'll be building from source, use the ```RUN``` syntax for Dockerfiles. See more details [here](https://docs.docker.com/engine/reference/builder/#run).

### Creating a workspace for your ROS package
You'll need to create a workspace for your ROS package just like you would on your host system. The template Dockerfile already contains a line of command that creates a workspace at ```/catkin_ws``` in the container.

### Copying over or cloning your source code
You can copy over your source code into the container using two methods. You could either use the ```COPY``` dockerfile syntax and copy a directory from the build context to a directory inside the container. (More details [here](https://docs.docker.com/engine/reference/builder/#copy))

You could also use the ```RUN``` syntax to clone a Github repo of your package into the ```src``` directory of the catkin workspace we've created.

### Building your package
```catkin_make``` will be used to build your ROS packages. The ```RUN``` command is used to bring up ```bash``` so that the usual ```source``` command could be used. See an example in the template Dockerfile.

### Creating an entry point command
Either ```CMD``` or ```ENTRYPOINT``` could be used to define an entrypoint command that gets executed when the container is brought up. You can find more detials on how the two commands interact [here](https://docs.docker.com/engine/reference/builder/#understand-how-cmd-and-entrypoint-interact).

## Setting up a race locally

## Submitting your agent