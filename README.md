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

## Setting up a race locally

## Submitting your agent