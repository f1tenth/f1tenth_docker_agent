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
A template for an agent Dockerfile is provided as ```agent.Dockerfile``` from this repo.

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

In the template dockerfile (commented out), ```CMD``` is used as an example to use a ROS launch file as the entry point command to the container. Depending on your code, you could either use a ros command (e.g. roslaunch), or a bash script to your liking.

## Setting up a race locally
For all the benchmarks, we'll be using docker compose to configure and bring up the docker containers. You can find an overview of docker compose [here](https://docs.docker.com/compose/).

This repo is set up to work with the ```f1tenth_gym_ros``` and the ```f1tenth_gym``` repos. The ```start.sh``` bash script will handle cloning and keeping these two repos up to date. For different benchmarks, environment variables have to be changed in ```docker-compose.yml```. Follow the specific instructions in each of the following section to build and bring up the containers. After you've changed the variables according to the benchmark that you're running, you could run ```start.sh``` to build and run the containers. Note that you might need ```sudo``` depending on your specific setup.

### Benchmark I: Single Vehicle Timed Trials
To use this race scenario, you'll need to change the ```RACE_SCENARIO``` variable in ```docker-compose.yml``` to 0. Note that only the topics corresponding to ```EGO_ID``` should be used in this benchmark.

### Benchmark II: Single Vehicle Obstacle Avoidance
For this scenario, you'll also need to change the ```RACE_SCENARIO``` variable in ```docker-compose.yml``` to 0. Additionally, you'll have to change the default map to a map with obstacles. Note that only the topics corresponding to ```EGO_ID``` should be used in this benchmark.

### Benchmark III: Two Vehicle Head-to-head
To use this race scenario, you'll need to change the ```RACE_SCENARIO``` variable in ```docker-compose.yml``` to 1.

### Changing the map
You can change the map that the sim uses by changing the ```RACE_MAP_PATH``` and the ```RACE_MAP_IMG_EXT``` variables accordingly. Keep in mind that this variable is the path to the map **inside** the sim docker container. The easiest way to add a map and to avoid changing the Dockerfile is to put the map image and yaml file into the ```maps``` directory of ```f1tenth_gym_ros``` that's cloned by ```start.sh```.

### Available topics
The easiest way to check the available topics is to run ```rostopic list``` after you've ran ```start.sh```.

### Adding an agent
After you follow the steps mentioned [here](https://github.com/f1tenth/f1tenth_docker_agent#creating-your-own-agent-dockerfiles), you can change the dockerfile used in ```docker-compose.yml``` for services ```agent1``` and ```agent2```. Make sure that your code subscribe to the correct topics determined by the ```EGO_ID``` and ```OPP_ID``` environment variables in ```docker-compose.yml```. For example, by default, the available topics are ```/ego_id/scan``` etc. When your agent is evaluated against other agents, the race will have two rollouts where the starting position of the cars are flipped by flipping ```EGO_ID``` and ```OPP_ID```. Note that the starting poses set in the environment variables will NOT be changed during the race, only the starting position of the cars are flipped. 

### Starting the race and visualization
You can start RVIZ on your host system by running ```roscore``` in one terminal, and ```rviz``` in another terminal. Once the GUI shows up, you can load a RVIZ config by *File* -> *Open Config*, and then select ```your_path_to/f1tenth_docker_agent/f1tenth_gym_ros/launch/gym_bridge.rviz```. And to start the race once you have the agents containers set, run ```start.sh```.

## Submitting your agent
### Setting up DockerHub
We'll be using DockerHub to host and submit your docker images. There are two ways to host a docker image on DockerHub and we'll go over both of them.

First of all, you'll need to create an account [here](https://hub.docker.com/signup). Then you can follow the Quick Start guide [here](https://docs.docker.com/docker-hub/) to use the first method to host your image on DockerHub with ```docker push```.

You could also set up automated builds following the guide [here](https://docs.docker.com/docker-hub/builds/).

Note that with automated builds, your Dockerfile is transparent on public repositories to other users. If you wish to have your source code hidden, use ```docker push```.

### Submitting your docker image
A submission portal (for IROS 2020) will be open at [this event page](https://iros2020.f1tenth.org). You'll need to provide your assigned unique submission passcode, your DockerHub repo link, and the unique agent id that will be used in assigning ROS topic names.