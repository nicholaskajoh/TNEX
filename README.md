# TNEX [WIP]
TNEX is a program that can drive a vehicle in the [CARLA simulator](https://carla.org). It employs similar techniques used in state-of-the-art self-driving cars, howbeit with much simpler algorithms. The goal of this project is to learn how autonomous vehicles work by building one.

## Architecture
![](components.jpg)

## Setup
### CARLA
- Download CARLA https://github.com/carla-simulator/carla/releases

- Run CARLA
```sh
./CarlaUE4.sh -opengl -quality-level=Low -fps=10
```

### ROS
- Install ROS Melodic http://wiki.ros.org/melodic/Installation/Ubuntu

- Change workspace
```sh
cd path/to/tnex
source `pwd`/devel/setup.bash # save this in your ~/.bashrc
```

- Install Rosbridge
```sh
sudo apt-get install ros-melodic-rosbridge-suite
```

- Build TNEX
```sh
catkin_make
```

- Run TNEX
```
roslaunch tnex_driver main.launch
```

### Webviz
- Pull Docker image
```sh
docker pull cruise/webviz
```

- Run Webviz
```sh
docker run --name tnex-webviz -p 8080:8080 cruise/webviz
```