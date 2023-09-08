# ROS

https://www.ros.org

## Setup with Docker

- Build image
```
docker build -t tnex-ros:noetic .
```

- Start container
```
docker run --name tnex-ros-noetic -it --volume `pwd`/catkin_ws:/root/catkin_ws tnex-ros:noetic
```

- Create catkin workspace
```
docker exec -it tnex-ros-noetic bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
catkin_make
```

- Start ROS master node
```
roscore
```

- Create package
```
cd ~/catkin_ws/src
catkin_create_pkg package_name std_msgs rospy
```