FROM ros:noetic

RUN apt-get update
RUN apt-get -y install git
RUN apt-get -y install ros-noetic-cv-bridge
RUN apt-get -y install python-is-python3
RUN apt-get -y install ros-noetic-image-transport ros-noetic-image-transport-plugins
RUN apt-get -y install ros-noetic-turtlesim

RUN /bin/bash -c "source ros_entrypoint.sh"