#!/bin/sh

#  Set up the Debian repository in the system.
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9 sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"

# Update the Debian package index.
sudo apt-get update

# In this case, "0.9.14" refers to a CARLA version, and "1" to the Debian revision.
sudo apt-get install carla-simulator=0.9.14-1
