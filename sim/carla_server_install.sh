#!/bin/sh

# Set up the Debian repository in the system.
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
sudo apt-get update

# Install package.
sudo apt-get install carla-simulator=0.9.13
