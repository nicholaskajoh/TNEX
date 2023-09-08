#!/bin/sh

# Start Carla server.
cd /opt/carla-simulator/bin/ ./CarlaUE4.sh -RenderOffScreen

# Install Python client.
pip3 install --upgrade pip
pip3 install carla

# Setup world.
#cd /opt/carla-simulator/bin/PythonAPI/util
#./config.py --map Town05 # Change map
#./config.py --weather ClearNoon # Change weather

cd /opt/carla-simulator/bin/PythonAPI/examples
python3 -m pip install -r requirements.txt
python3 generate_traffic.py --number-of-vehicles 50 --number-of-walkers 100
