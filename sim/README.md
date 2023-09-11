# Carla simulator setup
> This setup was tested on a GCP VM (with 1 NVIDIA L4 GPU) running Ubuntu 22.04.

## Install NVIDIA drivers
```sh
sudo apt install -y ubuntu-drivers-common
sudo ubuntu-drivers autoinstall
```

## Install Docker
```sh
sudo apt update
sudo apt install -y apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
apt-cache policy docker-ce
sudo apt install -y docker-ce

sudo usermod -aG docker ${USER}
# Log out and log back in.
id -nG

# Test installation.
docker info
```

## Install NVIDIA Container Toolkit
```sh
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Test installation.
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu20.04 nvidia-smi # Should display GPU info.
```

## Install Carla
```sh
docker pull carlasim/carla:0.9.14
docker images

docker run \
   -d \
   --name carla-server \
   --privileged \
   --gpus all \
   --net=host \
   -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
   carlasim/carla:0.9.14 /bin/bash ./CarlaUE4.sh -RenderOffScreen -nosound

# Test installation
docker logs --follow --since 1h carla-server

# Check GPU stats
sudo apt-get install -y gpustat
gpustat -cpi -P
```

## Configure Carla (from client machine)
```sh
docker pull carlasim/carla:0.9.14
docker images

docker run \
   -dit \
   --name carla-client \
   -u 0 \
   -e DISPLAY=$DISPLAY \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   carlasim/carla:0.9.14 /bin/bash
docker exec -it carla-client bash

# Run in container...
cd PythonAPI/examples

apt update && apt install -y python3.7 python3-pip libjpeg-dev libtiff-dev fontconfig
python3.7 -m pip install --upgrade pip
python3.7 -m pip install -r requirements.txt

python3.7 generate_traffic.py --host xx.xx.xx.xx --number-of-vehicles 50 --number-of-walkers 100
python3.7 manual_control.py --host xx.xx.xx.xx
```
