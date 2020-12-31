##  OpenVSLAM setup on MacOS

For Linux setup, please see https://openvslam.readthedocs.io/en/master/docker.html.

- Clone OpenVSLAM https://github.com/xdspacelab/openvslam

- Build docker image
```
cd /path/to/openvslam
docker build -t openvslam-socket -f Dockerfile.socket .
```

- Build docker image for SocketViewer
```
cd /path/to/openvslam
cd viewer
docker build -t openvslam-server .
```

- Run SocketViewer container
```
docker run --rm -it --name openvslam-server -p 3001:3001 openvslam-server
```

- Visit http://localhost:3001

- Add SocketViewer container IP to OpenVSLAM config

Get IP
```
docker inspect openvslam-server | grep -m 1 \"IPAddress\" | sed 's/ //g' | sed 's/,//g'
```

Add IP to video config.yaml like so
```
SocketPublisher.server_uri: "http://{IP_ADDRESS}:3000"
```

- Run OpenVSLAM container
```
docker run --rm -it --name openvslam-socket --volume `pwd`/data:/openvslam/build/data openvslam-socket
```

- Map/localize https://openvslam.readthedocs.io/en/master/simple_tutorial.html

Example of image localization:
```
./run_image_localization -v ./data/orb_vocab/orb_vocab.dbow2 -i ./data/aist_living_lab_2 -c ./data/aist_living_lab_2/config.yaml -p ./data/map.msg
```