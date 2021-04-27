import glob
import sys
import os
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import networkx as nx
import numpy as np


client = carla.Client('localhost', 2000)
client.set_timeout(15.0)

def get_map_graph():
    # get CARLA map waypoints
    world = client.get_world()
    carla_map = world.get_map()
    topology = carla_map.get_topology()

    # create graph from waypoints
    G = nx.DiGraph()

    for waypoint_pair in topology:
        w0 = waypoint_pair[0]
        w1 = waypoint_pair[1]
        w0_2d_loc = np.array([w0.transform.location.x, w0.transform.location.y])
        w1_2d_loc = np.array([w1.transform.location.x, w1.transform.location.y])
        dist = np.linalg.norm(w0_2d_loc - w1_2d_loc)
        G.add_node(w0.id, position=w0_2d_loc, color='black', size=10)
        G.add_node(w1.id, position=w1_2d_loc, color='black', size=10)
        G.add_edge(w0.id, w1.id, weight=dist)

    return G
