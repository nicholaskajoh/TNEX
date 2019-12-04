#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('%s/dist/carla-*%d.%d-%s.egg' % (
        os.path.dirname(os.path.realpath(__file__)),
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from matplotlib import pyplot as plt

import time
import networkx as nx
import numpy as np


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()

    carla_map = world.get_map()
    topology = carla_map.get_topology()

    G = nx.DiGraph()

    for waypoint_pair in topology:
        w0 = waypoint_pair[0]
        w1 = waypoint_pair[1]
        w0_2d_loc = np.array([w0.transform.location.x, w0.transform.location.y])
        w1_2d_loc = np.array([w1.transform.location.x, w1.transform.location.y])
        dist = np.linalg.norm(w0_2d_loc - w1_2d_loc)
        G.add_node(w0.id, position=w0_2d_loc)
        G.add_node(w1.id, position=w1_2d_loc)
        G.add_edge(w0.id, w1.id, weight=dist)

    node_pos = nx.get_node_attributes(G, 'position')

    _, ax = plt.subplots(figsize=(12, 8))
    plt.title('CARLA Map')
    nx.draw(G, node_pos, node_size=10)
    ax.set_axis_on()
    ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    plt.grid('on')
    plt.show()

if __name__ == '__main__':
    main()
