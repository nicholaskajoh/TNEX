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
import matplotlib.animation as animation

import time
import networkx as nx
import numpy as np
import redis
import pickle


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    print(e)


def main():
    # subscribe to transform channel to get vehicle position
    p = r.pubsub()

    transform = None
    def carla_transform_handler(message):
        nonlocal transform
        if message and type(message['data']) is not int:
            transform = pickle.loads(message['data'])

    p.subscribe(**{'carla_transform': carla_transform_handler})
    p.run_in_thread(sleep_time=0.001)

    # get CARLA map waypoints
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

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
        G.add_node(w0.id, position=w0_2d_loc)
        G.add_node(w1.id, position=w1_2d_loc)
        G.add_edge(w0.id, w1.id, weight=dist)

    # plot graph
    node_pos = nx.get_node_attributes(G, 'position')

    fig, ax = plt.subplots(figsize=(12, 8))

    def animate(i):
        ax.clear()

        nx.draw(G, node_pos, node_size=10)
        ax.set_axis_on()
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.title('CARLA Map')
        plt.grid('on')

        if transform is not None:
            ax.plot(transform['location']['x'], transform['location']['y'], color='red', marker='o', markersize=10)

    ani = animation.FuncAnimation(fig, animate, interval=500)
    plt.show()

if __name__ == '__main__':
    main()
