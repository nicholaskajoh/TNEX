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


# subscribe to transform channel to get vehicle position
p = r.pubsub()

transform = None
def carla_transform_handler(message):
    global transform
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
    G.add_node(w0.id, position=w0_2d_loc, color='black', size=10)
    G.add_node(w1.id, position=w1_2d_loc, color='black', size=10)
    G.add_edge(w0.id, w1.id, weight=dist)

# plot graph
fig, ax = plt.subplots(figsize=(12, 8))

node_pos = nx.get_node_attributes(G, 'position')

def animate(i):
    ax.clear()

    node_colors = list(nx.get_node_attributes(G, 'color').values())
    node_sizes = list(nx.get_node_attributes(G, 'size').values())
    nx.draw(G, node_pos, node_size=node_sizes, node_color=node_colors)
    ax.set_axis_on()
    ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    plt.title('CARLA Map')
    plt.grid('on')

    if transform is not None:
        ax.plot(transform['location']['x'], transform['location']['y'], color='green', marker='o', markersize=10)

ani = animation.FuncAnimation(fig, animate, interval=500)

# set destination point and calculate route
def onclick_handler(event):
    print('Calculating route to destination...')

    vehicle_pos = np.array([transform['location']['x'], transform['location']['y']])
    destination_pos = np.array([event.xdata, event.ydata])

    nodes_list = list(G)

    id_of_node_closest_to_vehicle = nodes_list[0]
    dist_of_node_closest_to_vehicle = np.linalg.norm(G.nodes[nodes_list[0]]['position'] - vehicle_pos)
    id_of_node_closest_to_destination = nodes_list[0]
    dist_of_node_closest_to_destination = np.linalg.norm(G.nodes[nodes_list[0]]['position'] - destination_pos)

    for node_id in nodes_list:
        dist_of_node_from_vehicle = np.linalg.norm(G.nodes[node_id]['position'] - vehicle_pos)
        if dist_of_node_from_vehicle < dist_of_node_closest_to_vehicle:
            id_of_node_closest_to_vehicle = node_id
            dist_of_node_closest_to_vehicle = dist_of_node_from_vehicle

        dist_of_node_from_destination = np.linalg.norm(G.nodes[node_id]['position'] - destination_pos)
        if dist_of_node_from_destination < dist_of_node_closest_to_destination:
            id_of_node_closest_to_destination = node_id
            dist_of_node_closest_to_destination = dist_of_node_from_destination

    print('Closest nodes to vehicle and destination found!')

    def heuristic(node1, node2):
        return np.linalg.norm(G.nodes[node1]['position'] - G.nodes[node2]['position'])

    # shortest path with A star
    route = nx.astar_path(G, id_of_node_closest_to_vehicle, id_of_node_closest_to_destination, heuristic=heuristic)
    print('Route generated!')

    # color nodes in route
    nx.set_node_attributes(G, 'black', name='color')
    nx.set_node_attributes(G, 10, name='size')
    attrs = {node_id: {'color': 'blue', 'size': 30} for node_id in route}
    nx.set_node_attributes(G, attrs)

    print('Map updated... done')

cid = fig.canvas.mpl_connect('button_press_event', onclick_handler)

plt.show()