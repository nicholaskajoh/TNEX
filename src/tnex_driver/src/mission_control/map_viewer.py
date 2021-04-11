#!/usr/bin/env python

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

import matplotlib
from matplotlib import pyplot as plt
import matplotlib.animation as animation

import time
import networkx as nx
from networkx.exception import NetworkXNoPath
import numpy as np
import redis
import json
from tnex_driver.msg import GNSSMeasurement
import rospy
import itertools

rospy.init_node('mctrl_map_viewer')
matplotlib.use('TkAgg')

try:
    r = redis.Redis()
except redis.ConnectionError as e:
    rospy.logerr(e)

# get ego vehicle location from GNSS sensor
ego_vehicle_location = None
def get_gnss(message):
    global ego_vehicle_location
    ego_vehicle_location = message.location

rospy.Subscriber('gnss', GNSSMeasurement, get_gnss)

# get CARLA map waypoints
client = carla.Client('localhost', 2000)
client.set_timeout(15.0)

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
    edge_colors = list(nx.get_edge_attributes(G, 'color').values())
    if (len(edge_colors) == 0):
        edge_colors = 'k'
    nx.draw(G, node_pos, node_size=node_sizes, node_color=node_colors, edge_color=edge_colors)
    ax.set_axis_on()
    ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    plt.title('Map Viewer')
    plt.grid('on')

    if ego_vehicle_location is not None:
        ax.plot(ego_vehicle_location.x, ego_vehicle_location.y, color='green', marker='o', markersize=10)

anim = animation.FuncAnimation(fig, animate, interval=100)

# set destination point and calculate route
def plan_route(destination_pos):
    rospy.loginfo('Calculating route to destination...')

    vehicle_pos = np.array([ego_vehicle_location.x, ego_vehicle_location.y])

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

    rospy.loginfo('Closest nodes to vehicle and destination found!')

    def heuristic(node1, node2):
        return np.linalg.norm(G.nodes[node1]['position'] - G.nodes[node2]['position'])

    # shortest path with A star
    try:
        route = nx.astar_path(G, id_of_node_closest_to_vehicle, id_of_node_closest_to_destination, heuristic=heuristic)
    except NetworkXNoPath as e:
        rospy.logerr('No route available: ' + str(e))
    else:
        rospy.loginfo('Route generated!')

        # color nodes in route
        nx.set_node_attributes(G, 'black', name='color')
        nx.set_node_attributes(G, 10, name='size')
        node_attrs = {node_id: {'color': 'blue', 'size': 30} for node_id in route}
        nx.set_node_attributes(G, node_attrs)
        # color edges in route
        nx.set_edge_attributes(G, 'black', name='color')
        edge_attrs = {(nodeA, nodeB): {'color': 'blue'} for (nodeA, nodeB) in zip(route, route[1:])}
        nx.set_edge_attributes(G, edge_attrs)
        rospy.loginfo('Map updated... done')

        # store route in Redis
        r.set('route', json.dumps(route))
        r.set('next_waypoint_index', 0)

cid = fig.canvas.mpl_connect('button_press_event', lambda event: plan_route(np.array([event.xdata, event.ydata])))

# reroute vehicle if it goes off course
# ...

def destroy():
    anim.event_source.stop()
    plt.close('all')

rospy.on_shutdown(destroy)
plt.show()