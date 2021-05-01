#!/usr/bin/env python

import matplotlib
from matplotlib import pyplot as plt
import matplotlib.animation as animation

import networkx as nx
import numpy as np
import rospy
import planner
import mission_control


rospy.init_node('mctrl_map_viewer')
matplotlib.use('TkAgg')

G = planner.get_map_graph()

# plot graph
fig, ax = plt.subplots(figsize=(12, 8))
node_pos = nx.get_node_attributes(G, 'position')

def animate(i):
    ax.clear()

    # map graph
    node_colors = list(nx.get_node_attributes(G, 'color').values())
    node_sizes = list(nx.get_node_attributes(G, 'size').values())
    edge_colors = list(nx.get_edge_attributes(G, 'color').values())
    if (len(edge_colors) == 0):
        edge_colors = 'k'
    nx.draw(G, node_pos, node_size=node_sizes, node_color=node_colors, edge_color=edge_colors)

    # ego vehicle
    ego_vehicle_location = planner.get_ego_vehicle_location()
    if ego_vehicle_location is not None:
        # location
        ax.plot(ego_vehicle_location.x, ego_vehicle_location.y, color='green', marker='o', markersize=10)

        # heading
        arrow_length = 30
        ax.arrow(ego_vehicle_location.x, ego_vehicle_location.y, 0, arrow_length, length_includes_head=True, head_width=5, ls=(5, (3, 6)), color='green') # map north
        ax.text(ego_vehicle_location.x - 3, ego_vehicle_location.y + arrow_length + 2, 'N', color='green')
        heading = planner.get_ego_vehicle_heading()
        if heading:
            [heading_in_degrees, dx, dy] = planner.get_ego_vehicle_map_heading(heading, arrow_length)
            ax.arrow(ego_vehicle_location.x, ego_vehicle_location.y, dx, dy, length_includes_head=True, head_width=5, color='green')
            ax.text(ego_vehicle_location.x + dx, ego_vehicle_location.y + dy, str(round(heading_in_degrees, 2)) + '°', color='green')

    ax.set_axis_on()
    ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    plt.title('Map Viewer')
    plt.grid('on')

anim = animation.FuncAnimation(fig, animate, repeat=False)

# set destination point and calculate route
def calc_route(event):
    global G

    ego_vehicle_location = planner.get_ego_vehicle_location()
    if ego_vehicle_location:
        vehicle_pos = np.array([ego_vehicle_location.x, ego_vehicle_location.y])
        destination_pos = np.array([event.xdata, event.ydata])

        _, G = planner.plan_route(G, vehicle_pos, destination_pos)

cid = fig.canvas.mpl_connect('button_press_event', calc_route)

def destroy():
    anim.event_source.stop()
    plt.close('all')
    mission_control.clear_vehicle_storage()

rospy.on_shutdown(destroy)
plt.show()
