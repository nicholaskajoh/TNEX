#!/usr/bin/env python

import matplotlib
from matplotlib import pyplot as plt
import matplotlib.animation as animation

import networkx as nx
import numpy as np
import rospy
import planner
from mission_control import vehicle_state
from sensors import map, gnss, imu


rospy.init_node('mctrl_map_viewer')
matplotlib.use('TkAgg')

G = map.get_map_graph()

# plot graph
fig, ax = plt.subplots(figsize=(12, 8))
node_pos = nx.get_node_attributes(G, 'position')
node_colors = list(nx.get_node_attributes(G, 'color').values())
node_sizes = list(nx.get_node_attributes(G, 'size').values())
edge_colors = list(nx.get_edge_attributes(G, 'color').values())
if (len(edge_colors) == 0):
    edge_colors = 'k'

def get_plots():
    plots = []

    # map
    nodes_plot = nx.draw_networkx_nodes(G, node_pos, node_size=node_sizes, node_color=node_colors)
    edges_plot = nx.draw_networkx_edges(G, node_pos, node_size=node_sizes, edge_color=edge_colors)
    plots.extend([nodes_plot, *edges_plot])

    # ego vehicle
    ego_vehicle_location = gnss.get_ego_vehicle_location()
    if ego_vehicle_location is not None:
        # location
        ev_pos_plot = ax.plot(ego_vehicle_location.x, ego_vehicle_location.y, color='green', marker='o', markersize=10)

        # heading
        arrow_length = 30
        map_north_plot = ax.arrow(ego_vehicle_location.x, ego_vehicle_location.y, 0, arrow_length, length_includes_head=True, head_width=5, ls=(5, (3, 6)), color='green') # map north
        map_north_text_plot = ax.text(ego_vehicle_location.x - 3, ego_vehicle_location.y + arrow_length + 2, 'N', color='green')
        plots.extend([*ev_pos_plot, map_north_plot, map_north_text_plot])

        heading = imu.get_ego_vehicle_heading()
        if heading:
            [heading_in_degrees, dx, dy] = imu.get_ego_vehicle_map_heading(heading, arrow_length)
            ev_heading_plot = ax.arrow(ego_vehicle_location.x, ego_vehicle_location.y, dx, dy, length_includes_head=True, head_width=5, color='green')
            ev_heading_text_plot = ax.text(ego_vehicle_location.x + dx, ego_vehicle_location.y + dy, str(round(heading_in_degrees, 2)) + '°', color='green')
            plots.extend([ev_heading_plot, ev_heading_text_plot])

    return plots

def init():
   return get_plots()

def animate(i):
    return get_plots()

anim = animation.FuncAnimation(fig, animate, repeat=False, init_func=init, blit=True, interval=100)

# set destination point and calculate route
def calc_route(event):
    global G
    global node_colors
    global node_sizes
    global edge_colors

    ego_vehicle_location = gnss.get_ego_vehicle_location()
    if ego_vehicle_location:
        vehicle_pos = np.array([ego_vehicle_location.x, ego_vehicle_location.y])
        destination_pos = np.array([event.xdata, event.ydata])

        _, G = planner.plan_route(G, vehicle_pos, destination_pos)

        node_colors = list(nx.get_node_attributes(G, 'color').values())
        node_sizes = list(nx.get_node_attributes(G, 'size').values())
        edge_colors = list(nx.get_edge_attributes(G, 'color').values())
        if (len(edge_colors) == 0):
            edge_colors = 'k'

cid = fig.canvas.mpl_connect('button_press_event', calc_route)

def destroy():
    anim.event_source.stop()
    plt.close('all')
    vehicle_state.clear()

rospy.on_shutdown(destroy)

ax.set_axis_on()
ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
plt.title('Map Viewer')
plt.grid('on')
plt.show()
