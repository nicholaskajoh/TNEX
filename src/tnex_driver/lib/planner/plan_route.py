import rospy
import numpy as np
import networkx as nx
from networkx.exception import NetworkXNoPath
import redis
import json


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    rospy.logerr(e)

def plan_route(G, vehicle_pos, destination_pos):
    rospy.loginfo('Calculating route to destination...')

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

    try:
        # shortest path with A star
        route = nx.astar_path(G, id_of_node_closest_to_vehicle, id_of_node_closest_to_destination, heuristic=heuristic)
    except NetworkXNoPath as e:
        route = None
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

        # store route data
        r.set('tnex_driver:route', json.dumps(route))
        r.set('tnex_driver:next_waypoint_index', 0)
        r.set('tnex_driver:next_waypoint_distance', dist_of_node_closest_to_vehicle)
        r.set('tnex_driver:destination_position', json.dumps(destination_pos.tolist()))

    return route, G
