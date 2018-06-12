#!/usr/bin/env python
# -*- coding: utf-8 -*-
from graph import UndirectGraph

if __name__ == '__main__':

    graph = UndirectGraph(base_node_id='d')
    graph.add_nodes(['a', 'b', 'c', 'd', 'e', 'f'])

    graph.add_edge('a', 'b', 7)  
    graph.add_edge('a', 'c', 9)
    graph.add_edge('a', 'f', 14)
    graph.add_edge('b', 'c', 10)
    graph.add_edge('b', 'd', 15)
    graph.add_edge('c', 'd', 11)
    graph.add_edge('c', 'f', 2)
    graph.add_edge('d', 'e', 6)
    graph.add_edge('e', 'f', 9)

    for node_id in graph.BFS():
        print("Node ID: {} Parent ID: {}".format(node_id, graph.nodes_dict[node_id].parent_id))