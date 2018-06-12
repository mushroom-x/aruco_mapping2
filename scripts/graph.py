#!/usr/bin/env python
# -*- coding: utf-8 -*-
class Node:
    '''
    定义图的节点 即ArucoTag码
    '''
    def __init__(self, node_id):
        self.id = node_id # 节点编号
        self.parent_id = -1 # 父亲节点
        self.edges = {} # 边的集合
        self.isvisited = False #是否被访问
    
    def __str__(self):
        node_str = "===== Node: {} =====\n".format(node_id)
        for target_node_id in self.edges.keys():
            node_str += ("To: {} Weight: {}\n".format(target_node_id, self.edges[target_node_id]))
        return node_str

    def set_parent_id(self, parent_id):
        self.parent_id = parent_id # 父节点的ID
    
    def get_neighbors(self):
        return list(self.edges.keys())

    def add_edge(self, target_node_id, weight):
        self.edges[target_node_id] = weight

class UndirectGraph:
    '''
    无向图
    '''
    def __init__(self, base_node_id):
        self.node_num = 0 # 节点的个数
        self.base_node_id = base_node_id # 世界坐标系对应Aruco码的ID
        self.nodes_dict = {} # 节点字典
        self.add_node(base_node_id) # 添加节点

    def __iter__(self):
        return iter(self.nodes_dict.values())

    def add_node(self, node_id):
        '''
        添加单个节点
        '''
        if node_id in self.nodes_dict.keys():
            # 该节点已存在
            print("当前节点 {} 已存在".format(node_id))
        else:
            self.nodes_dict[node_id] = Node(node_id)
            self.node_num += 1

    def add_nodes(self, node_list):
        '''
        添加多个节点
        '''
        for node_id in node_list:
            self.add_node(node_id)

    def get_node_id_list(self):
        '''
        获取所有ID的列表
        '''
        return list(self.nodes_dict.keys())

    def get_node(self, node_id):
        '''
        获取节点
        '''
        if node_id not in self.nodes_dict:
            return None
        return self.nodes_dict[node_id]


    def add_edge(self, frm_node_id, to_node_id, weight):
        '''
        添加节点
        因为是无向图,所以需要双向添加
        '''
        if frm_node_id not in self.nodes_dict.keys():
            self.add_node(frm_node_id)
        if to_node_id not in self.nodes_dict.keys():
            self.add_node(to_node_id)
        
        self.nodes_dict[frm_node_id].add_edge(to_node_id, weight)
        self.nodes_dict[to_node_id].add_edge(frm_node_id, weight)

    def init_visit_record(self):
        for node_id, node in self.nodes_dict.items():
            node.isvisited = False

    def BFS(self):
        '''
        广度优先遍历
        '''
        # 初始化访问记录
        self.init_visit_record()
        
        order = []

        bfs_queue = []
        bfs_queue.append(self.base_node_id)
        
        while len(bfs_queue) != 0:
            cur_node_id = bfs_queue.pop(0) # 弹出队首元素
            if self.nodes_dict[cur_node_id].isvisited:
                continue
            # 标记访问过该节点
            self.nodes_dict[cur_node_id].isvisited = True
            # 顺序数组添加元素
            order.append(cur_node_id)

            print("访问节点:  {}".format(cur_node_id))
            # 添加邻居
            neighbors = self.nodes_dict[cur_node_id].edges
            for node_id in neighbors:
                if not self.nodes_dict[node_id].isvisited:
                    bfs_queue.append(node_id)
                    if self.nodes_dict[node_id].parent_id == -1:
                        # 第一次遍历到这个节点，记录下父节点的位置
                        self.nodes_dict[node_id].parent_id = cur_node_id
        return order
            
        
