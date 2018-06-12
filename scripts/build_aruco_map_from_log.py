#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
节点功能说明
    从日志中建图
    先不考虑大空间的问题， 就先使用当前单个板上的数据
    首先直接绘制空间中的aruco marker

'''
import rospy
import rospkg

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,Point,Quaternion,TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import String

import pickle

import tf_conversions.posemath as pm
import PyKDL

import math
from aruco_msgs.msg import  MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

from graph import UndirectGraph


rospack = rospkg.RosPack()
package_path = rospack.get_path('aruco_mapping2') # 获取package路径

arucos_dict = None
rviz_marker_pub = None


tf_staic_broadcast = None
aruco_map  = {}

def load_arucos_dict():
    '''
    载入aruco字典
    '''
    global arucos_dict
    global package_path

    with open(package_path + '/data/arucos_dict.bin', 'rb') as f:
        arucos_dict = pickle.load(f)


rviz_marker_count = 0
def draw_single_marker(aruco_id, aruco_pose, alpha=0.01, height=0.005):
    '''
    绘制ArucoTag的立体Cube 超薄 透明
    '''
    global rviz_marker_pub
    global rviz_marker_count

    marker = Marker()
    marker.header.frame_id = "world" # 静止坐标系
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "aruco_raw_map"
    marker.id = rviz_marker_count
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = aruco_pose

    marker.scale.x = 0.165
    marker.scale.y = 0.165
    marker.scale.z = height

    # 颜色为白色
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1

    marker.color.a = alpha

    # 生命周期，是不是永久有效
    marker.lifetime = rospy.Duration()

    rviz_marker_pub.publish(marker)
    rviz_marker_count += 1


def get_base_tag_pose():
    pose = Pose()
    pose.position = Point(0,0,0)
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    return pose

def weighted_mean_filter(pose_list):
    '''
    加权均值滤波
    距离远 权重低
    '''
    weight_sum = 0
    pose_sum = np.float32([0,0,0,0,0,0])

    for distance, aruco_msg in pose_list:
        weight = 1.0 / distance
        # print("Weight: {}".format(weight))
        if weight > 5:
            # 设定最大的权值，防止因为单次近距离测量导致影响过大
            weight = 5

        weight_sum += weight 
        
        cur_pose = [0,0,0,0,0,0]
        cur_pose[0] = aruco_msg.position.x
        cur_pose[1] = aruco_msg.position.y
        cur_pose[2] = aruco_msg.position.z
        
        # 旋转使用欧拉角来表示
        euler = euler_from_quaternion((aruco_msg.orientation.x, aruco_msg.orientation.y, 
                        aruco_msg.orientation.z, aruco_msg.orientation.w))
        
        cur_pose[3] = euler[0]
        cur_pose[4] = euler[1]
        cur_pose[5] = euler[2]
        # 累加
        pose_sum += weight * np.float32(cur_pose)
       
    # 求均值
    pose_mean = pose_sum / weight_sum

    # 获取坐标均值
    x_mean = pose_mean[0]
    y_mean = pose_mean[1]
    z_mean = pose_mean[2]
    # 获取欧拉角均值
    roll_mean = pose_mean[3]
    pitch_mean = pose_mean[4]
    yaw_mean = pose_mean[5]
    
    # 创建一个Pose Message 对象
    aruco_mean_pose = Pose()
    aruco_mean_pose.position = Point(x_mean,y_mean,z_mean)
    # 欧拉角转换为四元数
    (x,y,z,w) = quaternion_from_euler(roll_mean, pitch_mean, yaw_mean)
    aruco_mean_pose.orientation.x =  x
    aruco_mean_pose.orientation.y = y
    aruco_mean_pose.orientation.z = z
    aruco_mean_pose.orientation.w = w

    return aruco_mean_pose

def cal_distance_between_arucos(pose_list):
    '''
    从pose message中计算码与码之间的相对距离
    '''
    pose = weighted_mean_filter(pose_list)
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    distance = math.sqrt(math.pow(x,2) + math.pow(y, 2) + math.pow(z, 2))

    return distance
def aruco_udg(max_dis=0.5):
    '''
    从当前的arucos_dict信息推导出aruco码的无向带权图， 权为距离
    max_dis是最长的距离
    '''
    global arucos_dict
    # 创建一个图对象， 并指名参考码
    graph = UndirectGraph(base_node_id=BASE_ARUCO_ID)
    # 添加节点
    graph.add_nodes(list( arucos_dict.keys()))
    
    # 添加Edge
    for frm_aruco_id in arucos_dict.keys():
        for to_aruco_id, pose_list in arucos_dict[frm_aruco_id].items():
            distance = cal_distance_between_arucos(pose_list)
            if distance < max_dis:
                graph.add_edge(frm_aruco_id, to_aruco_id, distance)
    return graph

def draw_raw_map():
    global package_path
    global arucos_dict
    global BASE_ARUCO_ID
    global rviz_marker_pub
    global aruco_map

    while(rviz_marker_pub.get_num_connections() < 1):
        if rospy.is_shutdown():
            rospy.signal_shutdown('Master主节点没有开启')

        # load_aruco_map()
        rospy.logwarn("请创建RVIZ的Subscriber")
        rospy.sleep(5)
    
    aruco_map[BASE_ARUCO_ID] = get_base_tag_pose()
    # 绘制参考marker
    draw_single_marker(BASE_ARUCO_ID, get_base_tag_pose(), alpha=0.8, height=0.05)
    
    # 构建一个图
    graph = aruco_udg(max_dis=rospy.get_param('max_distance'))
    order = graph.BFS()

    for aruco_id in order[1:]:
        # 跳过第一个， 因为第一个是base aruco， 已经确定
        # 按照拓扑顺序依次遍历
        parent_id = graph.nodes_dict[aruco_id].parent_id
        # 获取所有记录到的从parent变换到当前码的变换
        pose_list = arucos_dict[parent_id][aruco_id]
        # 均值滤波获取Pose
        pose = weighted_mean_filter(pose_list)
        # 父亲码在世界坐标系下的位姿
        parent_pose = aruco_map[parent_id]

        world2parent_mat = pm.toMatrix(pm.fromMsg(parent_pose))
        parent2child_mat = pm.toMatrix(pm.fromMsg(pose))
        world2child_mat = world2parent_mat.dot(parent2child_mat)
        
        aruco_pose = pm.toMsg(pm.fromMatrix(world2child_mat))

        # 地图追加该aruco码
        aruco_map[aruco_id] = aruco_pose
        # 绘制当前的aruco码
        draw_single_marker(aruco_id, aruco_pose, alpha=0.8, height=0.05)
        

    # 序列化保存    
    with open(package_path + '/data/aruco_map.bin', 'wb') as f:
        pickle.dump(aruco_map, f)
    # 发布静态TF变换
    for aruco_id, pose in aruco_map.items():
        send_aruco_static_transform(aruco_id, pose)
    
    # 结束当前的进程
    rospy.signal_shutdown('绘制了所有的aruco，并发布了aruco码的TF变换')



def send_aruco_static_transform(aruco_id, aruco_pose):
    '''
    发布静态Aruco的TF变换
    '''
    global tf_staic_broadcast # 静态广播

    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = 'world'
    transform_stamped.child_frame_id = 'aruco_%d'%aruco_id
    transform_stamped.transform.translation = aruco_pose.position
    transform_stamped.transform.rotation = aruco_pose.orientation

    tf_staic_broadcast.sendTransform(transform_stamped)


def finish_sample_collect_callback(info):
    rospy.loginfo(info) # 打印
    load_arucos_dict() # 载入数据
    draw_raw_map() # 绘制原始地图 并退出节点
    
if __name__=="__main__":
    rospy.init_node('build_aruco_map', anonymous=True) # 初始化节点
    BASE_ARUCO_ID = rospy.get_param('/build_aruco_map_from_log/base_aruco_id') # 获取基准参考码的ID
    tf_staic_broadcast = StaticTransformBroadcaster() # 静态TF变换发布
    rviz_marker_pub = rospy.Publisher('draw_aruco_map', Marker, queue_size=4000)    
    # 监听样本采集完成
    sample_done_sub = rospy.Subscriber('/aruco_sample_done', String, finish_sample_collect_callback)
    rospy.spin()