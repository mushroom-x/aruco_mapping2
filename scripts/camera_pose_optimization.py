#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
节点功能说明：
    根据ArucoTag更新Camera的姿态 
    并对当前的姿态做非线性优化
    并绘制实时的路径

'''
import rospy
import rospkg
from aruco_msgs.msg import  MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped,PoseStamped,Point
from nav_msgs.msg import Path
import tf
import tf_conversions.posemath as pm
import pickle
from numpy import linalg as LA
import tf2_ros
import math
import random
import time

rospack = rospkg.RosPack()
package_path = rospack.get_path('aruco_mapping2')

# tf_listener = None
tf_broadcast = tf.TransformBroadcaster()
aruco_subscriber = None
aruco_map = {}
camera_frame = None
camera_pose_record = [] # 保留相机的历史轨迹信息
rviz_pub_path = None # 发布轨迹信息
rviz_pub_marker = None
color_dict = {}

def load_aruco_map():
    '''
    载入aruco地图
    从二进制序列化文件中载入数据
    '''
    global aruco_map
    global color_dict
    global package_path
    
    with open(package_path + '/data/aruco_map.bin', 'rb') as f:
        aruco_map = pickle.load(f)

    with open(package_path + '/data/color_dict.bin', 'rb') as f:
        color_dict = pickle.load(f)

def update_camera_pose(camera_pose_msg):
    '''
    更新相机的TF变换
    '''
    global camera_frame
    
    camera_trans = TransformStamped()
    camera_trans.header.stamp = rospy.Time.now()
    camera_trans.header.frame_id = 'world'
    camera_trans.child_frame_id = 'camera'
    camera_trans.transform.translation = camera_pose_msg.position
    camera_trans.transform.rotation = camera_pose_msg.orientation

    tf_broadcast.sendTransform(camera_trans)


def get_camera_pose(aruco_msg):
    '''
    根据aruco的Message，与aruco地图（aruco_map）
    推算出aruco码相对于世界坐标系的位置跟姿态
    '''
    global aruco_map

    # 从姿态信息 构建 PyKDL Frame对象
    aruco2camera = pm.fromMsg(aruco_msg.pose.pose)
    # 调整姿态 绕X轴逆向旋转90度 弧度
    aruco2camera.M.DoRotX(-math.pi/2)
    aruco2camera.M.DoRotZ(math.pi) # 校正姿态 Z轴旋转180度
    # 矩阵取反，改为相机相对于aruco码的相对位置
    camera2aruco = aruco2camera.Inverse()
    # 获取aruco相对于世界坐标系的矩阵

    aruco2world_mat = pm.toMatrix(pm.fromMsg(aruco_map[aruco_msg.id]))
    # 矩阵相乘，获取相机相对于世界坐标系的变换矩阵
    camera2world_mat = aruco2world_mat.dot(pm.toMatrix(camera2aruco))
    # 将矩阵转换为姿态Message
    camera_pose_msg = pm.toMsg(pm.fromMatrix(camera2world_mat))
    return camera_pose_msg


def update_camera_path(camera_pose_msg):
    '''
    更新相机运行轨迹
    '''
    global camera_pose_record
    global rviz_pub_path
    ps = PoseStamped()
    ps.header.frame_id = "/world"
    ps.header.stamp = rospy.Time.now()

    ps.pose = camera_pose_msg
    camera_pose_record.append(ps)
    path = Path()
    path.header.frame_id = "/world"
    path.header.stamp = rospy.Time.now()
    path.poses = camera_pose_record
    rviz_pub_path.publish(path)


def draw_position_points(arucos):
    '''
    绘制根据不同码，推导出来的相机坐标的位置，绘制在rviz上。
    '''
    global points_draw_cnt
    global rviz_pub_marker
    global color_dict

    for aruco_msg in arucos:

        points = Marker()
        points.header.frame_id = "world"
        points.header.stamp = rospy.Time.now()
        points.ns = "camera_posi"
        points.id = aruco_msg.id
        points.type = Marker.POINTS
        # 点的尺寸可以自己设置
        points.scale.x = 0.10
        points.scale.y = 0.10
        points.scale.z = 0.10

        (r, g, b) = color_dict[aruco_msg.id]
        # 随意生成一个颜色
        points.color.r = r
        points.color.g = g
        points.color.b = b
        points.color.a = 0.9
        points.lifetime = rospy.Duration(0.1)

        camera_pose_msg = get_camera_pose(aruco_msg)
        
        pt = Point()
        pt.x = camera_pose_msg.position.x
        pt.y = camera_pose_msg.position.y
        pt.z = camera_pose_msg.position.z

        points.points.append(pt)

        rviz_pub_marker.publish(points)



# x y z 历史信息队列
last_x_queue = [0,0,0,0,0,0,0,0]
last_y_queue = [0,0,0,0,0,0,0,0]
last_z_queue = [0,0,0,0,0,0,0,0]
# 一维卷积核
conv_kernel = [0.07, 0.07, 0.07, 0.10, 0.10, 0.15, 0.15, 0.29]

def camera_pose_filter(arucos):
    '''
    相机位姿滤波， 先进行均值滤波， 后进行一维线性卷积
    '''
    global last_x_mean
    global last_y_mean
    global last_z_mean
    global conv_kernel

    # 均值滤波
    x_mean = 0
    y_mean = 0
    z_mean = 0

    aruco_num = len(arucos)

    for aruco_msg in arucos:
        camera_pose_msg = get_camera_pose(aruco_msg)
        x_mean += camera_pose_msg.position.x / aruco_num
        y_mean += camera_pose_msg.position.y / aruco_num
        z_mean += camera_pose_msg.position.z / aruco_num

    # 队列 弹出队首元素，插入新的取值
    last_x_queue.pop(0)
    last_x_queue.append(x_mean)
    last_y_queue.pop(0)
    last_y_queue.append(y_mean)
    last_z_queue.pop(0)
    last_z_queue.append(z_mean)

    # 一维线性卷积
    cur_x = 0
    cur_y = 0
    cur_z = 0
    for i in range(len(conv_kernel)):
        cur_x += conv_kernel[i] * last_x_queue[i]
        cur_y += conv_kernel[i] * last_y_queue[i]
        cur_z += conv_kernel[i] * last_z_queue[i]

    
    return (cur_x, cur_y, cur_z)

def callback(marker_array):
    global camera_frame
    global tf_broadcast
    
    '''
    # 写入时间戳 用于记录帧率 分析实时性
    with open('time_log.txt', 'a') as f:
        f.write("{}\n".format(time.time()))
    '''
    # 绘制camera位置点 多个aruco码的映射
    draw_position_points(marker_array.markers)

    ref_aruco_msg = marker_array.markers[0]
    camera_pose_msg = get_camera_pose(ref_aruco_msg)
    # 对相机的位置进行滤波（暂不包括位姿，可自行添加）
    (x, y, z) = camera_pose_filter(marker_array.markers)
    camera_pose_msg.position.x = x
    camera_pose_msg.position.y = y
    camera_pose_msg.position.z = z

    # 更新相机的TF变换
    update_camera_pose(camera_pose_msg)
    # 更新camera的运行轨迹
    update_camera_path(camera_pose_msg)

if __name__=="__main__":
    rospy.init_node('camera_pose_opti', anonymous=True) # 初始化节点
    load_aruco_map() # 载入地图
    tf_broadcast = tf2_ros.TransformBroadcaster() # 初始化广播器
    aruco_subscriber = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, callback)
    rviz_pub_path = rospy.Publisher('camera_path', Path, queue_size=100)
    rviz_pub_marker = rospy.Publisher('camera_marker', Marker, queue_size=400)

    rospy.spin()