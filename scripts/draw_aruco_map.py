#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Pacakge功能描述
    在RVIZ空间中绘制地图
    aruco tag 绘制坐标系（直线）
    备注编号
    发布静态TF
'''
import rospy
import rospkg

from visualization_msgs.msg import Marker,InteractiveMarker
from geometry_msgs.msg import Point,TransformStamped
import copy
from tf2_ros import StaticTransformBroadcaster
import random


rospack = rospkg.RosPack()
package_path = rospack.get_path('aruco_mapping2')

rivz_pub = None
tf_staic_broadcast = None
arucos_dict = {}

def draw_marker_cube(aruco_id, aruco_pose, color):
    '''
    绘制ArucoTag的立体Cube
    '''
    global rivz_pub
    marker = Marker()
    marker.header.frame_id = "world" # 静止坐标系
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "aruco_map"
    marker.id = aruco_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = aruco_pose

    marker.scale.x = 0.165
    marker.scale.y = 0.165
    marker.scale.z = 0.05

    (r, g, b) = color
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 0.8

    # 生命周期，是不是永久有效
    marker.lifetime = rospy.Duration()

    rivz_pub.publish(marker)

def send_aruco_static_transform(aruco_id, aruco_pose):
    '''
    发布静态Aruco的TF变换
    '''
    global tf_staic_broadcast

    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = 'world'
    transform_stamped.child_frame_id = 'aruco_%d'%aruco_id
    transform_stamped.transform.translation = aruco_pose.position
    transform_stamped.transform.rotation = aruco_pose.orientation

    tf_staic_broadcast.sendTransform(transform_stamped)



def add_sigle_marker(aruco_id, aruco_pose, color):
    '''
    添加单个arucotag
    '''
    global arucos_dict
    draw_marker_cube(aruco_id, aruco_pose, color)
    send_aruco_static_transform(aruco_id, aruco_pose)
    
def load_aruco_map():
    '''
    载入aruco地图
    '''
    import pickle
    global package_path

    f = open(package_path + '/data/aruco_map.bin', 'rb')
    aruco_map = pickle.load(f)

    color_dict = {}
    
    for aruco_id, aruco_pose  in aruco_map.items():
        rospy.loginfo("载入Aruco ID: {}".format(aruco_id))
        # 为这个ArucoTag生成一个颜色
        r = random.uniform(0.5,1)
        g = random.uniform(0.5,1)
        b = random.uniform(0.5,1)
        color = (r, g, b)
        color_dict[aruco_id] = color

        add_sigle_marker(aruco_id, aruco_pose, color)

    with open(package_path + '/data/color_dict.bin', 'wb') as f:
        pickle.dump(color_dict, f)

def draw_aruco_map():
    '''
    arucotag 绘制aruco地图
    '''
    global rivz_pub
    global tf_staic_broadcast
    rospy.init_node('draw_aruco_map', anonymous=True)
    tf_staic_broadcast = StaticTransformBroadcaster()
    rivz_pub = rospy.Publisher('aruco_map_rviz', Marker, queue_size=10)

    
    while(rivz_pub.get_num_connections() < 1):
        if rospy.is_shutdown():
            rospy.signal_shutdown('Master is not on')

        # load_aruco_map()
        rospy.logwarn("请创建RVIZ的Subscriber")
        rospy.sleep(5)


    load_aruco_map()
    rospy.signal_shutdown('Quit')

if __name__=="__main__":
    draw_aruco_map()