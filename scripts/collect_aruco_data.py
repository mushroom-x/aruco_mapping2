#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
节点功能说明
    收集ArucoTag之间的相对位姿的数据
'''
import rospy
import rospkg
from aruco_msgs.msg import  MarkerArray
import tf_conversions.posemath as pm
import math
from itertools import product  
import pickle
from std_msgs.msg import String

rospack = rospkg.RosPack()
package_path = rospack.get_path('aruco_mapping2')

MIN_SAMPLE_NUM =  int(rospy.get_param('/collect_aruco_data/frame_sample_num')) # 最小取样次数
ARUCO_NUM = int(rospy.get_param('/collect_aruco_data/aruco_num')) # aruco码的总个数

arucos_dict = {}
sample_count = 1

def transfrom_between_arucos(camera2ai, camera2aj):
    '''
    计算arucos之间的转换
    换算成以 aruco i为参考坐标系 对应arucoj的姿态
    '''
    # 获取以相机为世界坐标系下的码j的位姿
    camera2aj_mat = pm.toMatrix(camera2aj)
    # 获取以码i做世界坐标系下的相机的位姿
    ai2camera_mat = pm.toMatrix(camera2ai.Inverse())
    # 计算aj在ai下的位姿 位姿传递
    ai2aj_mat = ai2camera_mat.dot(camera2aj_mat)
    # 生成码j在码i坐标系下的位姿 对应的KDL Frame对象
    ai2aj_frame = pm.fromMatrix(ai2aj_mat)
    # 返回对象
    return pm.toMsg(ai2aj_frame)

def calculate_camera_distance(ai2camera, aj2camera):
    '''
    获取相机距离两个marker 中线的距离
    '''
    x = (ai2camera.p.x() + aj2camera.p.x()) / 2
    y = (ai2camera.p.y() + aj2camera.p.y()) / 2
    z = (ai2camera.p.z() + aj2camera.p.z()) / 2

    return math.sqrt(x**2 + y**2 + z**2)

def callback(marker_array):
    '''
    采集marker_array的数据
    {
        1:{
            2: [
                (distance, [x, y, z, r, p, y]),
                (distance, [x, y, z, r, p, y]),
                (distance, [x, y, z, r, p, y]),
            ]
        }
    }
    '''
    global package_path
    global sample_count
    global sample_done
    rospy.loginfo("Sample Num: {}".format(sample_count))
    if sample_count > MIN_SAMPLE_NUM and len(list(arucos_dict.keys())) == ARUCO_NUM:
        
        # 将数据存储在二进制文件中 二值化
        with open(package_path + '/data/arucos_dict.bin', 'wb') as f:
            pickle.dump(arucos_dict, f)

        signal = String()
        signal.data = "aruco码完成采样"
        sample_done_pub.publish(signal) # 发送完成采样的信号

        rospy.signal_shutdown("停止aruco码样本采样")
    elif len(list(arucos_dict.keys())) > ARUCO_NUM:
        # 画面中出现了大于最多aruco数的aruco
        rospy.logwarn('ERROR： 画面中的aruco多与预设值， 请调整画面中aruco的总个数。')
    sample_count += 1


    aruco_num = len(marker_array.markers)
    if aruco_num < 2:
        # 只有>=2个aruco才能计算相对位置
        rospy.loginfo("过少的aruco码的个数，aruco数量需要>=2")
        return

    aruco_frame_dict = {}

    for i in range(aruco_num):
        ai2camera = pm.fromMsg(marker_array.markers[i].pose.pose)
        # Note: 姿态修正跟ros中间件有关系 返回的message修改修改
        ai2camera.M.DoRotX(-math.pi/2) # 校正姿态 X轴逆向旋转90度弧度
        ai2camera.M.DoRotZ(math.pi) # 校正姿态 Z轴旋转180度
        aruco_frame_dict[marker_array.markers[i].id] = ai2camera
        
    # 计算转换
    aruco_id_list = list(aruco_frame_dict.keys())
    # 笛卡尔积 aruco id
    for (ai_id, aj_id) in product(aruco_id_list, aruco_id_list):
        if ai_id == aj_id:
            continue
        
        if ai_id not in arucos_dict:
            arucos_dict[ai_id] = {}

        if aj_id not in arucos_dict[ai_id]:
            arucos_dict[ai_id][aj_id] = []
        
        # 记录arucoj 参考arucoi的参照坐标
        aj_pose_ref_ai = transfrom_between_arucos(aruco_frame_dict[ai_id], aruco_frame_dict[aj_id])
        # 计算摄像头距离 两个aruco tag 中线的距离
        distance2camera = calculate_camera_distance(aruco_frame_dict[ai_id], aruco_frame_dict[aj_id])
        # 添加记录
        arucos_dict[ai_id][aj_id].append((distance2camera, aj_pose_ref_ai))



if __name__ == "__main__":
    rospy.init_node('collect_aruco_location', anonymous=True)

    aruco_subscriber = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, callback)
    sample_done_pub = rospy.Publisher("/aruco_sample_done", String, queue_size=1)
    rospy.spin()
    