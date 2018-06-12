#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pickle
# import tf_conversions.posemath as pm
import PyKDL
from geometry_msgs.msg import Pose
import numpy as np

'''
TODO 以 1为参考码

'''

'''
# 参考码
BASE_ARUCO_ID = 5

# 参考地图
# 数据格式 (x, y, z, qx, qy, qz, w)
REF_MAP = {
    1: (-0.343, -0.265, 0.0, 0.0, 0.0, 0.0, 1),
    2: (-0.343, 0.0, 0.0, 0.0, 0.0, 0.0, 1), 
    3: (-0.343, 0.265, 0.0, 0.0, 0.0, 0.0, 1),
    4: (0.0, -0.265, 0.0, 0.0, 0.0, 0.0, 1),
    5: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1),
    6: (0.0, 0.265, 0.0, 0.0, 0.0, 0.0, 1),
    7: (0.343, -0.265, 0.0, 0.0, 0.0, 0.0, 1),
    8: (0.343, 0.0, 0.0, 0.0, 0.0, 0.0, 1),
    9: (0.343, 0.265, 0.0, 0.0, 0.0, 0.0, 1),
}
'''

# 参考码
BASE_ARUCO_ID = 1

# 参考地图
# 数据格式 (x, y, z, qx, qy, qz, w)
REF_MAP = {
    1: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1),
    2: (0.0, 0.265, 0.0, 0.0, 0.0, 0.0, 1), 
    3: (0.0, 0.530, 0.0, 0.0, 0.0, 0.0, 1),
    4: (0.343, 0.0, 0.0, 0.0, 0.0, 0.0, 1),
    5: (0.343, 0.265, 0.0, 0.0, 0.0, 0.0, 1),
    6: (0.343, 0.530, 0.0, 0.0, 0.0, 0.0, 1),
    7: (0.686, 0.0, 0.0, 0.0, 0.0, 0.0, 1),
    8: (0.686, 0.265, 0.0, 0.0, 0.0, 0.0, 1),
    9: (0.686, 0.530, 0.0, 0.0, 0.0, 0.0, 1),
}

def extract_frame_data(frame):
    # frame 是geometry_msg Pose类型
    x = frame.position.x
    y = frame.position.y
    z = frame.position.z
    qx = frame.orientation.x
    qy = frame.orientation.y
    qz = frame.orientation.z
    w = frame.orientation.w

    return (x, y, z, qx, qy, qz, w)


def get_output_aruco_map():
    '''
    从日志文件中获取地图
    输出的地图数据格式与REAL_MAP一致
    '''
    output_map = {}

    with open('/home/fange/catkin_ws/src/aruco_mapping2/data/aruco_map.bin', 'rb') as f:
        arucos_map = pickle.load(f)

    for aruco_id, frame in arucos_map.items():
        print("aruco id: {}".format(aruco_id))
        print("frame: {}".format(frame))
        print(extract_frame_data(frame))
        output_map[aruco_id] = extract_frame_data(frame)
    return output_map


def cal_mapping_error(ref_map, output_map):
    '''
    计算建图误差(均值)
    '''
    err_sum = np.float32([0,0,0,0,0,0,0])
    count = 0
    for aruco_id in ref_map.keys():
        print("\n======ARUCO_ID = {}========\n".format(aruco_id))
        count += 1
        err = np.float32(ref_map[aruco_id]) - np.float32(output_map[aruco_id])
        print("参考 {}".format(ref_map[aruco_id]))
        print("实际 {}".format(output_map[aruco_id]))
        print("Error {}: {}".format(aruco_id, err))
        err_sum += np.abs(err)
    return err_sum / count


if __name__ == "__main__":
    output_map = get_output_aruco_map()
    err = cal_mapping_error(REF_MAP, output_map)
    print("建图误差： (x, y, z, qx, qy, qz, w)")
    np.set_printoptions(suppress=True, precision=4)
    print(err)