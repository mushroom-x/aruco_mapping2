#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
测试 PyKDL
'''
import PyKDL
import pickle
import rospy
import tf_conversions.posemath as pm
from numpy import linalg as LA
if __name__=="__main__":
    rospy.init_node('draw_aruco_map', anonymous=True)
    arucos_dict_path = "/home/fange/catkin_ws/src/aruco_mapping2/data/arucos_dict.bin"

    f = open(arucos_dict_path, 'rb')
    aruco_dict = pickle.load(f)
    f.close()

    aruco_ids = list(aruco_dict.keys())
    rospy.loginfo("aruco_ids: \n{}".format(aruco_ids))
    
    aruco_id = aruco_ids[1]
    rospy.loginfo("object type: \n{}".format(type(aruco_dict[aruco_id])))
    rospy.loginfo("to matrix\n")
    rospy.loginfo("{}".format(pm.toMatrix(aruco_dict[aruco_id])))
    matA = pm.toMatrix(aruco_dict[aruco_id])
    matB = LA.inv(matA)

    rospy.loginfo("A * B = \n{}".format(matA.dot(matB)))