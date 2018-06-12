#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg


rospackage = rospkg.RosPack()

print(rospackage.get_path('aruco_mapping2'))

rospy.init_node('test_path', anonymous=True)
