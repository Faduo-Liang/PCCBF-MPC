#!/home/lfd/miniconda3/envs/pytorch/bin/python3
# coding:utf-8
import rospy
import sys
sys.path.append('/home/lfd/TB4_local_planning/MPC_Point_Based_CBF/onboard_detector/src/onboard_detector/scripts/yolo_detector')
import numpy as np
from yolo_detector import *

if __name__ == '__main__':
    # 创建节点
    rospy.init_node("hello")
    rospy.loginfo("hello world !")
    yolo_detector()
    rospy.spin()
