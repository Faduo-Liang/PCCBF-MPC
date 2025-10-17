#!/home/lfd/miniconda3/envs/pytorch/bin/python3
# coding:utf-8
# import sys
# sys.path.append('/home/lfd/TB4_local_planning/MPC_Point_Based_CBF/onboard_detector/src/onboard_detector/scripts/yolo_detector')
import rospy
import numpy as np
from yolo_detector import *


def main():
	rospy.init_node("yolo_detector_node")
	rospy.loginfo("hello world!!!")
	print("[onboardDetector]: yolo detector node start...")
	yolo_detector()
	rospy.spin()

if __name__=="__main__":
	main()
	
