#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import os


def publish_waypoints():
    rospy.init_node("waypoints_publisher", anonymous=True)
    waypoints_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=10)

    # Waypoints 좌표를 파일에서 읽어옵니다
    script_dir = os.path.dirname(__file__)  # 현재 스크립트 디렉토리
    file_path = os.path.join(script_dir, "waypoints.txt")
    waypoints = np.loadtxt(file_path, delimiter=",")

    pose_array = PoseArray()
    for x, y in waypoints:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose_array.poses.append(pose)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        waypoints_pub.publish(pose_array)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass
