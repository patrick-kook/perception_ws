#!/usr/bin/env python3

import rospy
import tf
import time
from frenet_utils import FrenetConverter
from f110_msgs.msg import WpntArray, ObstacleArray, Obstacle as ObstacleMessage
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
from tf.transformations import quaternion_from_euler

def normalize_s(x, track_length):
    x = x % (track_length)
    if x > track_length / 2:
        x -= track_length
    return x

class Obstacle:
    def __init__(self, x, y, size, theta) -> None:
        self.center_x = x
        self.center_y = y
        self.size = size
        self.theta = theta

class Detect:
    def __init__(self) -> None:
        self.converter = None
        rospy.init_node('StaticObstacleDetector', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.laserCb)
        rospy.Subscriber('/global_path/optimal_trajectory_wpnt', WpntArray, self.pathCb)

        # Publishers
        self.obstacles_msg_pub = rospy.Publisher('/perception/detection/raw_obstacles', ObstacleArray, queue_size=5)
        self.obstacles_marker_pub = rospy.Publisher('/perception/obstacles_markers', MarkerArray, queue_size=5)

        # Parameters
        self.rate = rospy.get_param("/detect/rate", 10)
        self.lambda_angle = rospy.get_param("/detect/lambda", 0.5) * math.pi / 180
        self.sigma = rospy.get_param("/detect/sigma", 0.1)
        self.min_2_points_dist = rospy.get_param("/detect/min_2_points_dist", 0.1)
        self.min_obs_size = rospy.get_param("/detect/min_obs_size", 10)
        self.max_obs_size = rospy.get_param("/detect/max_obs_size", 0.5)
        self.max_viewing_distance = rospy.get_param("/detect/max_viewing_distance", 9)
        self.boundaries_inflation = rospy.get_param("/detect/boundaries_inflation", 0.1)

        # Variables
        self.waypoints = None
        self.biggest_d = None
        self.smallest_d = None
        self.s_array = None
        self.d_right_array = None
        self.d_left_array = None
        self.track_length = None
        self.car_s = 0
        self.scans = None
        self.current_stamp = None
        self.tracked_obstacles = []
        self.tf_listener = tf.TransformListener()

        while self.waypoints is None:
            rospy.sleep(0.1)
            print("[Static Obstacle Detection]: waiting for waypoints...")

        self.converter = self.initialize_converter()

    def shutdown(self):
        rospy.logwarn('Static Obstacle Detection is shutdown')

    def initialize_converter(self):
        converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo("[Static Obstacle Detection]: initialized FrenetConverter object")
        return converter

    def laserCb(self, msg):
        self.scans = msg

    def pathCb(self, data):
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        if self.s_array is None and self.converter is not None:
            waypoint_array = data.wpnts
            points = []
            self.s_array = []
            self.d_right_array = []
            self.d_left_array = []
            for waypoint in waypoint_array:
                self.s_array.append(waypoint.s_m)
                self.d_right_array.append(waypoint.d_right - self.boundaries_inflation)
                self.d_left_array.append(waypoint.d_left - self.boundaries_inflation)
                resp = self.converter.get_cartesian(waypoint.s_m, -waypoint.d_right + self.boundaries_inflation)
                points.append(Point(resp[0], resp[1], 0))
                resp = self.converter.get_cartesian(waypoint.s_m, waypoint.d_left - self.boundaries_inflation)
                points.append(Point(resp[0], resp[1], 0))
            self.smallest_d = min(self.d_right_array + self.d_left_array)
            self.biggest_d = max(self.d_right_array + self.d_left_array)
            self.track_length = data.wpnts[-1].s_m

    def clearmarkers(self) -> list:
        marker = Marker()
        marker.action = 3
        return [marker]

    def laserPointOnTrack(self, s, d) -> bool:
        if normalize_s(s - self.car_s, self.track_length) > self.max_viewing_distance:
            return False
        if abs(d) >= self.biggest_d:
            return False
        if abs(d) <= self.smallest_d:
            return True
        idx = bisect_left(self.s_array, s)
        if idx:
            idx -= 1
        if d <= -self.d_right_array[idx] or d >= self.d_left_array[idx]:
            return False
        return True

    def scans2ObsPointCloud(self):
        l = self.lambda_angle
        d_phi = self.scans.angle_increment
        sigma = self.sigma
        self.current_stamp = rospy.Time.now()
        try:
            lct = self.tf_listener.getLatestCommonTime("map", "laser")
        except:
            rospy.logerr("[Static Obstacle Detection]: lookup Transform between map and laser not possible")
            return []
        trans, quat = self.tf_listener.lookupTransform('/map', '/laser', lct)
        T = np.array(trans)
        R = tf.transformations.quaternion_matrix(quat)
        angles = np.linspace(self.scans.angle_min, self.scans.angle_max, len(self.scans.ranges))
        x_lf = (self.scans.ranges * np.cos(angles)).flatten()
        y_lf = (self.scans.ranges * np.sin(angles)).flatten()
        z_lf = (-T[0] * np.ones(len(self.scans.ranges))).flatten()
        xyz_lf = np.vstack((x_lf, y_lf, z_lf, np.ones(len(self.scans.ranges))))
        H_l2m = R
        H_l2m[:3, -1] = T
        xyz_map = H_l2m @ xyz_lf
        cloudPoints_list = [(xyz_map[0, i], xyz_map[1, i]) for i in range(xyz_map.shape[1])]
        objects_pointcloud_list = [[[cloudPoints_list[0][0], cloudPoints_list[0][1]]]]
        for idx, point in enumerate(cloudPoints_list):
            if idx == 0:
                continue
            dist = math.sqrt(point[0] ** 2 + point[1] ** 2)
            d_max = (dist * math.sin(d_phi) / math.sin(l - d_phi) + 3 * sigma) / 2
            if math.dist([cloudPoints_list[idx - 1][0], cloudPoints_list[idx - 1][1]], [point[0], point[1]]) > d_max:
                objects_pointcloud_list.append([[point[0], point[1]]])
            else:
                objects_pointcloud_list[-1].append([point[0], point[1]])
        x_points = [obs[int(len(obs) / 2)][0] for obs in objects_pointcloud_list]
        y_points = [obs[int(len(obs) / 2)][1] for obs in objects_pointcloud_list]
        s_points, d_points = self.converter.get_frenet(np.array(x_points), np.array(y_points))
        remove_array = [object for idx, object in enumerate(objects_pointcloud_list) if len(object) < self.min_obs_size or not self.laserPointOnTrack(s_points[idx], d_points[idx])]
        for object in remove_array:
            objects_pointcloud_list.remove(object)
        markers_array = self.create_markers(objects_pointcloud_list)
        self.publish_markers(markers_array)
        return objects_pointcloud_list

    def create_markers(self, objects_pointcloud_list):
        markers_array = []
        for idx, object in enumerate(objects_pointcloud_list):
            marker = self.create_marker(object[0][0], object[0][1], idx * 10)
            markers_array.append(marker)
            marker = self.create_marker(object[-1][0], object[-1][1], idx * 10 + 2)
            markers_array.append(marker)
        return markers_array

    def create_marker(self, x, y, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.current_stamp
        marker.id = marker_id
        marker.type = marker.SPHERE
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 0.5
        marker.color.g = 1.0
        marker.color.r = 0.0
        marker.color.b = 0.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1
        return marker

    def publish_markers(self, markers_array):
        self.obstacles_marker_pub.publish(self.clearmarkers())
        self.obstacles_marker_pub.publish(markers_array)

    def obsPointClouds2obsArray(self, objects_pointcloud_list):
        current_obstacle_array = []
        min_dist = self.min_2_points_dist
        for obstacle in objects_pointcloud_list:
            theta = np.linspace(0, np.pi / 2 - np.pi / 180, 90)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            distance1 = np.dot(obstacle, [cos_theta, sin_theta])
            distance2 = np.dot(obstacle, [-sin_theta, cos_theta])
            D10 = -distance1 + np.amax(distance1, axis=0)
            D11 = distance1 - np.amin(distance1, axis=0)
            D20 = -distance2 + np.amax(distance2, axis=0)
            D21 = distance2 - np.amin(distance2, axis=0)
            min_array = np.argmin([np.linalg.norm(D10, axis=0), np.linalg.norm(D11, axis=0)], axis=0)
            D10 = np.transpose(D10)
            D11 = np.transpose(D11)
            D10[min_array == 1] = D11[min_array == 1]
            D10 = np.transpose(D10)
            min_array = np.argmin([np.linalg.norm(D20, axis=0), np.linalg.norm(D21, axis=0)], axis=0)
            D20 = np.transpose(D20)
            D21 = np.transpose(D21)
            D20[min_array == 1] = D21[min_array == 1]
            D20 = np.transpose(D20)
            D = np.minimum(D10, D20)
            D[D < min_dist] = min_dist
            theta_opt = np.argmax(np.sum(np.reciprocal(D), axis=0)) * np.pi / 180
            distances1 = np.dot(obstacle, [np.cos(theta_opt), np.sin(theta_opt)])
            distances2 = np.dot(obstacle, [-np.sin(theta_opt), np.cos(theta_opt)])
            max_dist1 = np.max(distances1)
            min_dist1 = np.min(distances1)
            max_dist2 = np.max(distances2)
            min_dist2 = np.min(distances2)
            if np.var(distances2) > np.var(distances1):
                if np.linalg.norm(-distances1 + max_dist1) < np.linalg.norm(distances1 - min_dist1):
                    corner1 = [np.cos(theta_opt) * max_dist1 - np.sin(theta_opt) * min_dist2, np.sin(theta_opt) * max_dist1 + np.cos(theta_opt) * min_dist2]
                    corner2 = [np.cos(theta_opt) * max_dist1 - np.sin(theta_opt) * max_dist2, np.sin(theta_opt) * max_dist1 + np.cos(theta_opt) * max_dist2]
                else:
                    corner1 = [np.cos(theta_opt) * min_dist1 - np.sin(theta_opt) * max_dist2, np.sin(theta_opt) * min_dist1 + np.cos(theta_opt) * max_dist2]
                    corner2 = [np.cos(theta_opt) * min_dist1 - np.sin(theta_opt) * min_dist2, np.sin(theta_opt) * min_dist1 + np.cos(theta_opt) * min_dist2]
            else:
                if np.linalg.norm(-distances2 + max_dist2) < np.linalg.norm(distances2 - min_dist2):
                    corner1 = [np.cos(theta_opt) * max_dist1 - np.sin(theta_opt) * max_dist2, np.sin(theta_opt) * max_dist1 + np.cos(theta_opt) * max_dist2]
                    corner2 = [np.cos(theta_opt) * min_dist1 - np.sin(theta_opt) * max_dist2, np.sin(theta_opt) * min_dist1 + np.cos(theta_opt) * max_dist2]
                else:
                    corner1 = [np.cos(theta_opt) * min_dist1 - np.sin(theta_opt) * min_dist2, np.sin(theta_opt) * min_dist1 + np.cos(theta_opt) * min_dist2]
                    corner2 = [np.cos(theta_opt) * max_dist1 - np.sin(theta_opt) * min_dist2, np.sin(theta_opt) * max_dist1 + np.cos(theta_opt) * min_dist2]
            colVec = np.array([corner2[0] - corner1[0], corner2[1] - corner1[1]])
            orthVec = np.array([-colVec[1], colVec[0]])
            center = corner1 + 0.5 * colVec + 0.5 * orthVec
            current_obstacle_array.append(Obstacle(center[0], center[1], np.linalg.norm(colVec), theta_opt))
        return current_obstacle_array

    def publishObstaclesMessage(self):
        obstacles_array_message = ObstacleArray()
        obstacles_array_message.header.stamp = self.current_stamp
        obstacles_array_message.header.frame_id = "map"
        for obstacle in self.tracked_obstacles:
            obsMsg = ObstacleMessage()
            obsMsg.id = obstacle.id
            obsMsg.s_center = obstacle.center_x
            obsMsg.d_center = obstacle.center_y
            obsMsg.size = obstacle.size
            obstacles_array_message.obstacles.append(obsMsg)
        self.obstacles_msg_pub.publish(obstacles_array_message)

    def main(self):
        rate = rospy.Rate(self.rate)
        rospy.loginfo('[Static Obstacle Detection]: Waiting for global waypoints')
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.loginfo('[Static Obstacle Detection]: Ready')
        while not rospy.is_shutdown():
            objects_pointcloud_list = self.scans2ObsPointCloud()
            current_obstacles = self.obsPointClouds2obsArray(objects_pointcloud_list)
            self.tracked_obstacles = current_obstacles
            self.publishObstaclesMessage()
            rate.sleep()

if __name__ == '__main__':
    detect = Detect()
    detect.main()