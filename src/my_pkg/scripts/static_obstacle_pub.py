#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from f110_msgs.msg import Obstacle, ObstacleArray, WpntArray
import sys
sys.path.append('/home/patrick/f1_ws/src/my_pkg/scripts/')
from frenet_utils import FrenetConverter


class ObstacleArrayPublisher:
    def __init__(self):
        rospy.init_node("obstacle_array_publisher")

        # Publishers
        self.obstacle_pub = rospy.Publisher("/perception/obstacles", ObstacleArray, queue_size=10)
        self.marker_pub = rospy.Publisher("/perception/obstacles_markers", MarkerArray, queue_size=10)

        # Subscribers
        rospy.Subscriber("/global_path/optimal_trajectory_wpnt", WpntArray, self.global_path_callback)

        # Frenet Converter
        self.converter = None

        # Define static obstacles
        self.static_obstacles = [
            {
                "id": 2,
                "s_start": 8.2,
                "s_end": 7.8,
                "d_left": 0.2,
                "d_right": -0.2,
                "s_center": 8,
                "d_center": -0.1,
                "size": 0.2,
                "is_static": True,
                "is_visible": True
            },
        ]

        # Loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def global_path_callback(self, data):
        """
        Callback for receiving the global path and initializing the Frenet converter.
        """
        rospy.loginfo("[ObstacleArrayPublisher] Received global path.")
        waypoints_x = [wp.x_m for wp in data.wpnts]
        waypoints_y = [wp.y_m for wp in data.wpnts]
        self.converter = FrenetConverter(waypoints_x, waypoints_y)

    def publish_obstacles(self):
        rospy.loginfo("Publishing static obstacles and markers...")

        while not rospy.is_shutdown():
            if self.converter is None:
                rospy.logwarn("FrenetConverter not initialized. Waiting for global path...")
                self.rate.sleep()
                continue

            # Create ObstacleArray message
            obstacle_array_msg = ObstacleArray()
            obstacle_array_msg.header.stamp = rospy.Time.now()
            obstacle_array_msg.header.frame_id = "map"

            marker_array = MarkerArray()

            for obs_data in self.static_obstacles:
                # Create Obstacle message
                obstacle_msg = Obstacle()
                obstacle_msg.id = obs_data["id"]
                obstacle_msg.s_start = obs_data["s_start"]
                obstacle_msg.s_end = obs_data["s_end"]
                obstacle_msg.d_left = obs_data["d_left"]
                obstacle_msg.d_right = obs_data["d_right"]
                obstacle_msg.s_center = obs_data["s_center"]
                obstacle_msg.d_center = obs_data["d_center"]
                obstacle_msg.size = obs_data["size"]
                obstacle_msg.is_static = obs_data["is_static"]
                obstacle_msg.is_visible = obs_data["is_visible"]

                obstacle_array_msg.obstacles.append(obstacle_msg)

                # Convert s, d to x, y
                x, y = self.converter.get_cartesian(obs_data["s_center"], obs_data["d_center"])

                # Create Marker for RViz visualization
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "static_obstacles"
                marker.id = obs_data["id"]
                marker.type = Marker.CUBE  # Using cube to represent the obstacle
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0

                # Initialize orientation (identity quaternion)
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = obs_data["size"]
                marker.scale.y = obs_data["size"]
                marker.scale.z = 1.0  # Height of the cube
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0  # Blue color

                marker_array.markers.append(marker)

            # Publish obstacles and markers
            self.obstacle_pub.publish(obstacle_array_msg)
            self.marker_pub.publish(marker_array)

            rospy.loginfo(f"Published {len(self.static_obstacles)} obstacles.")
            self.rate.sleep()

if __name__ == "__main__":
    try:
        publisher = ObstacleArrayPublisher()
        publisher.publish_obstacles()
    except rospy.ROSInterruptException:
        pass
