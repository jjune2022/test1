import os
import sys
import rospy
import numpy as np
import tf2_ros
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from camera_data.msg import ReferencePoses
from visualization_msgs.msg import Marker
from collections import deque
from tf.transformations import euler_from_quaternion


# status dictionary
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}


class Stanleycontroller:
    def __init__(self, hz=50, k=0.6, k2=0.2):
        rospy.init_node('Stanleycontroller')
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/ref_pos', ReferencePoses, self.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.local_points = []
        self.global_points = None
        # self.waypoints = []
        self.hz = hz
        self.dt = 1 / hz  # time step
        self.odom_pose = None  # position x, y, z, orientation x, y, z, w
        self.odom_twist = None  # linear x, y, z, angular x, y, z
        self.x0 = 0.0
        self.y0 = 0.0        
        self.k = k            # Stanley controller gain for angle correction
        self.k2 = k2          # Stanley controller gain for distance correction
        self.min_vel = 0.0
        self.max_vel = 0.1
        self.angles = np.linspace(0, np.pi, 100)
        self.waypoints = [(0.01*i, 0.01*i) for i in range(500)] + [(5 - 0.01*i, 5 + 0.01*i) for i in range(500)] + [(0 - 0.01*i, 10 - 0.01*i) for i in range(500)] + [(-5 + 0.01*i, 5 - 0.01*i) for i in range(500)]
        self.publish_way_points() #way point 표시

    # odom은 bringup으로부터 자동으로 publish됨(global좌표)
    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist

    def waypoints_callback(self, data):
        self.x0, self.y0, self.theta0 = self.odom_pose.position.x, self.odom_pose.position.y, \
                                        self.get_yaw_from_quaternion(self.odom_pose.orientation)  # world
        # self.global_points = self.our_points[self.marker_id: self.marker_id + self.horizon]
        # self.local_points = self.global2local(self.global_points, self.x0, self.y0, self.theta0)
        # self.publish_global_points() #global point 표시
        self.run_stanley()
    
    
    
    def run_stanley(self):
        if len(self.waypoints) < 2 or self.odom_pose is None:
            return
        
        # Extract current state
        self.x0, self.y0, self.theta0 = self.odom_pose.position.x, self.odom_pose.position.y, self.get_yaw_from_quaternion(self.odom_pose.orientation)
        v0 = self.odom_twist.linear.x

        # Find the closest waypoint
        min_dist = float('inf')
        closest_idx = 0
        for i, (wx, wy) in enumerate(self.waypoints):
            dist = np.sqrt((self.x0 - wx)**2 + (self.y0 - wy)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Waypoint to track is the closest one or next waypoint
        target_wp = self.waypoints[closest_idx]

        # Compute the heading error
        path_angle = np.arctan2(target_wp[1] - self.y0, target_wp[0] - self.x0) #rad
        path_distance = np.sqrt((target_wp[0] - self.x0) ** 2 + (target_wp[1] - self.y0) ** 2)
        heading_error = path_angle - self.theta0

        # Normalize heading error to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        
        # Stanley control
        angle_correction = heading_error + np.arctan2(self.k * path_distance, v0)
        # angular velocity pet second
        # omega = angle_correction /self.dt
        omega = angle_correction / 1 #/ref_pos 1초마다 publish한다고 가정
        omega = np.clip(omega, -3, 3)  # Limit angular velocity

        # Control effort
        velocity = np.clip(v0, self.min_vel, self.max_vel)

        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = velocity
        control_cmd.angular.z = omega
        self.pub.publish(control_cmd)
        
        # Debug info
        rospy.loginfo(f"Velocity: {velocity}, Angular Velocity: {omega}, Heading Error: {heading_error}, Target: {target_wp}")


    def publish_way_points(self):
        """
        Publish the transformed points to RViz as a Marker
        """
        for point in self.waypoints:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "transformed_points"
            marker.id = self.marker_id  # Unique ID for each point
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            # marker.pose.position.x = (point[0] - self.x0) * 0.01 + self.x0
            # marker.pose.position.y = (point[1] - self.y0) * 0.01 + self.y0
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            self.marker_id += 1  # Increment the marker ID for the next point


    def global2local(self, points, x0, y0, theta0):
        local_points = []
        cos_theta = np.cos(theta0)
        sin_theta = np.sin(theta0)
        
        for point in points:
            x_prime = point[0] - x0
            y_prime = point[1] - y0
            x_local = cos_theta * x_prime + sin_theta * y_prime
            y_local = -sin_theta * x_prime + cos_theta * y_prime
            local_points.append((x_local, y_local))

        return local_points

    def local2global(self, points, x0, y0, theta0):
        global_points = []
        cos_theta = np.cos(-theta0)
        sin_theta = np.sin(-theta0)
        
        for point in points:
            x_prime = cos_theta * point[0] + sin_theta * point[1]
            y_prime = -sin_theta * point[0] + cos_theta * point[1]
            global_points.append((x0 + x_prime, y0 + y_prime))
            
        return global_points


    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion into yaw angle (in radians)
        """
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]


if __name__ == "__main__":
    hz = 50
    rospy.init_node("Stanleycontroller")
    node = Stanleycontroller(hz)
    rate = rospy.Rate(hz)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()