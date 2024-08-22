import rospy
import numpy as np
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from camera_data.msg import ReferencePoses

class StanleyPlanner:
    def __init__(self, hz=50, k=0.6, k2=0.2):
        rospy.init_node('StanleyPlanner')
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
        if closest_idx + 1 < len(self.waypoints):
            target_wp = self.waypoints[closest_idx + 1]
        else:
            target_wp = self.waypoints[closest_idx]

        # Compute the heading error
        path_angle = np.arctan2(target_wp[1] - self.y0, target_wp[0] - self.x0)
        path_distance = np.sqrt((target_wp[0] - self.x0) ** 2 + (target_wp[1] - self.y0) ** 2)
        heading_error = path_angle - self.theta0

        # Normalize heading error to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        
        # Stanley control
        angle_correction = heading_error + np.arctan2(self.k * path_distance, v0)
        omega = self.k * angle_correction
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

    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion into yaw angle (in radians)
        """
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

if __name__ == "__main__":
    try:
        node = StanleyPlanner(hz=50)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
