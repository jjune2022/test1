#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import math
from tf.transformations import euler_from_quaternion

PI = 3.14159265
E = 2.718281828

class StanleyPlanner:
    def __init__(self):
        rospy.init_node('Stanley', anonymous=True)

        # Parameters
        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'base_link')
        self.v = rospy.get_param('~robot_cmd_vel', 0.2)
        self.k = rospy.get_param('~stanley_ang_gain', 0.6)
        self.k2 = rospy.get_param('~stanley_trans_gain', 0.2)

        # Subscribers
        self.path_sub = rospy.Subscriber('/move_base/NavfnROS/plan', nav_msgs.msg.Path, self.path_callback)
        self.odometry_sub = rospy.Subscriber('/odom', nav_msgs.msg.Odometry, self.pose_callback)

        # Publishers
        self.speed_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.plot_pub = rospy.Publisher('/plotMsgs', std_msgs.msg.String, queue_size=1)

        # Variables
        self.i = 0
        self.step = 0
        self.path_size = 0
        self.condition = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def pose_callback(self, msg):
        current_time = rospy.Time.now().to_sec()
        if not self.condition or (current_time - self.proslo) > 1:
            return

        self.state = msg

        while True:
            try:
                transform = self.tf_buffer.lookup_transform(self.map_frame_id, self.robot_frame_id, rospy.Time(0))
                break
            except tf2_ros.TransformException:
                continue

        # Extracting the robot's position and orientation
        robot_X = transform.transform.translation.x
        robot_Y = transform.transform.translation.y

        # Check distance to goal
        modulDistance1 = math.sqrt(
            (self.path.poses[-1].pose.position.y - robot_Y) ** 2 +
            (self.path.poses[-1].pose.position.x - robot_X) ** 2
        )
        if modulDistance1 < 0.5:
            cmd_vel = geometry_msgs.msg.Twist()
            cmd_vel.angular.z = 0
            cmd_vel.linear.x = 0
            self.speed_pub.publish(cmd_vel)
            return

        # Find closest point on path
        for h in range(self.i + 1, len(self.path.poses)):
            pastModul = math.sqrt(
                (self.path.poses[h - 1].pose.position.y - robot_Y) ** 2 +
                (self.path.poses[h - 1].pose.position.x - robot_X) ** 2
            )
            modulDistanceForI = math.sqrt(
                (self.path.poses[h].pose.position.y - robot_Y) ** 2 +
                (self.path.poses[h].pose.position.x - robot_X) ** 2
            )
            if pastModul - modulDistanceForI < 0:
                self.i = h - 1
                break

        if self.path_size < self.i + self.step:
            self.step = max(0, self.path_size - self.i)

        # Calculate the cross product to determine the side of the path
        a11 = self.path.poses[self.i + self.step].pose.position.x - self.path.poses[self.i].pose.position.x
        a12 = self.path.poses[self.i + self.step].pose.position.y - self.path.poses[self.i].pose.position.y
        a21 = robot_X - self.path.poses[self.i].pose.position.x
        a22 = robot_Y - self.path.poses[self.i].pose.position.y
        Xprod = a11 * a22 - a12 * a21

        x1 = -robot_X + self.path.poses[self.i].pose.position.x
        y1 = -robot_Y + self.path.poses[self.i].pose.position.y
        x2_1 = x2_2 = y2_1 = y2_2 = 0

        if x1 != 0:
            y2_1 = 1
            y2_2 = -1
            x2_1 = -(y1 / x1) * y2_1
            x2_2 = -(y1 / x1) * y2_2
        else:
            x2_1 = 1
            x2_2 = -1

        x2 = self.path.poses[self.i + 1].pose.position.x - self.path.poses[self.i].pose.position.x
        y2 = self.path.poses[self.i + 1].pose.position.y - self.path.poses[self.i].pose.position.y

        cos1 = (x2_1 * x2 + y2_1 * y2) / (math.sqrt(x2_1 ** 2 + y2_1 ** 2) * math.sqrt(x2 ** 2 + y2 ** 2))
        cos2 = (x2_2 * x2 + y2_2 * y2) / (math.sqrt(x2_2 ** 2 + y2_2 ** 2) * math.sqrt(x2 ** 2 + y2 ** 2))
        pathAngleCross = math.atan2(y2_1 if cos1 >= 0 else y2_2, x2_1 if cos1 >= 0 else x2_2)

        sign = -1 if Xprod > 0 else 1 if Xprod < 0 else 0

        # Calculate the path angle
        roll, pitch, anglePath = euler_from_quaternion((
            self.path.poses[self.i].pose.orientation.x,
            self.path.poses[self.i].pose.orientation.y,
            self.path.poses[self.i].pose.orientation.z,
            self.path.poses[self.i].pose.orientation.w
        ))
        anglePath = math.atan2(
            self.path.poses[self.i + self.step].pose.position.y - self.path.poses[self.i].pose.position.y,
            self.path.poses[self.i + self.step].pose.position.x - self.path.poses[self.i].pose.position.x
        )

        # Correct the angles
        if anglePath > PI:
            anglePath -= 2 * PI
        elif anglePath < -PI:
            anglePath += 2 * PI

        delta = -self.yaw + anglePath

        # Calculate the Stanley control
        modulDistance = math.sqrt(
            (self.path.poses[self.i].pose.position.y - robot_Y) ** 2 +
            (self.path.poses[self.i].pose.position.x - robot_X) ** 2
        )

        angle = self.k2 * (delta + sign * math.atan2(self.k * modulDistance / self.v, 1))
        omega = angle / (current_time - self.proslo)

        kvel = pow(E, -7 * abs(delta))
        velocity = 0.2 * kvel

        if abs(delta) >= PI:
            velocity = 0

        omega = max(min(omega, 3), -3)

        cmd_vel = geometry_msgs.msg.Twist()
        cmd_vel.angular.z = omega
        cmd_vel.linear.x = velocity

        self.speed_pub.publish(cmd_vel)

        # Publish analysis
        plot_msgs = std_msgs.msg.String()
        plot_msgs.data = "omega: {:.2f}, error: {:.2f}, pathX: {:.2f}, pathY: {:.2f}, robotX: {:.2f}, robotY: {:.2f}, modulDistance: {:.2f}, pathAngle: {:.2f}".format(
            omega, delta, self.path.poses[self.i].pose.position.x, self.path.poses[self.i].pose.position.y,
            robot_X, robot_Y, modulDistance, anglePath
        )
        self.plot_pub.publish(plot_msgs)

    def path_callback(self, msg):
        self.i = 0
        self.path = msg
        self.condition = True

        while True:
            try:
                transform = self.tf_buffer.lookup_transform(self.map_frame_id, self.robot_frame_id, rospy.Time(0))
                break
            except tf2_ros.TransformException:
                continue

        robot_X = transform.transform.translation.x
        robot_Y = transform.transform.translation.y

        self.path_size = len(self.path.poses)
        if self.path_size == 0:
            self.condition = False

        next_step = 0
        index_next = 0

        for r in range(self.path_size):
            modulDistance = math.sqrt(
                (self.path.poses[r].pose.position.y - self.path.poses[index_next].pose.position.y) ** 2 +
                (self.path.poses[r].pose.position.x - self.path.poses[index_next].pose.position.x) ** 2
            )
            next_step += 1
            if modulDistance >= 0.05:
                self.step = (self.step + next_step) / 2
                next_step = 0
                index_next = r

if __name__ == '__main__':
    try:
        StanleyPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
