#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_markers():
    rospy.init_node('marker_example_node')

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Arrow marker
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "base_link"
        arrow_marker.header.stamp = rospy.Time.now()
        arrow_marker.ns = "example"
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = 1.0
        arrow_marker.pose.position.y = 0.0
        arrow_marker.pose.position.z = 0.0
        arrow_marker.pose.orientation.x = 0.0
        arrow_marker.pose.orientation.y = 0.0
        arrow_marker.pose.orientation.z = 0.0
        arrow_marker.pose.orientation.w = 1.0
        arrow_marker.scale.x = 1.0  # Arrow length
        arrow_marker.scale.y = 0.1  # Arrow width
        arrow_marker.scale.z = 0.1  # Arrow height
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        # Sphere marker
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "base_link"
        sphere_marker.header.stamp = rospy.Time.now()
        sphere_marker.ns = "example"
        sphere_marker.id = 1
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = 2.0
        sphere_marker.pose.position.y = 0.0
        sphere_marker.pose.position.z = 0.0
        sphere_marker.pose.orientation.x = 0.0
        sphere_marker.pose.orientation.y = 0.0
        sphere_marker.pose.orientation.z = 0.0
        sphere_marker.pose.orientation.w = 1.0
        sphere_marker.scale.x = 0.5
        sphere_marker.scale.y = 0.5
        sphere_marker.scale.z = 0.5
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 1.0

        # Text marker
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "example"
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.0
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.5  # Text height
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = "Hello, RViz!"

        # Publish markers
        marker_pub.publish(arrow_marker)
        marker_pub.publish(sphere_marker)
        marker_pub.publish(text_marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_markers()
    except rospy.ROSInterruptException:
        pass
