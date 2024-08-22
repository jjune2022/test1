import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Polygon
from std_msgs.msg import UInt8, Int32
import numpy as np

class BroadcastRviz:
    def __init__(self):
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.state_pub = rospy.Publisher('/visualization_values', MarkerArray, queue_size=10)

        rospy.Subscriber('/points_topic', Polygon, self.points_callback)
        rospy.Subscriber('/state_speed', UInt8, self.speed_callback)
        rospy.Subscriber('/state_steer', Int32, self.steer_callback)
        rospy.Subscriber('/state_brake', UInt8, self.brake_callback)
        rospy.Subscriber('/state_encoder', Int32, self.encoder_callback)

        # Set up initial values
        self.setup_vehicle_model()
        self.setup_lines()
        self.speed = 0
        self.steer = 0
        self.brake = 0
        self.encoder = 0


    def setup_vehicle_model(self):
        # Create a Marker for the vehicle model
        self.vehicle_marker = Marker()
        self.vehicle_marker.header.frame_id = "map"
        self.vehicle_marker.ns = "vehicle"
        self.vehicle_marker.id = 0
        self.vehicle_marker.type = Marker.CUBE
        self.vehicle_marker.pose.orientation = Quaternion(0, 0, 0, 1)
        self.vehicle_marker.scale.x = 1.4
        self.vehicle_marker.scale.y = 2.0
        self.vehicle_marker.scale.z = 0.5
        self.vehicle_marker.color.r = 1.0
        self.vehicle_marker.color.g = 0.0
        self.vehicle_marker.color.b = 0.0
        self.vehicle_marker.color.a = 1.0
        self.vehicle_marker.pose.position.x = 0.0
        self.vehicle_marker.pose.position.y = 0.0
        self.vehicle_marker.pose.position.z = self.vehicle_marker.scale.z / 2
        

    def setup_lines(self):
        # Create markers for the lines
        self.line_marker1 = Marker()
        self.line_marker1.header.frame_id = "map"
        self.line_marker1.ns = "lines"
        self.line_marker1.id = 1
        self.line_marker1.type = Marker.LINE_STRIP
        self.line_marker1.action = Marker.ADD
        self.line_marker1.pose.orientation = Quaternion(-1, 0, 0, 0)
        self.line_marker1.scale.x = 0.1
        self.line_marker1.color.r = 0.0
        self.line_marker1.color.g = 1.0
        self.line_marker1.color.b = 0.0
        self.line_marker1.color.a = 1.0
        self.line_marker1.pose.position.x = -1.4
        self.line_marker1.pose.position.y = 6.0

        self.line_marker2 = Marker()
        self.line_marker2.header.frame_id = "map"
        self.line_marker2.ns = "lines"
        self.line_marker2.id = 2
        self.line_marker2.type = Marker.LINE_STRIP
        self.line_marker2.action = Marker.ADD
        self.line_marker2.pose.orientation = Quaternion(-1, 0, 0, 0)
        self.line_marker2.scale.x = 0.1
        self.line_marker2.color.r = 1.0
        self.line_marker2.color.g = 0.0
        self.line_marker2.color.b = 0.0
        self.line_marker2.color.a = 1.0
        self.line_marker2.pose.position.x = -1.4
        self.line_marker2.pose.position.y = 6.0

    def points_callback(self, msg):
        # Clear existing points
        self.line_marker1.points = []
        self.line_marker2.points = []
        
        # Populate markers with points from the Polygon message and convert cm to m
        half = len(msg.points) // 2
        for i, p in enumerate(msg.points):
            point = Point()
            point.x = p.x / 100.0  # Convert cm to m
            point.y = p.y / 100.0  # Convert cm to m
            point.z = 0
            if i < half:
                self.line_marker1.points.append(point)
                self.line_marker1.header.stamp = rospy.Time.now()
                
            else:
                self.line_marker2.points.append(point)
                self.line_marker2.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.line_marker1)
            self.marker_pub.publish(self.line_marker2)
            self.marker_pub.publish(self.vehicle_marker)



    def create_marker(self, id, text, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = text
        marker.pose.position.x += len(text) * 0.1
        return marker

    def update_markers(self):
        marker_array = MarkerArray()
        marker_array.markers.append(self.create_marker(3, f"Speed: {self.speed}", [4, 6, 0]))
        marker_array.markers.append(self.create_marker(4, f"Steer: {self.steer}", [4, 5.5, 0]))
        marker_array.markers.append(self.create_marker(5, f"Brake: {self.brake}", [4, 5, 0]))
        marker_array.markers.append(self.create_marker(6, f"Encoder: {self.encoder}", [4, 4.5, 0]))
        self.state_pub.publish(marker_array)

    def speed_callback(self, msg):
        self.speed = msg.data
        self.update_markers()

    def steer_callback(self, msg):
        self.steer = round(msg.data / 71, 4)
        self.update_markers()

    def brake_callback(self, msg):
        self.brake = msg.data
        self.update_markers()

    def encoder_callback(self, msg):
        self.encoder = msg.data
        self.update_markers()


if __name__ == '__main__':
    rospy.init_node('Broadcaster')
    node = BroadcastRviz()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
