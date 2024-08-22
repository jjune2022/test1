import rospy
from std_msgs.msg import UInt8, Int32
from visualization_msgs.msg import Marker, MarkerArray

class VisualizeValues:
    def __init__(self):
        rospy.init_node('visualize_values')

        # Subscribers
        rospy.Subscriber('/state_speed', UInt8, self.speed_callback)
        rospy.Subscriber('/state_steer', Int32, self.steer_callback)
        rospy.Subscriber('/state_brake', UInt8, self.brake_callback)
        rospy.Subscriber('/state_encoder', Int32, self.encoder_callback)

        # Publisher for markers
        self.marker_pub = rospy.Publisher('/visualization_values', MarkerArray, queue_size=10)

        # Initialize values
        self.speed = 0
        self.steer = 0
        self.brake = 0
        self.encoder = 0

        rospy.spin()

    def create_marker(self, id, ns, text, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.text = text
        return marker

    def update_markers(self):
        marker_array = MarkerArray()
        marker_array.markers.append(self.create_marker(0, "speed", f"Speed: {self.speed}", [5, 6, 0]))
        marker_array.markers.append(self.create_marker(1, "steer", f"Steer: {self.steer}", [5, 5.5, 0]))
        marker_array.markers.append(self.create_marker(2, "brake", f"Brake: {self.brake}", [5, 5, 0]))
        marker_array.markers.append(self.create_marker(3, "encoder", f"Encoder: {self.encoder}", [5, 4.5, 0]))
        self.marker_pub.publish(marker_array)

    def speed_callback(self, msg):
        self.speed = msg.data
        self.update_markers()

    def steer_callback(self, msg):
        self.steer = msg.data
        self.update_markers()

    def brake_callback(self, msg):
        self.brake = msg.data
        self.update_markers()

    def encoder_callback(self, msg):
        self.encoder = msg.data
        self.update_markers()

if __name__ == '__main__':
    try:
        VisualizeValues()
    except rospy.ROSInterruptException:
        pass
