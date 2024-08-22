# import rospy
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, Quaternion
# from cv_bridge import CvBridge
# import cv2

# class BroadcastRviz:
#     def __init__(self):
#         self.rgb_raw_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
#         self.image_pub = rospy.Publisher('/camera/color/image_with_markers', Image, queue_size=10)
#         self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
#         rospy.Timer(rospy.Duration(1.0), self.marker_callback)

#     def image_callback(self, msg):
#         bridge = CvBridge()
#         cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # Example points (replace with your own points)
#         points = [(100, 100), (200, 200), (300, 300)]
#         line_start = (50, 50)
#         line_end = (400, 400)

#         for point in points:
#             cv2.circle(cv_image, point, 5, (0, 255, 0), -1)  # Draw points

#         cv2.line(cv_image, line_start, line_end, (0, 255, 0), 2)  # Draw line

#         self.image_pub.publish(bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

#     def marker_callback(self, event):
#         # Create and publish markers for points and lines
#         line_marker = Marker()
#         line_marker.header.frame_id = "camera_link"  # Ensure this frame_id exists in your tf
#         line_marker.header.stamp = rospy.Time.now()
#         line_marker.ns = "points_and_lines"
#         line_marker.id = 0
#         line_marker.type = Marker.LINE_LIST
#         line_marker.action = Marker.ADD
#         line_marker.pose.orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
#         line_marker.scale.x = 0.1  # Line width
#         line_marker.color.r = 1.0
#         line_marker.color.g = 0.0
#         line_marker.color.b = 0.0
#         line_marker.color.a = 1.0
#         line_marker.lifetime = rospy.Duration(5)  # Marker will disappear after 5 seconds

#         # Define points and lines
#         line_marker.points = []

#         # Example lines (replace with your own lines)
#         line_points = [
#             (0.5, 0.5, 0), (4, 4, 0),  # Line 1
#             (1, 1, 0), (2, 2, 0)       # Line 2
#         ]

#         for p in line_points:
#             pt = Point()
#             pt.x = p[0]
#             pt.y = p[1]
#             pt.z = p[2]
#             line_marker.points.append(pt)

#         self.marker_pub.publish(line_marker)

#         # Create and publish a text marker
#         text_marker = Marker()
#         text_marker.header.frame_id = "camera_link"
#         text_marker.header.stamp = rospy.Time.now()
#         text_marker.ns = "text"
#         text_marker.id = 1
#         text_marker.type = Marker.TEXT_VIEW_FACING
#         text_marker.action = Marker.ADD
#         text_marker.pose.position.x = 3.0  # Set this to the desired position
#         text_marker.pose.position.y = 2.0  # Set this to the desired position
#         text_marker.pose.position.z = 1.0  # Set this to the desired position
#         text_marker.pose.orientation = Quaternion(0, 0, 0, 1)
#         text_marker.scale.z = 0.5  # Text height
#         text_marker.color.r = 1.0
#         text_marker.color.g = 1.0
#         text_marker.color.b = 1.0
#         text_marker.color.a = 1.0
#         text_marker.text = "Hello, RViz!"
#         text_marker.lifetime = rospy.Duration(0.5)  # Text will disappear after 5 seconds

#         self.marker_pub.publish(text_marker)

# if __name__ == '__main__':
#     rospy.init_node('image_marker_node')
#     node = BroadcastRviz()
#     rate = rospy.Rate(1)
#     while not rospy.is_shutdown():
#         rate.sleep()


# import rospy
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, Quaternion
# import tf

# class VehicleVisualization:
#     def __init__(self):
#         rospy.init_node('vehicle_visualization_node', anonymous=False)
#         self.rate = rospy.Rate(10)

#         # Publisher for the vehicle model and lines
#         self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        
#         # Set up initial values
#         self.setup_vehicle_model()
#         self.setup_lines()

#     def setup_vehicle_model(self):
#         # Create a Marker for the vehicle model
#         self.vehicle_marker = Marker()
#         self.vehicle_marker.header.frame_id = "base_link"
#         self.vehicle_marker.ns = "vehicle"
#         self.vehicle_marker.id = 0
#         self.vehicle_marker.type = Marker.CUBE
#         self.vehicle_marker.action = Marker.ADD
#         self.vehicle_marker.pose.position.x = 0.0
#         self.vehicle_marker.pose.position.y = 0.0
#         self.vehicle_marker.pose.position.z = 0.5  # Set z to 0.5 to lift it above the ground
#         self.vehicle_marker.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
#         self.vehicle_marker.scale.x = 1.0  # Set vehicle size
#         self.vehicle_marker.scale.y = 0.5
#         self.vehicle_marker.scale.z = 0.5
#         self.vehicle_marker.color.r = 1.0
#         self.vehicle_marker.color.g = 0.0
#         self.vehicle_marker.color.b = 0.0
#         self.vehicle_marker.color.a = 1.0

#     def setup_lines(self):
#         # Create a Marker for the lines
#         self.line_marker = Marker()
#         self.line_marker.header.frame_id = "base_link"
#         self.line_marker.ns = "lines"
#         self.line_marker.id = 1
#         self.line_marker.type = Marker.LINE_LIST
#         self.line_marker.action = Marker.ADD
#         self.line_marker.pose.orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
#         self.line_marker.scale.x = 0.1  # Line width
#         self.line_marker.color.r = 0.0
#         self.line_marker.color.g = 1.0
#         self.line_marker.color.b = 0.0
#         self.line_marker.color.a = 1.0

#         # Initial line points
#         self.update_line_points([(1.0, 0.5, 0.5), (10.0, 0.5, 0.5), (1.0, -0.5, 0.5), (10.0, -0.5, 0.5)])

#     def update_line_points(self, points):
#         self.line_marker.points = []
#         for p in points:
#             point = Point()
#             point.x = p[0]
#             point.y = p[1]
#             point.z = p[2]
#             self.line_marker.points.append(point)

#     def run(self):
#         while not rospy.is_shutdown():
#             # Publish the vehicle model marker
#             self.vehicle_marker.header.stamp = rospy.Time.now()
#             self.marker_pub.publish(self.vehicle_marker)

#             # Publish the line marker
#             self.line_marker.header.stamp = rospy.Time.now()
#             self.marker_pub.publish(self.line_marker)

#             self.rate.sleep()

# if __name__ == '__main__':
#     try:
#         vehicle_visualization = VehicleVisualization()
#         vehicle_visualization.run()
#     except rospy.ROSInterruptException:
#         pass

# 점 하나로 보여주기
# import rospy
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, Quaternion
# import tf
# import numpy as np

# class VehicleVisualization:
#     def __init__(self):
#         rospy.init_node('vehicle_visualization_node', anonymous=False)
#         self.rate = rospy.Rate(10)

#         # Publisher for the vehicle model and lines
#         self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        
#         # Set up initial values
#         self.setup_vehicle_model()
#         self.setup_lines()

#     def setup_vehicle_model(self):
#         # Create a Marker for the vehicle model
#         self.vehicle_marker = Marker()
#         self.vehicle_marker.header.frame_id = "base_link"
#         self.vehicle_marker.ns = "vehicle"
#         self.vehicle_marker.id = 0
#         self.vehicle_marker.type = Marker.CUBE
#         self.vehicle_marker.action = Marker.ADD
#         self.vehicle_marker.pose.position.x = 0.0
#         self.vehicle_marker.pose.position.y = 0.0
#         self.vehicle_marker.pose.position.z = 0.5  # Set z to 0.5 to lift it above the ground
#         self.vehicle_marker.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
#         self.vehicle_marker.scale.x = 1.0
#         self.vehicle_marker.scale.y = 0.5
#         self.vehicle_marker.scale.z = 0.5
#         self.vehicle_marker.color.r = 1.0
#         self.vehicle_marker.color.g = 0.0
#         self.vehicle_marker.color.b = 0.0
#         self.vehicle_marker.color.a = 1.0

#     def setup_lines(self):
#         # Create markers for the lines
#         self.line_marker1 = Marker()
#         self.line_marker1.header.frame_id = "base_link"
#         self.line_marker1.ns = "lines"
#         self.line_marker1.id = 1
#         self.line_marker1.type = Marker.LINE_STRIP
#         self.line_marker1.action = Marker.ADD
#         self.line_marker1.pose.orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
#         self.line_marker1.scale.x = 0.1  # Line width
#         self.line_marker1.color.r = 0.0
#         self.line_marker1.color.g = 1.0
#         self.line_marker1.color.b = 0.0
#         self.line_marker1.color.a = 1.0

#         self.line_marker2 = Marker()
#         self.line_marker2.header.frame_id = "base_link"
#         self.line_marker2.ns = "lines"
#         self.line_marker2.id = 2
#         self.line_marker2.type = Marker.LINE_STRIP
#         self.line_marker2.action = Marker.ADD
#         self.line_marker2.pose.orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
#         self.line_marker2.scale.x = 0.1  # Line width
#         self.line_marker2.color.r = 1.0
#         self.line_marker2.color.g = 0.0
#         self.line_marker2.color.b = 0.0
#         self.line_marker2.color.a = 1.0

#         # Initial line points
#         points1 = [
#     [18, 0], [18, 5], [18, 11], [18, 17], [18, 23], [18, 29], [18, 35], [19, 41], [19, 47], [19, 53], 
#     [19, 59], [19, 66], [19, 72], [20, 78], [20, 84], [20, 91], [20, 97], [20, 104], [20, 110], [20, 117], 
#     [20, 123], [21, 130], [21, 137], [21, 143], [21, 150], [21, 157], [21, 164], [21, 171], [21, 178], [21, 185], 
#     [22, 192], [22, 199], [22, 206], [22, 213], [22, 220], [22, 227], [22, 235], [22, 242], [22, 249], [22, 257], 
#     [22, 264], [23, 272], [23, 279], [23, 287], [23, 294], [23, 302], [23, 310], [23, 317], [23, 325], [23, 333], 
#     [23, 341], [23, 349], [23, 357], [23, 365], [23, 373], [23, 381], [23, 389], [23, 397], [23, 405], [23, 414], 
#     [23, 422], [23, 430], [23, 439], [23, 447], [23, 456], [23, 464], [23, 473], [23, 481], [23, 490], [24, 499]
# ]
#         points2 = [
#     [109, 0], [112, 5], [116, 11], [120, 17], [124, 23], [128, 29], [131, 35], [135, 41], [138, 47], [142, 53], 
#     [145, 59], [148, 66], [151, 72], [154, 78], [157, 84], [160, 91], [163, 97], [166, 104], [169, 110], [171, 117], 
#     [174, 123], [176, 130], [179, 137], [181, 143], [183, 150], [185, 157], [188, 164], [190, 171], [192, 178], 
#     [193, 185], [195, 192], [197, 199], [199, 206], [200, 213], [202, 220], [203, 227], [205, 235], [206, 242], 
#     [207, 249], [208, 257], [209, 264], [210, 272], [211, 279], [212, 287], [213, 294], [214, 302], [214, 310], 
#     [215, 317], [215, 325], [216, 333], [216, 341], [216, 349], [217, 357], [217, 365], [217, 373], [217, 381], 
#     [217, 389], [216, 397], [216, 405], [216, 414], [215, 422], [215, 430], [214, 439], [214, 447], [213, 456], 
#     [212, 464], [211, 473], [211, 481], [210, 490], [209, 499]
# ]

#         self.update_line_points(self.line_marker1, np.array(points1)/100)
#         self.update_line_points(self.line_marker2, np.array(points2)/100)

#     def update_line_points(self, marker, points):
#         marker.points = []
#         for p in points:
#             point = Point()
#             point.x = p[0]
#             point.y = p[1]
#             point.z = 0  # Set z to 0 for 2D lines
#             marker.points.append(point)

#     def run(self):
#         while not rospy.is_shutdown():
#             # Publish the vehicle model marker
#             self.vehicle_marker.header.stamp = rospy.Time.now()
#             self.marker_pub.publish(self.vehicle_marker)

#             # Publish the line markers
#             self.line_marker1.header.stamp = rospy.Time.now()
#             self.marker_pub.publish(self.line_marker1)

#             self.line_marker2.header.stamp = rospy.Time.now()
#             self.marker_pub.publish(self.line_marker2)

#             self.rate.sleep()

# if __name__ == '__main__':
#     try:
#         vehicle_visualization = VehicleVisualization()
#         vehicle_visualization.run()
#     except rospy.ROSInterruptException:
#         pass


# 토픽 구독
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Polygon
import tf.transformations
import numpy as np

class VehicleVisualization:
    def __init__(self):
        rospy.init_node('vehicle_visualization_node', anonymous=False)
        self.rate = rospy.Rate(10)

        # Publisher for the vehicle model and lines
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        
        # Subscriber for the points data
        rospy.Subscriber('/points_topic', Polygon, self.points_callback)

        # Set up initial values
        self.setup_vehicle_model()
        self.setup_lines()

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
        self.vehicle_marker.pose.position.y = 0.5
        self.vehicle_marker.pose.position.z = self.vehicle_marker.scale.z / 2

    def setup_lines(self):
        # Create markers for the lines
        self.line_marker1 = Marker()
        self.line_marker1.header.frame_id = "map"
        self.line_marker1.ns = "lines"
        self.line_marker1.id = 1
        self.line_marker1.type = Marker.LINE_STRIP
        self.line_marker1.action = Marker.ADD
        q = tf.transformations.quaternion_from_euler(0, np.pi, np.pi)
        self.line_marker1.pose.orientation = Quaternion(*q)  # Identity quaternion
        self.line_marker1.scale.x = 0.1  # Line width
        self.line_marker1.color.r = 0.0
        self.line_marker1.color.g = 1.0
        self.line_marker1.color.b = 0.0
        self.line_marker1.color.a = 1.0
        self.vehicle_marker.pose.position.x = 1.4
        self.vehicle_marker.pose.position.y = -6.0

        self.line_marker2 = Marker()
        self.line_marker2.header.frame_id = "map"
        self.line_marker2.ns = "lines"
        self.line_marker2.id = 2
        self.line_marker2.type = Marker.LINE_STRIP
        self.line_marker2.action = Marker.ADD
        q = tf.transformations.quaternion_from_euler(0, np.pi, np.pi)
        self.line_marker2.pose.orientation = Quaternion(*q)  # Identity quaternion
        self.line_marker2.scale.x = 0.1  # Line width
        self.line_marker2.color.r = 1.0
        self.line_marker2.color.g = 0.0
        self.line_marker2.color.b = 0.0
        self.line_marker2.color.a = 1.0
        self.vehicle_marker.pose.position.y = -6.0

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
            point.z = 0  # Set z to 0 for 2D lines
            if i < half:
                self.line_marker1.points.append(point)
            else:
                self.line_marker2.points.append(point)

    def run(self):
        while not rospy.is_shutdown():
            # Publish the vehicle model marker
            self.vehicle_marker.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.vehicle_marker)

            # Publish the line markers
            self.line_marker1.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.line_marker1)

            self.line_marker2.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.line_marker2)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        vehicle_visualization = VehicleVisualization()
        vehicle_visualization.run()
    except rospy.ROSInterruptException:
        pass
