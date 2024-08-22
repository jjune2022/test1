import pyrealsense2 as rs
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv2
from cv_bridge import CvBridge

def get_camera_info(frame, profile):
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    camera_info = CameraInfo()
    camera_info.header.stamp = rospy.Time.now()
    camera_info.header.frame_id = "camera_color_optical_frame"
    camera_info.height = intrinsics.height
    camera_info.width = intrinsics.width
    camera_info.distortion_model = "plumb_bob"  # or "equidistant" depending on your camera

    # D, K, R, P matrices
    camera_info.D = [intrinsics.coeffs[i] for i in range(5)]
    camera_info.K = [0] * 9
    camera_info.R = [0] * 9
    camera_info.P = [0] * 12

    camera_info.K[0] = intrinsics.fx
    camera_info.K[2] = intrinsics.ppx
    camera_info.K[4] = intrinsics.fy
    camera_info.K[5] = intrinsics.ppy
    camera_info.K[8] = 1

    camera_info.R[0] = camera_info.R[4] = camera_info.R[8] = 1

    camera_info.P[0] = intrinsics.fx
    camera_info.P[2] = intrinsics.ppx
    camera_info.P[5] = intrinsics.fy
    camera_info.P[6] = intrinsics.ppy
    camera_info.P[10] = 1

    return camera_info

def main():
    rospy.init_node('realsense_publisher')
    color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    aligned_depth_pub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=10)

    bridge = CvBridge()

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        rospy.loginfo("Realsense is now publishing!")
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to ROS Image messages
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")

            # Get camera info
            camera_info_msg = get_camera_info(color_frame, color_frame.profile)

            # Publish the images and camera info
            color_pub.publish(color_msg)
            aligned_depth_pub.publish(depth_msg)
            camera_info_pub.publish(camera_info_msg)

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass