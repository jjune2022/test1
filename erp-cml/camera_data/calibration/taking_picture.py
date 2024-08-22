#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # 이미지 저장
        cv2.imwrite('image.png', cv_image)
        # 노드 종료
        rospy.signal_shutdown("Image saved.")

def main():
    rospy.init_node('image_saver', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
