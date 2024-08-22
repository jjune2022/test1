import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
import time
import copy

import rospy
from sensor_msgs.msg import Image, CameraInfo
from camera_data.msg import ReferencePoses
import cv2
from cv_bridge import CvBridge, CvBridgeError

from b_spline_func import bspline
from kalman_filter_3rd import kalman_filter
from avg_input import *
from funcs import *
import config
import pyrealsense2 as rs


class PosePublisher:
    def __init__(self):
        self.pos_pub = rospy.Publisher("/ref_pos", ReferencePoses, queue_size=10)
        self.image_pub = rospy.Publisher('/camera/color/image_with_markers', Image, queue_size=10)
        self.rgb_info = None
        self.depth_info = None
        
        self.x_esti = None
        self.prev_P = None
        self.no_line_cnt = [0,0,0,0]
        self.prev_Q_l = None
        self.prev_Q_r = None
        self.P_0 = 100*np.eye(6)
        self.prev_esti = None
        self.frame_count = 0
        self.start_time = time.time()
        self.final = None
        self.depth_image = None

        self.rgb_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback, "rgb")
        self.rgb_raw_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_raw_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        # self.depth_raw_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

        self.bridge = CvBridge()
        # Declare filters
        self.decimation = rs.decimation_filter()
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        
    def camera_info_callback(self, data, camera_type):
        if camera_type == "rgb" and not self.rgb_info:
            self.rgb_info = data
            self.rgb_info_sub.unregister()  # Unsubscribe after receiving the info
    

    def depth_callback(self, msg):
        self.depth_image = msg
        # print(type(self.depth_image))
        # rospy.loginfo("Depth image has been updated.")


    def image_callback(self, rgb_image):
        # Load RGB and Depth image
        bridge = CvBridge()
        try:
            cv_rgb = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            cv_depth = bridge.imgmsg_to_cv2(self.depth_image, "16UC1")
        except CvBridgeError as e:
            print(e)
            return
        #--------------------------------
        rgb_intrinsic = np.array(self.rgb_info.K).reshape(3, 3)
        bev_pts = world_to_img_pts(cv_rgb, rgb_intrinsic)
        
        
        if self.lane_det_main(cv_rgb, bev_pts) == None:
            return
        else:
            final_Q_l, final_Q_r, bspline_est_left_pts, bspline_est_right_pts, inv_matrix = self.lane_det_main(cv_rgb, bev_pts)

        #! bev img pts -> img -> world -> mid point
        #! 1. bev pts -> img pts   (mid lane)
        final_pt = (bspline_est_left_pts + bspline_est_right_pts)/2  # mid line
        bevpts = np.array(final_pt, dtype=np.float32)
        if len(bevpts.shape) == 2:
            bevpts = bevpts[:, np.newaxis, :]  # Add the required dimension

        img_pts = cv2.perspectiveTransform(bevpts, inv_matrix).astype(int)
        img_pts = img_pts[:, 0, :].astype(int)
        indices = np.linspace(0, len(img_pts) - 1, 10, dtype=int)
        sampled_points = img_pts[-10:]
        # print(sampled_points)

        refpose = ReferencePoses()
        refpose.header.stamp = rospy.Time.now()
        refpose.header.frame_id = "base_link"
        world_coords_list = []
        
        #! midpoint vis
        cam_coords = np.array([cv_depth[j, i] * np.linalg.inv(rgb_intrinsic) @ np.array([i, j, 1]) for i, j in sampled_points])
        for idx, (i, j) in enumerate(sampled_points):
            # print("i,j:",i,j)
            region = cv_depth[j-1:j+2, i-1:i+2]
            depth_value = np.mean(region) * 0.1

            print("depth", depth_value)
            f = (rgb_intrinsic[0, 0] + rgb_intrinsic[1, 1]) / 2
            theta = np.arctan(np.sqrt((i - rgb_intrinsic[0, 2])**2 + (j - rgb_intrinsic[1, 2])**2) / f)
            Z_c = depth_value * np.cos(theta)
            normalized_coord = np.linalg.inv(rgb_intrinsic) @ np.array([i, j, 1])
            cam_coords = Z_c * normalized_coord

            world_coords = config.extrinsic @ np.append(cam_coords, 1)
            world_target_point = world_coords[:3]
            world_coords_list.append(world_target_point)
            refpose.points[9-idx].x = world_target_point[0]
            refpose.points[9-idx].y = world_target_point[1]
            refpose.points[9-idx].z = world_target_point[2]
            cv2.circle(self.final, (i, j), 3, (0, 255, 0), -1)  # 시각화

        self.pos_pub.publish(refpose)
        self.image_pub.publish(bridge.cv2_to_imgmsg(self.final, encoding="bgr8"))
        cv2.imshow('Final', self.final)
        
        cv2.waitKey(1)
        


    def lane_det_main(self, raw_img, bev_pts):
        # cv2.imshow('raw_img', raw_img)
        # print(f"--------{config.q}--------")
        gray = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
        bev, inv_matrix = BEV(gray, bev_pts)
        cv2.imshow('bev', bev)
        canny_dilate = preprocessing_newnew(bev)
        bev = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
        cv2.waitKey(1)

        if config.initial_not_found:
            # print("@@@@@@@@@ FINDING INITIAL LANE @@@@@@@@@")
            lines_in_section, lines_in_section_img, Q_l, Q_r = extract_lines_in_section_initial(canny_dilate, self.no_line_cnt)
            # R = np.zeros((6, 1))
        else:
            # print("@@@@@@@@@ INITIAL LANE FOUND @@@@@@@@@")
            lines_in_section, lines_in_section_img, Q_l, Q_r = extract_lines_in_section(canny_dilate, self.prev_Q_l, self.prev_Q_r, self.no_line_cnt)
            R = R_set_considering_control_points(Q_l, Q_r, self.prev_esti, self.no_line_cnt)
        
        no_line_cnt_update(self.no_line_cnt, lines_in_section)
        if config.initial_not_found: #! If initial lane not found, skip everything below
            print(f"Initial lane not found: {self.x_esti}")
            return None
        
        # Prepare KF state vectors
        first_elements = []
        for inner_list in Q_l:
            if len(inner_list) > 0:
                first_elements.append(inner_list[0])
            else:
                first_elements.append(0)
        KF_Q_l = np.array(first_elements)
        
        first_elements = []
        for inner_list in Q_r:
            if len(inner_list) > 0:
                first_elements.append(inner_list[0])
            else:
                first_elements.append(0)
        KF_Q_r = np.array(first_elements)
        
        if KF_Q_l is not None and KF_Q_r is not None:
            z_meas = np.concatenate((KF_Q_l, KF_Q_r), axis=0)
            z_meas = np.expand_dims(z_meas, axis=1)
        else:
            z_meas = []
        
        if self.prev_Q_l == None: # Initial
            self.x_esti = z_meas.copy()
            self.P = self.P_0
            self.prev_P = self.P_0
            R = np.diag(1000 * np.ones(6))
        
        # print("R:", R)
        # print(self.x_esti)
        # print(z_meas)
        
        self.x_esti, self.P = kalman_filter(self.x_esti, z_meas, self.P, R, config.q) #!!!
        self.prev_esti = copy.deepcopy(self.x_esti)
        # print("@@Updated@", self.x_esti)
        new_Q_l = self.x_esti[0:len(config.section_list)].tolist()
        new_Q_r = self.x_esti[len(config.section_list):len(config.section_list)*2].tolist()
        new_Q_l = [[int(new_Q_l[i][j]) for j in range(len(new_Q_l[i]))] for i in range(len(new_Q_l))]
        new_Q_r = [[int(new_Q_r[i][j]) for j in range(len(new_Q_r[i]))] for i in range(len(new_Q_r))]
        for i in range(len(new_Q_l)):
            new_Q_l[i].append(config.section_list[i])
            new_Q_r[i].append(config.section_list[i])
            cv2.circle(lines_in_section_img, (new_Q_l[i][0], new_Q_l[i][1]), 7, (255,255,255), 2)
            cv2.circle(lines_in_section_img, (new_Q_r[i][0], new_Q_r[i][1]), 7, (255,255,255), 2)
        
        cv2.imshow("KF", lines_in_section_img)
        # cv2.imwrite("vis/"+str(config.q)+".png", lines_in_section_img)

        
        img = np.zeros((500, 300, 3), dtype=np.uint8) # B-spline img
        bspline_img, bspline_est_left_pts = bspline(new_Q_l, bev, (0, 255, 0)) # estimation
        bspline_img, bspline_est_right_pts = bspline(new_Q_r, bspline_img, (0, 255, 0)) # estimation
        bspline_img, bspline_meas_left_pts = bspline(Q_l, bspline_img, (0, 0, 255)) # measurement
        bspline_img, bspline_meas_right_pts = bspline(Q_r, bspline_img, (0, 0, 255)) # measurement
        
        # For merging two images
        img, bspline_left_pts = bspline(new_Q_l, img, (0, 255, 0)) # estimation
        img, bspline_right_pts = bspline(new_Q_r, img, (0, 255, 0)) # estimation

        self.prev_Q_l = copy.deepcopy(new_Q_l)
        self.prev_Q_r = copy.deepcopy(new_Q_r)
        self.prev_P = copy.deepcopy(self.P)
        
        # cv2.imshow("B-spline on BEV", bspline_img)
        # cv2.imwrite("vis/"+str(config.q)+".jpg", bspline_img)

        if img is not None:
            newwarp1 = cv2.warpPerspective(img, inv_matrix, (raw_img.shape[1], raw_img.shape[0]))
            self.final = cv2.addWeighted(raw_img, 1, newwarp1, 1, 0) # original 이미지에 복구한 이미지 합성
            
            
            # cv2.imshow('B-spline', img)
            # cv2.imshow('Final', self.final)
            # cv2.imwrite("visualizations/unist_final/"+str(config.q)+".jpg", final)
        
        
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            end_time = time.time()
            fps = self.frame_count / (end_time - self.start_time)
            # print("Processed {0} frames in {1:.2f} seconds, approx FPS: {2:.2f}".format(self.frame_count, end_time - self.start_time, fps))
            self.frame_count = 0
            self.start_time = time.time()
            # cv2.waitKey(1000)
        cv2.waitKey(1)

        config.q += 1 #* for drawing
        # print("--------")

        return new_Q_l, new_Q_r, bspline_est_left_pts, bspline_est_right_pts, inv_matrix   




if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    node = PosePublisher()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()