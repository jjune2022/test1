import math
import numpy as np

import cv2

from cfg import config

def world_to_img(img):   
    pts = np.array([
        [config.world_x_max, config.world_y_max, 0, 1],  
        [config.world_x_max, config.world_y_min, 0, 1],
        [config.world_x_min, config.world_y_min, 0, 1],
        [config.world_x_min, config.world_y_max, 0, 1],
    ], dtype=np.float32)

    # camera x, y, z = intrinsic @ extrinsic inverse @ world x, y, z, 1
    cam_pts = np.linalg.inv(config.extrinsic) @ pts.T   # 4 X 4
    img_pts = config.intrinsic @ cam_pts[:3]        

    # projection
    x, y = img_pts[:2] / img_pts[2]

    line_color = (0, 0, 255)
    line_thickness = 2
    
    # vis
    # for i in range(len(x)):
    #     cv2.circle(img, (int(x[i]), int(y[i])), 4, (0, 0, 255), -1)
    #     cv2.line(img, (int(x[i])-500, int(y[i])), (int(x[i])+500, int(y[i])), line_color, line_thickness)
    #     cv2.line(img, (int(x[i]), int(y[i])-500), (int(x[i]), int(y[i])+500), line_color, line_thickness)

    # cv2.imshow("img", img)
    # cv2.waitKey(10)

    return np.vstack((x, y))

    '''
    for i in range(pts.shape[0]):
        cam_pts = np.linalg.inv(config.extrinsic) @ pts[i]
        # cam_pts = config.extrinsic @ pts[i]
        img_pts_hom = intrinsic @ cam_pts[:3]
        x,y = img_pts_hom[:2] / img_pts_hom[2]
        img_pts.append((x, y))
        cv2.circle(img, (int(x), int(y)), 4, (0, 0, 255), -1)
        # cv2.imshow("img", img)

    
    return img_pts
    '''