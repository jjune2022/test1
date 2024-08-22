import cv2
import numpy as np


file_name = 'image.png'
# mtx = np.array([[337.5550842285156,                   0, 322.2391662597656],
#                 [                0,  337.5550842285156, 179.506591796875],
#                 [                0,                   0,                 1]])  # Zed left camera & 640x360 image
# mtx = np.array([[ 908.9029541015625,                0.0,  639.3457641601562],
#                 [               0.0,   908.635009765625, 360.27484130859375],
#                 [               0.0,                0.0,                1.0]])  # realsense camera & 1280x720 image
mtx = np.array([[ 454.18850708,                0.0,  322.35043335],
                [               0.0,   453.9779052, 179.39408875],
                [               0.0,                0.0,                1.0]])
dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

T_vb = np.array([[ 0, 0, 1, 155.0],
                 [ 0,-1, 0,  30.0],
                 [ 1, 0, 0,  5.0],
                 [ 0, 0, 0,   1.0]])


CHECKERBOARD = (7,9) # 체커보드 행과 열당 내부 코너 수
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
# 3D 점의 세계 좌표 정의
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * 5  # 체커보드의 각 사각형 길이
objpoints = []
imgpoints = [] 
axis = np.float32([[10,0,0], [0,10,0], [0,0,5]]).reshape(-1,3)


def draw(img, corners, imgpts):  # bgr -> rgb 순서로 바꿈
    c = corners[0].ravel()
    corner = (int(c[0]), int(c[1]))
    cv2.line(img, corner, (int(imgpts[0][0][0]), int(imgpts[0][0][1])), (0,0,255), 2)
    cv2.line(img, corner, (int(imgpts[1][0][0]), int(imgpts[1][0][1])), (0,255,0), 2)
    cv2.line(img, corner, (int(imgpts[2][0][0]), int(imgpts[2][0][1])), (255,0,0), 2)
    return img


img = cv2.imread(file_name)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# 이미지에서 원하는 개수의 코너가 발견되면 ret = true
ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
# 원하는 개수의 코너가 감지되면 픽셀 좌표 미세조정 -> 체커보드 이미지 표시
if ret == True:
    objpoints.append(objp)
    # 주어진 2D 점에 대한 픽셀 좌표 미세조정
    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    imgpoints.append(corners2)

    img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    _, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
    
    # world axis project to image plane
    imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
    img = draw(img, corners2, imgpts)

    cv2.imshow('img',img)
    cv2.imwrite("modified_image.png", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# print("rvecs : \n", rvecs) # rotation
# print("tvecs : \n", tvecs) # translation
rot_matrix, _ = cv2.Rodrigues(rvecs)
T_cb = np.vstack([np.hstack([rot_matrix, tvecs]), [0,0,0,1]])
T_vc = T_vb @ np.linalg.inv(T_cb)
print("T_vehicle-cam: \n", T_vc)
# print("T_vehicle-cam: \n", np.linalg.inv(T_vc))



# # *===============================Check (cv2.projectPoints & Naive approach)===============================
# print()
# world_point = np.array([[0, 0, 0]], dtype=np.float32)  # Just a 3D point, no need for homogeneous coordinate
# image_points_distorted, _ = cv2.projectPoints(world_point, rvecs, tvecs, mtx, dist)
# print("cv2 projection:", image_points_distorted)
# cv2.circle(img, (int(image_points_distorted[0][0][0]), int(image_points_distorted[0][0][1])), 10, (0, 255, 0), 1)

# #! Drawing axis
# axis = np.float32([[1,0,0], [0,1,0], [0,0,1]]).reshape(-1,3)
# imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
# cv2.line(img, (int(image_points_distorted[0][0][0]), int(image_points_distorted[0][0][1])), (int(imgpts[0][0][0]), int(imgpts[0][0][1])), (255,0,0), 2)
# cv2.line(img, (int(image_points_distorted[0][0][0]), int(image_points_distorted[0][0][1])), (int(imgpts[1][0][0]), int(imgpts[1][0][1])), (0,255,0), 2)
# cv2.line(img, (int(image_points_distorted[0][0][0]), int(image_points_distorted[0][0][1])), (int(imgpts[2][0][0]), int(imgpts[2][0][1])), (0,0,255), 2)

# #! Naive projection of world axis
# world_point = np.array([[0, 0, 0, 1]], dtype=np.float32) 
# camera_point = T_cb @ world_point.T  # 3x4 @ 4x1 = 3x1
# image_point = mtx @ camera_point  # 3x3 @ 3x1 = 3x1
# image_point_cartesian = (image_point[:2] / image_point[2]).reshape(1,2)

# print("Naive:", image_point_cartesian)
# #! Checked that cv2.projectPoints and naive approach yield the same results.
# imgpts, _ = cv2.projectPoints(np.float32([[-10,0,0]]), rvecs, tvecs, mtx, dist)
# # cv2.circle(img, (int(imgpts[0][0][0]), int(imgpts[0][0][1])), 5, (255, 0, 0), -1)
# # print(int(imgpts[0][0][0]), int(imgpts[0][0][1]))

# cv2.imshow('b',img)
# cv2.imwrite("b.png", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()