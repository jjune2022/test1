import numpy as np

A = np.eye(6)
H = np.eye(6)
Q = np.diag(np.diag(10**-4 * np.ones(6)))
    
    
# def kalman_filter(x, z_meas, P):
#     # (1) Prediction.
#     x_pred = A @ x_esti
#     P_pred = A @ P @ A.T + Q
        
#     if not all(len(sublist) == 0 for sublist in z_meas):
                    
#         # (2) Kalman Gain.
#         K = P_pred @ H @ np.linalg.inv(H @ P_pred @ H.T + R)

#         # (3) Estimation.
#         x_esti = x_pred + K @ (z_meas - H @ x_pred)

#         # (4) Error Covariance.
#         P = P_pred - K @ H @ P_pred
#         # print(P)
                
#         return x_esti, P

#     return x_pred, P_pred
    
def kalman_filter(x, z, P, R, step):
    # (1) Prediction - time update
    x_ = A @ x
    P_ = A @ P @ A.T + Q
    #- print("@@Prediction@@", x_)

    # if z == []:
    #     print(111)
    #     R = np.diag(1000 * np.ones(3))
    #     z = np.zeros((3,1))
    
    
    
    # (2) Correction - measurement update
    K = P_ @ H @ np.linalg.inv(H @ P_ @ H.T + R)
    # print(K)
    # (3) Estimation.
    x_esti = x_ + K @ (z - H @ x_)
    # (4) Error Covariance.
    P = P_ - K @ H @ P_
    
    # print(z)
    # print(x_)
    # print(x_esti)
    
    return x_esti, P_



    # # (2) Correction - measurement update
    # # kalman gain
    # K = P_ @ H.T @ np.linalg.inv(H @ P_ @ H.T + R)
    # # print(K)
    # x_esti = (np.eye(3) - K @ H) @ x_ + K @ H @ z 
    # # print(z)
    # # print(x_)
    # # print(x_esti)
    
    # return x_esti, (np.eye(3) - K @ H) @ P_