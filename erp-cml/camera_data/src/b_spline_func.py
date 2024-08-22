import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
import cv2


def bspline(Q, img, color):
    """ 
    Originally, although Q contains more one empty list, it only draws existing lines.
    But, somewhat it brings error if Q has empty list.
    Taking only the existing list solves this issue.
    """    
    Q_filtered = [sublist for sublist in Q if sublist]
    if not Q_filtered:
        return img, []

    ctr = np.array(Q_filtered)

    x = []
    y = []
    for i in range(len(ctr)):
        if len(ctr[i]) > 0:
            x.append(ctr[i][0])
            y.append(ctr[i][1])
    
    if len(x) < 2:
        print("Not enough points to form a spline.")
        return img, []
    
    l=len(x)
    Order = l-1
    
    t=np.linspace(0,1,l-(Order-1),endpoint=True)
    t=np.append(np.zeros(Order),t)
    t=np.append(t,np.zeros(Order)+1)
    
    tck=[t,[x,y],Order]
    u3=np.linspace(0,1,(max(l*2,70)),endpoint=True)
    out = interpolate.splev(u3,tck)
    
    # plt.plot(x,y,'k--',label='Control polygon',marker='o',markerfacecolor='red')
    # plt.plot(out[0],out[1],'b',linewidth=2.0,label='B-spline curve')
    # plt.legend(loc='best')
    # plt.axis([0, 1280, 720, 0])  # Set plot limits to image size
    # plt.title('Cubic B-spline curve evaluation in image coordinate system')
    # plt.show()
    
    for x_,y_ in zip(x,y):
        cv2.circle(img, (x_,y_), 7, color, 1)
    points = np.int32(np.column_stack(out))
    cv2.polylines(img, [points], False, color, 1)

    return img, points
            
    
# img = np.zeros((500, 300, 3), dtype=np.uint8)
# Q = [[168, 0], [143, 100], [51, 500]]
# img, pts =bspline(Q, img, (0,255,0))
# cv2.imshow("ASD", img)
# cv2.waitKey(10000)