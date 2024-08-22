import cv2
import numpy as np

frame_buffer = []
N = 15  # Number of frames to average

def process_frame(frame):
    # Convert a frame to grayscale
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

def temporal_blur(frames):
    # Apply temporal blurring by averaging frames
    return np.mean(np.array(frames), axis=0).astype(np.uint8)

def main(gray_frame):
    # gray_frame = process_frame(frame)

    # Add the processed frame to the buffer and ensure it contains only the last N frames
    frame_buffer.append(gray_frame)
    if len(frame_buffer) > N:
        frame_buffer.pop(0)
        
     # Apply temporal blurring when we have N frames in the buffer
    if len(frame_buffer) == N:
        blurred_frame = temporal_blur(frame_buffer)

        return blurred_frame

    return None




# cap = cv2.VideoCapture('ulsan.mp4')
# i=0
# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     gray_frame = process_frame(frame)

#     # Add the processed frame to the buffer and ensure it contains only the last N frames
#     frame_buffer.append(gray_frame)
#     if len(frame_buffer) > N:
#         frame_buffer.pop(0)

#     # Apply temporal blurring when we have N frames in the buffer
#     if len(frame_buffer) == N:
#         blurred_frame = temporal_blur(frame_buffer)
#         cv2.imshow('Temporally Blurred Frame', blurred_frame)
#         cv2.imwrite("try/left_right_lane_bspline/avg_input/"+str(i)+".jpg", blurred_frame)
#     i+=1
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
