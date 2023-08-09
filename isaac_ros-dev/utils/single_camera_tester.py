import cv2
import numpy as np

cap1 = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2) # Back
#cap3 = cv2.VideoCapture("/dev/video3", cv2.CAP_V4L2) # Right

# FRAME_WIDTH = 640
# FRAME_HEIGHT = 480

# cap1.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
# cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

while True:
    ret1, frame1 = cap1.read()
 
    cv2.imshow("FRAME", frame1)

    cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap1.release()
        #cap3.release()
        break