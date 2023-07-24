import cv2
import numpy as np

cap1 = cv2.VideoCapture("/dev/video8", cv2.CAP_V4L2) # Back
cap2 = cv2.VideoCapture("/dev/video4", cv2.CAP_V4L2) # Left
cap3 = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2) # Right

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

cap1.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

cap2.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# cap3.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
# cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

frame3 = np.zeros((480,640,3), dtype=np.uint8)


while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    ret2, frame3 = cap3.read()
    
    # images = [frame2, frame1, frame3]

    frame1 = np.pad(frame1, pad_width=((0,0),(320,320),(0,0)))
    merged_frames = np.concatenate((frame2, frame3), axis=1)
    #merged_frames = np.concatenate((frame1, frame3), axis=1)

    merged_frames = np.concatenate((merged_frames, frame1), axis=0)

    cv2.imshow("FRAME", merged_frames)

    cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap1.release()
        cap2.release()
        #cap3.release()
        break
