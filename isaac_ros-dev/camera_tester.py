import cv2
import numpy as np

#cap1 = cv2.VideoCapture("/dev/right_fisheye") # Fisheye Back
#cap2 = cv2.VideoCapture("/dev/video10") # Left
cap3 = cv2.VideoCapture("/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0") # Right
# left is 6
# 12 is right

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# cap1.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
# cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# cap2.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
# cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

cap3.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


while True:
    # ret1, frame1 = cap1.read()
    # ret2, frame2 = cap2.read()
    ret2, frame3 = cap3.read()
    
    # images = [frame2, frame1, frame3]

    # imageStitcher = cv2.Stitcher_create()
    # frame1 = np.pad(frame1, pad_width=((0,0),(320,320),(0,0)))
    # merged_frames = np.concatenate((frame2, frame3), axis=1)
    # merged_frames = np.concatenate((merged_frames, frame1), axis=0)

    # merged_frames = imageStitcher.stitch(images)


    cv2.imshow("FRAME", frame3)

    cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
