import os
import re

DEFAULT_CAMERA_NAME = '/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0'

device_num = 0
if os.path.exists(DEFAULT_CAMERA_NAME):
    device_path = os.path.realpath(DEFAULT_CAMERA_NAME)
    device_re = re.compile("\/dev\/video(\d+)")
    print(device_re)
    info = device_re.match(device_path)
    print(info)
    if info:
        device_num = int(info.group(1))
        print("Using default video capture device on /dev/video" + str(device_num))