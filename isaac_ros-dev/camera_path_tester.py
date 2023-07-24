import os
import re
import json

DEFAULT_CAMERA_NAMES = ["/dev/back_fisheye","/dev/left_fisheye","/dev/right_fisheye"]

device_num = 0
device_dict = dict()


for device_name in DEFAULT_CAMERA_NAMES:
    if os.path.exists(device_name):
        device_path = os.path.realpath(device_name)
        device_re = re.compile("\/dev\/video(\d+)")
        info = device_re.match(device_path)
        print(info)
        if info:
            device_num = int(info.group(1))
            real_device = device_num - device_num % 4
            print("Using default video capture device on /dev/video" + str(device_num))
            print("Real Device for " + device_name + " on /dev/video" + str(real_device))
            device_dict[device_name] = real_device

print(device_dict)

with open('configs/camera_configs/cameras.json', 'w') as fp:
    j = json.dumps(device_dict, fp)
    fp.write(j)