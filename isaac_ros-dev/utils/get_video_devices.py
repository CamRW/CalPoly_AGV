#!/usr/bin/env python3 
import pyudev
import re
import json
# This code is used to assign the correct video devices for identical cameras, it requires that the cameras are plugged into the same port every time
# If ports change, the sys path, listed as the value in the devices dict must be updated appropriately


if __name__ == "__main__":
    context = pyudev.Context()
    # '/dev/front_fisheye': '/sys/devices/platform/3610000.xhci/usb1/1-2/1-2.1/1-2.1:1.0',
    # '/dev/back_boxcam': '/sys/devices/platform/3610000.xhci/usb1/1-4/1-4.3/1-4.3:1.0'
    devices = {
            '/dev/left_fisheye': '/sys/devices/platform/3610000.xhci/usb1/1-4/1-4.2/1-4.2:1.0',
            '/dev/right_fisheye': '/sys/devices/platform/3610000.xhci/usb1/1-2/1-2.2/1-2.2.4/1-2.2.4:1.0'
            }

    device_dict = {}

    for key, val in devices.items():
        vid_arr = []
        # print(key, val)
        pyudev_dev = pyudev.Devices.from_sys_path(context, val)
        for child in pyudev_dev.children:
            child_dev_node = child.device_node
            # print(child_dev_node)
            if child_dev_node != None:
                if 'video' in child_dev_node:
                    vid_num = int(re.search(r'\d+', child_dev_node).group())
                    vid_arr.append(vid_num)
        device_dict[key] = min(vid_arr)

    print(device_dict)

    with open('/home/agxorin1/workspaces/isaac_ros-dev/configs/camera_configs/cameras.json', 'w') as fp:
        j = json.dumps(device_dict)
        fp.write(j)
