import serial
import time
import sys

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
ser.close()



# message = str(1) + "," + str(2)
# time.sleep(1)
# # ser.flush()
# # print(bytes(message, 'utf-8'))
# # print(ser.in_waiting)
# # ser.write(bytes(message, 'utf-8'))
# # print(ser.in_waiting)
# # print(ser.readline())
# # print(ser.in_waiting)
# # print(ser.readline().decode()) 
# while True:
#     # ser.write(bytes(message, 'utf-8'))
#     print(ser.readline())
#     # ser.write(bytes(message, 'utf-8'))
#     # print(ser.in_waiting)
#     time.sleep(1)


# # #     # ser.write(bytes(message, 'utf-8'))
# #     while ser.in_waiting > 0:
# #         # encoder_data = ser.readline().decode().strip()
# #         # encoder_data_split = encoder_data.split(',') 
# #         print(ser.readline().decode())
# #         time.sleep(0.01)
# # #     # ser.write(bytes(message, 'utf-8'))