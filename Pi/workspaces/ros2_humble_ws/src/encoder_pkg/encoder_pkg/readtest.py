import serial
import time
import sys

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
# ser.flush()
# print(ser.in_waiting)
# print(ser.in_waiting)
# print(ser.readline().decode()) 
print(type(ser.readline()))
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

#while True:
    # ser.write(bytes(message, 'utf-8'))
#    print(ser.readline())



#     # ser.write(bytes(message, 'utf-8'))
#     while ser.in_waiting > 0:W
#         # encoder_data = ser.readline().decode().strip()
#         # encoder_data_split = encoder_data.split(',') 
#         print(ser.readline().decode())
#         time.sleep(0.01)
#     # ser.write(bytes(message, 'utf-8'))
