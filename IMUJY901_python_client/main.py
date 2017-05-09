import numpy as np
import os
import math
import time
import serial
import struct
import socket
import cv2
import threading
orientation = np.array([0.0, 0.0, 0.0, 0.0]) # original quaternion

unsentData = True   # true when there are new values of IMU that have not been sent to unity

#reads from IMU euler and quaternions. from documentation euler angles start with characters 'US' and quaternions from 'UY'
def readSerialAngles(ser):
    extraPackets = ser.in_waiting / 33 #determines how many old packets are waiting in buffer
    junk = ser.read(extraPackets*33) #read all old packets
    euler = ser.read(11)
    euler = struct.unpack('cchhhhc', euler) #US euler
    quat = ser.read(11)
    quat = struct.unpack('cchhhhc', quat) #UY quaternion
    if euler[0] != 'U':
        print "error connecting to IMU serial"
    if euler[1] != 'S':   #  swap euler and quaternon based on starting character 'US'
        temp = quat
        quat = euler
        euler = temp
    roll = euler[4] / 32768.0 * 180   #calculation from documentation to get angle in degrees
    pitch = euler[2] / 32768.0 * 180
    yaw = euler[3] / 32768.0 * 180

    ax = 0  # currently not directly reading acceleration values
    ay = 0
    az = 0

    q0 = quat[2] / 32768.0
    q1 = quat[3] / 32768.0
    q2 = quat[4] / 32768.0
    q3 = quat[5] / 32768.0
    return (pitch, yaw, roll, ax, ay, az, q0, q1, q2, q3)

# reads data from IMU, reads junk until it finds the start of a message
def readIMU():
    print "readIMU function"
    ser = serial.Serial()
    ser.set_buffer_size(rx_size=33)
    ser.port = 'COM3'
    ser.baudrate = 115200
    ser.open()
    if not ser.is_open:
        print "error: serial port not opened"
        os.system("pause")
    else:
        while True:
            junk = ser.read(1)
            junk = struct.unpack('c', junk)
            if junk == ('Y',):
                junk = ser.read(9)
                junk = struct.unpack('ccccccccc', junk)
                break
        while cv2.waitKey(1) != 27 and ser.is_open:
            pitch, yaw, roll, ax, ay, az, q0, q1, q2, q3 = readSerialAngles(ser)
            q = [q0, q1, q2, q3]
            global orientation
            global unsentData
            orientation = q
            unsentData = True
            #accel = removeGravity(accel, q)
            #deadReckon(accel)
        ser.close()


def sendData():
    global orientation
    global unsentData
    address = ('127.0.0.1',8888)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    #sock.setsockopt(socket.IPPROTO_IP, socket.IPPROTO_IP, 2)
    while True:
        if unsentData:
           # print "send"
            packet = struct.pack('ffffc', orientation[0],
                                 orientation[1], orientation[2], orientation[3], 'A')
            sock.sendto(packet,address)
            unsentData = False
            print orientation


t1=threading.Thread(target=readIMU)
t2=threading.Thread(target=sendData)
t1.start()
t2.start()

print "main func running"

t1.join()
t2.join()

print "end"