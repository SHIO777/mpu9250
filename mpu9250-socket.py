import mpu9250
import time
 #coding: utf-8
from socket import socket, AF_INET, SOCK_DGRAM

PORT = 5000
ADDRESS = "XXX.XXX.XXX.XXX" # targetIP
s = socket(AF_INET, SOCK_DGRAM)
sensor = mpu9250.SL_MPU9250(0x68,1)
sensor.resetRegister()
sensor.powerWakeUp()
sensor.setAccelRange(8,True)
sensor.setGyroRange(1000,True)
sensor.setMagRegister('100Hz','16bit')
while True:
    axis = sensor.getAxis()
    data = str(axis[0])+','+str(axis[1])+','+str(axis[2])
    print(data)
    s.sendto(data.encode(), (ADDRESS, PORT))
    time.sleep(0.01)
s.close()