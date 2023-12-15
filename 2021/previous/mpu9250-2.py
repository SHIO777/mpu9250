import mpu9250
import time
from socket import socket, AF_INET, SOCK_DGRAM


PORT = 5000
ADDRESS = "192.168.11.2"
s = socket(AF_INET, SOCK_DGRAM)
sensor = mpu9250.MPU9250(1.0, 0x68, 0x0c)
sensor.reset_register()
sensor.power_wake_up()
sensor.set_accel_range(8, True)
sensor.set_gyro_range(1000, True)
sensor.set_mag_register('100Hz', '16bit')
while True:
    now = time.time()
    axis = sensor.get_axis()
    data = str(axis[0]) + ',' + str(axis[1]) + ',' + str(axis[2])
    print(data)
    s.sendto(data.encode(), (ADDRESS, PORT))
    sleep_time = sensor.sampling_time - (time.time() - now)
    if sleep_time < 0.0:
        continue
    time.sleep(sleep_time)
s.close()
