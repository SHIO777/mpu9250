import mpu9250
import time
from socket import socket, AF_INET, SOCK_DGRAM
import math as m

PORT = 5000
ADDRESS = "192.168.1.35"
s = socket(AF_INET, SOCK_DGRAM)
sensor = mpu9250.MPU9250(0.1, 0x68, 0x0c, 2, 250, 4800)
sensor.reset_register()
sensor.power_wake_up()
sensor.set_accel_range(True)
sensor.set_gyro_range(True)
sensor.set_mag_register('100Hz', '16bit')

log_data = []

try:
    while True:
        now = time.time()
        axis = sensor.get_axis()    # -> pitch, yaw, roll
        data = str(axis[0]) + ',' + str(axis[1]) + ',' + str(axis[2])
    #     print(data)
        s.sendto(data.encode(), (ADDRESS, PORT))
        # print("{:8.3f} {:8.3f}".format(axis[3], axis[4]))
        # print("{:8.3f} {:8.3f} {:8.3f}".format(m.degrees(axis[0]), m.degrees(axis[1]), m.degrees(axis[2])))
        print("-"*30)
        log_data.append(axis)
        sleep_time = sensor.sampling_time - (time.time() - now)
        if sleep_time < 0.0:
            continue
        time.sleep(sleep_time)

finally:
    print("\n udp.py is stopping...\n")
    s.close()
    print("Prepairing output log file...")

    """ログ出力"""
    # import datetime
    # file_name = "udp_log_{0:%Y%m%d-%H%M%S}".format(datetime.datetime.now())
    # fmt_name = "/home/pi/data/{}.csv".format(file_name)
    # header = "pitch,yaw,roll,mag[0],mag[1],mag[2]"
    # f = open(fmt_name, "w")
    # f.write(header+"\n")
    # for i in range(len(log_data)):
    #     value = "%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f" % (log_data[i][0], log_data[i][1], log_data[i][2], log_data[i][3], log_data[i][4], log_data[i][5])
    #     f.write(value+"\n")
    # f.close()
