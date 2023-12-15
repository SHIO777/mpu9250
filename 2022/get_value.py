import datetime
from tkinter.messagebox import askokcancel 
import mpu9250
import time
import math as m


sampling_time = 10.0

sensor = mpu9250.MPU9250(0.1, 0x68, 0x0c, 2, 250, 4800)
sensor.reset_register()
sensor.power_wake_up()
sensor.set_accel_range(True)
sensor.set_gyro_range(True)
sensor.set_mag_register('100Hz', '16bit')

file_name = "pitch_10_{0:%Y%m%d-%H%M%S}".format(datetime.datetime.now())
f_name = "/home/pi/mpu9250/data/{}.csv".format(file_name)
header = "date,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z"
f = open(f_name, "w")
f.write(header+"\n")

start = time.time()

try:
    # while True:
    while time.time() - start <= sampling_time:
        now = time.time()
        date = datetime.datetime.now()
        acc, gyr, mag = sensor.get_axis()
        
        # print("accel", acc)
        # print("gyro ", gyr)
        # print("mag  ", mag)
        
        value = "%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f" % (
            date, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], mag[0], mag[1], mag[2])
        f.write(value+"\n")

        sleep_time = sensor.sampling_time - (time.time() - now)
        if sleep_time < 0.0:
            continue
        time.sleep(sleep_time)

finally:
    f.close()
    print("\n udp.py is stopping...\n")
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
