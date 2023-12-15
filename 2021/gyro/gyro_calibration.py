import time, datetime
import sys

sys.path.append("../")
from mpu9250 import *

sensor = MPU9250(0.1, 0x68, 0x0c, 2, 250, 4800)
sensor.reset_register()
sensor.power_wake_up()
sensor.set_accel_range(False)
sensor.set_gyro_range(True)
sensor.set_mag_register('100Hz', '16bit')

"""ジャイロセンサの値を取得"""
file_name = "uncalibrated_{0:%Y%m%d-%H%M%S}".format(datetime.datetime.now())
file_name = "calibrated_{0:%Y%m%d-%H%M%S}".format(datetime.datetime.now())

fmt_name = "/home/pi/data/{}.csv".format(file_name)
header = "yyyy-mm-dd hh:mm:ss.mmmmmm,x [dps],y [dps],z [dps]"
f = open(fmt_name, "w")
f.write(header+"\n")
for i in range(500):
    date = datetime.datetime.now()
    now = time.time()
    gyro = sensor.get_gyro()
    value = "%s,%6.3f,%6.3f,%6.3f" % (date, gyro[0], gyro[1], gyro[2])
    f.write(value+"\n")

    sleep_time = sensor.sampling_time - (time.time()-now)
    if sleep_time < 0.0:
        continue
    time.sleep(sleep_time)
