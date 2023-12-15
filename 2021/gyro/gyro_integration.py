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

file_name = "gyro_integ_{0:%Y%m%d-%H%M%S}".format(datetime.datetime.now())

fmt_name = "/home/pi/data/{}.csv".format(file_name)
header = "delta_t,x [dps],y [dps],z [dps]"

input("Press Enter and Rotate Gyro 180 degrees")
print("Recording Data...")
record_time = 5
data, t_vec = [[], [], []], []
t0 = time.time()
while time.time() - t0 < record_time:
    gyro = sensor.get_gyro()
    t_vec.append(time.time()-t0)
    data[0].append(gyro[0])
    data[1].append(gyro[1])
    data[2].append(gyro[2])

print(data)
    
f = open(fmt_name, "w")
f.write(header+"\n")
for i in range(len(data[0])):
    value = "%s,%6.3f,%6.3f,%6.3f" % (t_vec[i], data[0][i], data[1][i], data[2][i])
    f.write(value+"\n")
f.close()
