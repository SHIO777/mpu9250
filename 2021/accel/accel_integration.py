import time, datetime
import sys

sys.path.append("../")
from mpu9250 import *

def imu_integrator():
    dt_stop = 5.0   # seconds to record and integrate
    data_array, t_array = [], []
    print("Starting Data Acquisition")
    loop_bool = False
    sensor = MPU9250(0.1, 0x68, 0x0c, 2, 250, 4800)
    sensor.reset_register()
    sensor.power_wake_up()
    sensor.set_accel_range(True)
    sensor.set_gyro_range(True)
    sensor.set_mag_register('100Hz', '16bit')
    input("Press Enter to start")
    t0 = time.time()
    while True:
        try:
            ax, ay, az = sensor.get_accel()
            wx, wy, wz = sensor.get_gyro()
            mx, my, mz = sensor.get_mag()
            t_array.append(time.time()-t0)
            data = [ax, ay, az, wx, wy, wz, mx, my, mz]
            data_array.append(data)
            if not loop_bool:
                loop_bool = True
                print("Start Moving IMU...")
        except:
            continue
        if time.time()-t0 > dt_stop:
            print("Data Acquisition Stopped")
            break
    
    file_name = "accel_integ_{0:%Y%m%d-%H%M%S}".format(datetime.datetime.now())
    fmt_name = "/home/pi/data/{}.csv".format(file_name)
    header = "delta_t,x [g],y [g],z [g],x [dps],y [dps],z [dps],x [uT],y [uT],z [uT]"
    f = open(fmt_name, "w")
    f.write(header+"\n")

    for i in range(len(data_array)):
        value = "%s,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f" \
            % (t_array[i], data_array[i][0], data_array[i][1], data_array[i][2], data_array[i][3], data_array[i][4], data_array[i][5], data_array[i][6], data_array[i][7], data_array[i][8])
        f.write(value+"\n")

    f.close()
    print("length: ", len(data_array))

if __name__ == '__main__':
    imu_integrator()
