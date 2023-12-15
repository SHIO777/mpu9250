"""
2021, Sep
Author: Tatsuya Shiotsuka
########################################
This code handles the wiringpi
communications between the RaspberryPi and the
MPU9250 IMU.

Reference:
https://makersportal.com/blog/calibration-of-an-inertial-measurement-unit-with-raspberry-pi

to start venv -> cd pi/MPU9250
              -> . venv/bin/activate

########################################
bugfix:2021/09/11: Change the mag_range 4912 to 4800 uT.
                   Change the denominator of mag_coefficient_16 and mag_coefficient_14,
                   32760.0 and 8190.0 to (2.0 ** 15.0) and (2.0 ** 13.0) respectively.

update:YYYY/MM/DD:
bugfix:YYYY/MM/DD:   
"""

import wiringpi as wi
import time, datetime
import math

class MPU9250:
    
    REG_PWR_MGMT_1 = 0x6B
    REG_INT_PIN_CFG = 0x37
    REG_ACCEL_CONFIG_1 = 0x1C
    REG_ACCEL_CONFIG_2 = 0x1D
    REG_GYRO_CONFIG = 0x1B

    MAG_MODE_POWER_DOWN = 0
    MAG_MODE_SERIAL_1 = 1
    MAG_MODE_SERIAL_2 = 2
    MAG_MODE_SINGLE = 3
    MAG_MODE_EX_TRIGGER = 4
    MAG_MODE_SELF_TEST = 5
    MAG_ACCESS = False
    MAG_MODE = 0
    MAG_BIT = 16

    offset_accel_x = 0
    offset_accel_y = 0
    offset_accel_z = 0
    offset_gyro_x = 0
    offset_gyro_y = 0
    offset_gyro_z = 0

    def __init__(self, sampling_time, mpu9250_address, ak8963_address, 
                    accel_range=8, gyro_range=1000, mag_range=4800):
        """
        :param sampling_time: sampling time in sec
        :param mpu9250_address: 0x68
        :param ak8963_address: 0x0c
        :param accel_range: 2, 4, 8, 16 [g]
        :param gyro_range: 250, 500, 1000, 2000 [dps]
        :param mag_range: 4800 [uT]
        """

        self.sampling_time = sampling_time
        self.mpu9250_address = mpu9250_address          # ジャイロ、加速度のアドレス
        self.ak8963_address = ak8963_address            # コンパスのアドレス

        wi.wiringPiSetup()
        self.i2c = wi.I2C()

        self.mpu9250 = self.i2c.setup(mpu9250_address)
        self.ak8963 = self.i2c.setup(ak8963_address)

        # coefficient
        self.gyro_range = gyro_range
        self.accel_range = accel_range
        self.mag_range = mag_range

        # センサ初期化
        self.reset_register()
        self.power_wake_up()

        # センシングされたdecimal値をdpsに変換する係数
        self.gyro_coefficient = self.gyro_range / (2.0**15.0)
        # センシングされたdecimal値をgに変換する係数
        self.accel_coefficient = self.accel_range / (2.0**15.0)
        # センシングされたdecimal値をuTに変換する係数（16bit）
        self.mag_coefficient_16 = self.mag_range / (2.0**15.0)
        # self.mag_coefficient_16 = self.mag_range / (2.0 ** 15.0)
        # センシングされたdecimal値をuTに変換する係数（14bit）
        self.mag_coefficient_14 = self.mag_range / (2.0**13.0)
        # self.mag_coefficient_14 = self.mag_range / (2.0 ** 13.0)


    def reset_register(self):
        """レジスタを初期設定に戻す"""
        if self.MAG_ACCESS:
            self.i2c.writeReg8(self.ak8963, 0x0B, 0x01)
        self.i2c.writeReg8(self.mpu9250, 0x6B, 0x80)
        self.MAG_ACCESS = False
        time.sleep(0.1)

    def power_wake_up(self):
        """レジスタをセンシング可能な状態にする"""
        # REG_PWR_MGMT_1をクリア
        self.i2c.writeReg8(self.mpu9250, self.REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # I2Cで磁気センサ機能へアクセスできるようにする
        self.i2c.writeReg8(self.mpu9250, self.REG_INT_PIN_CFG, 0x02)
        self.MAG_ACCESS = True
        time.sleep(0.1)


    def set_accel_range(self,  _calibration=False):
        """
        :param _calibration: boolean
        :return: None
        """
        if self.accel_range == 16:
            _data = 0x18
        elif self.accel_range == 8:
            _data = 0x10
        elif self.accel_range == 4:
            _data = 0x08
        else:
            _data = 0x00
        self.i2c.writeReg8(self.mpu9250, self.REG_ACCEL_CONFIG_1, _data)
        time.sleep(0.1)

        # オフセット値のリセット
        self.offset_accel_x = 0
        self.offset_accel_y = 0
        self.offset_accel_z = 0

    def set_gyro_range(self, _calibration=False):
        """
        :param _calibration: Boolean
        :return: None
        """
        if self.gyro_range == 2000:
            _data = 0x18
        elif self.gyro_range == 1000:
            _data = 0x10
        elif self.gyro_range == 500:
            _data = 0x08
        else:
            _data = 0x00
        self.i2c.writeReg8(self.mpu9250, self.REG_GYRO_CONFIG, _data)
        time.sleep(0.1)

        # オフセット値のリセット
        self.offset_gyro_x = 0
        self.offset_gyro_y = 0
        self.offset_gyro_z = 0

        if _calibration:
            self.calibration_gyro(1000)
        return

    def set_mag_register(self, _mode, _bit):
        """
        :param _mode:
            8Hz:        磁気センサ8Hz連続測定モード,
            100Hz:      磁気センサ100Hz連続測定モード,
        :param _bit: 14bit or 16bit
            磁気センサが出力するビット数
        :return: None
        """
        if not self.MAG_ACCESS:
            raise Exception('001 Access to a sensor is invalid.')

        _write_data = 0x00
        if _mode == '8Hz':
            _write_data = 0x02
            self.MAG_MODE = self.MAG_MODE_SERIAL_1
        elif _mode == '100Hz':
            _write_data = 0x06
            self.MAG_MODE = self.MAG_MODE_SERIAL_2
        else:
            raise Exception('002 set_mag_register write data "%s" is not defined' % _mode)

        if _bit == '14bit':
            _write_data = _write_data | 0x00
            self.MAG_BIT = 14
        elif _bit == '16bit':
            _write_data = _write_data | 0x10
            self.MAG_BIT = 16
        else:
            raise Exception('003 set_mag_register _bit "%s" is not defined' % _bit)

        self.i2c.writeReg8(self.ak8963, 0x0A, _write_data)

    
    def calibration_gyro(self, _count=1000):
        print("Gyro calibration start...")
        _sum = [0, 0, 0] # TODO: 0.0 floatにする必要

        for i in range(_count):
            _data = self.get_gyro()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]
        
        self.offset_gyro_x = -1.0 * _sum[0] / _count
        self.offset_gyro_y = -1.0 * _sum[1] / _count
        self.offset_gyro_z = -1.0 * _sum[2] / _count

        print('Gyro calibration complete!')
        return self.offset_gyro_x, self.offset_gyro_y, self.offset_gyro_z


    def u2s(self, unsigned_data):
        """
        unsigned to signed 符号なしから符号ありへ
        :param unsigned_data: 16bit unsigned data
        :return: 16bit signed data
        """
        if unsigned_data & (0x01 << 15):
            # 上のビット演算が0以外であれば(負数であれば)
            # ^ XOR どちらかが1のとき
            return -1 * ((unsigned_data ^ 0xffff) + 1)
        return unsigned_data


    def get_accel(self):
        accel_x_high = self.i2c.readReg8(self.mpu9250, 0x3B)
        accel_x_low = self.i2c.readReg8(self.mpu9250, 0x3C)
        accel_y_high = self.i2c.readReg8(self.mpu9250, 0x3D)
        accel_y_low = self.i2c.readReg8(self.mpu9250, 0x3E)
        accel_z_high = self.i2c.readReg8(self.mpu9250, 0x3F)
        accel_z_low = self.i2c.readReg8(self.mpu9250, 0x40)
        # print(accel_x_high, accel_x_low, accel_y_high, accel_y_low, accel_z_high, accel_z_low)
        raw_x = self.accel_coefficient * self.u2s(accel_x_high << 8 | accel_x_low) + self.offset_accel_x
        raw_y = self.accel_coefficient * self.u2s(accel_y_high << 8 | accel_y_low) + self.offset_accel_y
        raw_z = self.accel_coefficient * self.u2s(accel_z_high << 8 | accel_z_low) + self.offset_accel_z
        return raw_x, raw_y, raw_z


    def get_gyro(self):
        gyro_x_high = self.i2c.readReg8(self.mpu9250, 0x43)
        gyro_x_low = self.i2c.readReg8(self.mpu9250, 0x44)
        gyro_y_high = self.i2c.readReg8(self.mpu9250, 0x45)
        gyro_y_low = self.i2c.readReg8(self.mpu9250, 0x46)
        gyro_z_high = self.i2c.readReg8(self.mpu9250, 0x47)
        gyro_z_low = self.i2c.readReg8(self.mpu9250, 0x48)
        raw_x = self.gyro_coefficient * self.u2s(gyro_x_high << 8 | gyro_x_low) + self.offset_gyro_x
        raw_y = self.gyro_coefficient * self.u2s(gyro_y_high << 8 | gyro_y_low) + self.offset_gyro_y
        raw_z = self.gyro_coefficient * self.u2s(gyro_z_high << 8 | gyro_z_low) + self.offset_gyro_z
        return raw_x, raw_y, raw_z

    def get_mag(self):
        if not self.MAG_ACCESS:
            raise Exception('004 access to a mag sensor denied.')

        if self.MAG_MODE == self.MAG_MODE_SINGLE:
            if self.MAG_BIT == 14:
                _write_data = 0x01
            else:
                _write_data = 0x11
            self.i2c.writeReg8(self.ak8963, 0x0A, _write_data)
            time.sleep(0.01)
        elif self.MAG_MODE == self.MAG_MODE_SERIAL_1 or self.MAG_MODE == self.MAG_MODE_SERIAL_2:
            status = self.i2c.readReg8(self.ak8963, 0x02)
            if (status & 0x02) == 0x02:
                # データオーバーランがあるので再度センシング
                self.i2c.readReg8(self.ak8963, 0x09)
        elif self.MAG_MODE == self.MAG_MODE_EX_TRIGGER:
            # todo: 実装
            return
        elif self.MAG_MODE == self.MAG_MODE_POWER_DOWN:
            raise Exception('005 Mag sensor power down')
        # ST1レジスタを確認してデータ読み出しが可能か確認する
        status = self.i2c.readReg8(self.ak8963, 0x02)
        while (status & 0x01) != 0x01:
            # データレディ状態まで待つ
            time.sleep(0.01)
            status = self.i2c.readReg8(self.ak8963, 0x02)

        # データ読み出し
        mag_x_high = self.i2c.readReg8(self.ak8963, 0x03)
        mag_x_low = self.i2c.readReg8(self.ak8963, 0x04)
        mag_y_high = self.i2c.readReg8(self.ak8963, 0x05)
        mag_y_low = self.i2c.readReg8(self.ak8963, 0x06)
        mag_z_high = self.i2c.readReg8(self.ak8963, 0x07)
        mag_z_low = self.i2c.readReg8(self.ak8963, 0x08)
        mag_off = self.i2c.readReg8(self.ak8963, 0x09)
        raw_x = self.gyro_coefficient * self.u2s(mag_x_high << 8 | mag_x_low)
        raw_y = self.gyro_coefficient * self.u2s(mag_y_high << 8 | mag_y_low)
        raw_z = self.gyro_coefficient * self.u2s(mag_z_high << 8 | mag_z_low)
        st2 = mag_off

        # オーバーフローチェック
        if (st2 & 0x08) == 0x08:
            # オーバーフローのため正しい値が得られていない
            raise Exception('006 Mag sensor over flow')

        # uTへの変換
        if self.MAG_BIT == 16:
            raw_x = raw_x * self.mag_coefficient_16
            raw_y = raw_y * self.mag_coefficient_16
            raw_z = raw_z * self.mag_coefficient_16
        else:
            raw_x = raw_x * self.mag_coefficient_14
            raw_y = raw_y * self.mag_coefficient_14
            raw_z = raw_z * self.mag_coefficient_14

        return raw_x, raw_y, raw_z

if __name__ == '__main__':
    sensor = MPU9250(0.1, 0x68, 0x0c, 2, 250, 4800)
    sensor.reset_register()
    sensor.power_wake_up()
    sensor.set_accel_range(False)
    sensor.set_gyro_range(True)
    sensor.set_mag_register('100Hz', '16bit')

    file_name = "calibrated_data"
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
    f.close()



    # while True:
    #     now = time.time()
    #     accel = sensor.get_accel()
    #     gyro = sensor.get_gyro()
    #     mag = sensor.get_mag()
    #     # print(time.time()-now)

    #     """各センサの値を表示"""
    #     imu_devs   = ["ACCELEROMETER","GYROSCOPE","MAGNETOMETER"]
    #     imu_labels = ["x-dir","y-dir","z-dir"]
    #     imu_units  = ["g","g","g","dps","dps","dps","uT","uT","uT"]
    #     imu_vals = [accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]]
    #     # for i, imu_value in enumerate(imu_vals):
    #     #     if i % 3 == 0:
    #     #         print(20*"_"+"\n"+imu_devs[int(i/3)])
    #     #     print("{0}: {1:3.2f} {2}".format(imu_labels[i%3], imu_value, imu_units[i]))

    #     horizontal_intensity = math.sqrt(mag[0]*mag[0]+mag[1]*mag[1])
    #     print(horizontal_intensity)

    #     sleep_time = sensor.sampling_time - (time.time()-now)
    #     if sleep_time < 0.0:
    #         continue
    #     time.sleep(sleep_time)
