"""
2021, Sep
Author: T.S.
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

bugfix:2021/09/15: Change variables of def get_mag().
                   mag_x_low, mag_y_low, mag_z_low to mag_*_high
                   mag_x_high, mag_y_high, mag_z_high to mag_*_low

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


        """mpu9250_i2c.pyを参考に"""
        #AK8963 registers
        # self.AK8963_ADDR   = 0x0C
        self.AK8963_ST1    = 0x02
        self.HXH          = 0x04
        self.HYH          = 0x06
        self.HZH          = 0x08
        self.AK8963_ST1   = 0x02
        self.AK8963_ST2   = 0x09
        self.AK8963_CNTL  = 0x0A
        self.AK8963_ASAX = 0x10
        self.AK8963_ADDR = self.i2c.setup(0x0C)

        self.mag_sens = 4800.0 # magnetometer sensitivity: 4800 uT



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

        if _calibration:
            self.calibration_accel(1000)
        return

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
            _write_data = 0x02  # 0x02 = 0b0010 = 8 Hz
            self.MAG_MODE = self.MAG_MODE_SERIAL_1
        elif _mode == '100Hz':
            _write_data = 0x06  # 0x06 = 0b0110 = 100 Hz
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


    def calibration_accel(self, _count=1000):
        print('Accel calibration start...')
        _sum = [0, 0, 0]

        for i in range(_count):
            _data = self.get_accel()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # 平均値をオフセットに設定
        self.offset_accel_x = -1.0 * _sum[0] / _count
        self.offset_accel_y = -1.0 * _sum[1] / _count
        # z軸は，重力分の1.0 gを差し引く
        self.offset_accel_z = -1.0 * ((_sum[2] / _count) - 1.0)

        print('Accel calibration is done!')
        return self.offset_accel_x, self.offset_accel_y, self.offset_accel_z

    
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

        print('Gyro calibration is done!')
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
        # 2021.09.15 bugfix, change mag_*_high to mag_*_low and vice versa
        mag_x_low = self.i2c.readReg8(self.ak8963, 0x03)
        mag_x_high = self.i2c.readReg8(self.ak8963, 0x04)
        mag_y_low = self.i2c.readReg8(self.ak8963, 0x05)
        mag_y_high = self.i2c.readReg8(self.ak8963, 0x06)
        mag_z_low = self.i2c.readReg8(self.ak8963, 0x07)
        mag_z_high = self.i2c.readReg8(self.ak8963, 0x08)
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

        # print("{:8.3f} {:8.3f} {:8.3f}".format(raw_x, raw_y, raw_z))

        return raw_x, raw_y, raw_z
    
    """mpu9250_i2c.pyを参考に"""

    def AK8963_start(self):
        self.i2c.writeReg8(self.AK8963_ADDR,self.AK8963_CNTL,0x00)
        time.sleep(0.1)
        self.i2c.writeReg8(self.AK8963_ADDR,self.AK8963_CNTL,0x0F)
        time.sleep(0.1)
        # coeff_data = bus.read_i2c_block_data(self.AK8963_ADDR,self.AK8963_ASAX,3)
        coeff_data1 = self.i2c.readReg8(self.AK8963_ADDR,self.AK8963_ASAX)
        coeff_data2 = self.i2c.readReg8(self.AK8963_ADDR,self.AK8963_ASAX+1)
        coeff_data3 = self.i2c.readReg8(self.AK8963_ADDR,self.AK8963_ASAX+2)
        coeff_data = [coeff_data1, coeff_data2, coeff_data3]

        AK8963_coeffx = (0.5*(coeff_data[0]-128)) / 256.0 + 1.0
        AK8963_coeffy = (0.5*(coeff_data[1]-128)) / 256.0 + 1.0
        AK8963_coeffz = (0.5*(coeff_data[2]-128)) / 256.0 + 1.0
        time.sleep(0.1)
        self.i2c.writeReg8(self.AK8963_ADDR,self.AK8963_CNTL,0x00)
        time.sleep(0.1)
        AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
        AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
        AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
        # 0b0001 << 4 + 0b0110 = 0b10000 + 0b0110 = 0b10110 = 0x16 = 0d22
        self.i2c.writeReg8(self.AK8963_ADDR,self.AK8963_CNTL,AK8963_mode)   # AK8963_CNTL = 0x0A
        time.sleep(0.1)
        return [AK8963_coeffx,AK8963_coeffy,AK8963_coeffz] 
        
    def AK8963_reader(self, register):
        # read magnetometer values
        low = self.i2c.readReg8(self.AK8963_ADDR, register-1)
        high = self.i2c.readReg8(self.AK8963_ADDR, register)
        # print(low, high)
        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)
        # print(value)
        # convert to +- value
        if(value > 32768):
            value -= 65536
        
        return value

    def AK8963_conv(self):
        # raw magnetometer bits
        while 1:
    ##        if ((self.i2c.readReg8(AK8963_ADDR,AK8963_ST1) & 0x01))!=1:
    ##            return 0,0,0
            mag_x = self.AK8963_reader(self.HXH)
            mag_y = self.AK8963_reader(self.HYH)
            mag_z = self.AK8963_reader(self.HZH)

            # the next line is needed for AK8963
            if (self.i2c.readReg8(self.AK8963_ADDR,self.AK8963_ST2)) & 0x08!=0x08:
                break
            
        #convert to acceleration in g and gyro dps
    ##    m_x = AK8963_coeffs[0]*(mag_x/(2.0**15.0))*mag_sens
    ##    m_y = AK8963_coeffs[1]*(mag_y/(2.0**15.0))*mag_sens
    ##    m_z = AK8963_coeffs[2]*(mag_z/(2.0**15.0))*mag_sens
        m_x = (mag_x/(2.0**15.0))*self.mag_sens
        m_y = (mag_y/(2.0**15.0))*self.mag_sens
        m_z = (mag_z/(2.0**15.0))*self.mag_sens
        return m_x,m_y,m_z
    

    def get_axis(self):
        """
        around x axis -> pitch
        around y axis -> roll
        around z axis -> yaw
        """
        accel = self.get_accel()
        mag = self.get_mag()
        # Return the arc tangent of y/x in radians
        roll = math.atan2(accel[1], accel[2])
        # temp = math.sqrt(accel[2] ** 2 + accel[1] ** 2)
        temp = math.sqrt(accel[2]**2 + accel[1]**2)
        pitch = math.atan2(-accel[0], temp)
        # numerator = math.cos(roll) * mag[1] - math.sin(roll) * mag[2]
        # denominator1 = math.cos(pitch) * mag[0]
        # denominator2 = math.sin(pitch) * math.sin(roll) * mag[1]
        # denominator3 = math.sin(pitch) * math.cos(roll) * mag[2]
        # yaw = math.atan2(-numerator, denominator1+denominator2+denominator3)
        # return pitch, roll, yaw

        yaw = math.atan2(mag[1], mag[0])
        print(math.degrees(pitch), math.degrees(yaw), math.degrees(roll), mag[0], mag[1], mag[2])
        return pitch, yaw, roll, mag[0], mag[1], mag[2]

"""
2021, Sep
Author: T.S.
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

bugfix:2021/09/15: Change variables of def get_mag().
                   mag_x_low, mag_y_low, mag_z_low to mag_*_high
                   mag_x_high, mag_y_high, mag_z_high to mag_*_low

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


        """mpu9250_i2c.pyを参考に"""
        #AK8963 registers
        # self.AK8963_ADDR   = 0x0C
        self.AK8963_ST1    = 0x02
        self.HXH          = 0x04
        self.HYH          = 0x06
        self.HZH          = 0x08
        self.AK8963_ST1   = 0x02
        self.AK8963_ST2   = 0x09
        self.AK8963_CNTL  = 0x0A
        self.AK8963_ASAX = 0x10
        self.AK8963_ADDR = self.i2c.setup(0x0C)

        self.mag_sens = 4800.0 # magnetometer sensitivity: 4800 uT



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

        if _calibration:
            self.calibration_accel(1000)
        return

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
            _write_data = 0x02  # 0x02 = 0b0010 = 8 Hz
            self.MAG_MODE = self.MAG_MODE_SERIAL_1
        elif _mode == '100Hz':
            _write_data = 0x06  # 0x06 = 0b0110 = 100 Hz
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


    def calibration_accel(self, _count=1000):
        print('Accel calibration start...')
        _sum = [0, 0, 0]

        for i in range(_count):
            _data = self.get_accel()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # 平均値をオフセットに設定
        self.offset_accel_x = -1.0 * _sum[0] / _count
        self.offset_accel_y = -1.0 * _sum[1] / _count
        # z軸は，重力分の1.0 gを差し引く
        self.offset_accel_z = -1.0 * ((_sum[2] / _count) - 1.0)

        print('Accel calibration is done!')
        return self.offset_accel_x, self.offset_accel_y, self.offset_accel_z

    
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

        print('Gyro calibration is done!')
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
        # 2021.09.15 bugfix, change mag_*_high to mag_*_low and vice versa
        mag_x_low = self.i2c.readReg8(self.ak8963, 0x03)
        mag_x_high = self.i2c.readReg8(self.ak8963, 0x04)
        mag_y_low = self.i2c.readReg8(self.ak8963, 0x05)
        mag_y_high = self.i2c.readReg8(self.ak8963, 0x06)
        mag_z_low = self.i2c.readReg8(self.ak8963, 0x07)
        mag_z_high = self.i2c.readReg8(self.ak8963, 0x08)
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

        # print("{:8.3f} {:8.3f} {:8.3f}".format(raw_x, raw_y, raw_z))

        return raw_x, raw_y, raw_z
    
    """mpu9250_i2c.pyを参考に"""

    def AK8963_start(self):
        self.i2c.writeReg8(self.AK8963_ADDR,self.AK8963_CNTL,0x00)
        time.sleep(0.1)
        self.i2c.writeReg8(self.AK8963_ADDR,self.AK8963_CNTL,0x0F)
        time.sleep(0.1)
        # coeff_data = bus.read_i2c_block_data(self.AK8963_ADDR,self.AK8963_ASAX,3)
        coeff_data1 = self.i2c.readReg8(self.AK8963_ADDR,self.AK8963_ASAX)
        coeff_data2 = self.i2c.readReg8(self.AK8963_ADDR,self.AK8963_ASAX+1)
        coeff_data3 = self.i2c.readReg8(self.AK8963_ADDR,self.AK8963_ASAX+2)
        coeff_data = [coeff_data1, coeff_data2, coeff_data3]

        AK8963_coeffx = (0.5*(coeff_data[0]-128)) / 256.0 + 1.0
        AK8963_coeffy = (0.5*(coeff_data[1]-128)) / 256.0 + 1.0
        AK8963_coeffz = (0.5*(coeff_data[2]-128)) / 256.0 + 1.0
        time.sleep(0.1)
        self.i2c.writeReg8(self.AK8963_ADDR,self.AK8963_CNTL,0x00)
        time.sleep(0.1)
        AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
        AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
        AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
        # 0b0001 << 4 + 0b0110 = 0b10000 + 0b0110 = 0b10110 = 0x16 = 0d22
        self.i2c.writeReg8(self.AK8963_ADDR,self.AK8963_CNTL,AK8963_mode)   # AK8963_CNTL = 0x0A
        time.sleep(0.1)
        return [AK8963_coeffx,AK8963_coeffy,AK8963_coeffz] 
        
    def AK8963_reader(self, register):
        # read magnetometer values
        low = self.i2c.readReg8(self.AK8963_ADDR, register-1)
        high = self.i2c.readReg8(self.AK8963_ADDR, register)
        # print(low, high)
        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)
        # print(value)
        # convert to +- value
        if(value > 32768):
            value -= 65536
        
        return value

    def AK8963_conv(self):
        # raw magnetometer bits
        while 1:
    ##        if ((self.i2c.readReg8(AK8963_ADDR,AK8963_ST1) & 0x01))!=1:
    ##            return 0,0,0
            mag_x = self.AK8963_reader(self.HXH)
            mag_y = self.AK8963_reader(self.HYH)
            mag_z = self.AK8963_reader(self.HZH)

            # the next line is needed for AK8963
            if (self.i2c.readReg8(self.AK8963_ADDR,self.AK8963_ST2)) & 0x08!=0x08:
                break
            
        #convert to acceleration in g and gyro dps
    ##    m_x = AK8963_coeffs[0]*(mag_x/(2.0**15.0))*mag_sens
    ##    m_y = AK8963_coeffs[1]*(mag_y/(2.0**15.0))*mag_sens
    ##    m_z = AK8963_coeffs[2]*(mag_z/(2.0**15.0))*mag_sens
        m_x = (mag_x/(2.0**15.0))*self.mag_sens
        m_y = (mag_y/(2.0**15.0))*self.mag_sens
        m_z = (mag_z/(2.0**15.0))*self.mag_sens
        return m_x,m_y,m_z
    

    def get_axis(self):
        """
        around x axis -> pitch
        around y axis -> roll
        around z axis -> yaw
        """
        accel = self.get_accel()
        gyro = self.get_gyro()
        mag = self.get_mag()
        # Return the arc tangent of y/x in radians
        # roll = math.atan2(accel[1], accel[2])

        # temp = math.sqrt(accel[2] ** 2 + accel[1] ** 2)
        # temp = math.sqrt(accel[2]**2 + accel[1]**2)
        # pitch = math.atan2(-accel[0], temp)

        # numerator = math.cos(roll) * mag[1] - math.sin(roll) * mag[2]
        # denominator1 = math.cos(pitch) * mag[0]
        # denominator2 = math.sin(pitch) * math.sin(roll) * mag[1]
        # denominator3 = math.sin(pitch) * math.cos(roll) * mag[2]
        # yaw = math.atan2(-numerator, denominator1+denominator2+denominator3)
        # return pitch, roll, yaw

        # yaw = math.atan2(mag[1], mag[0])
        # print("{:8.3f} {:8.3f} {:8.3f} {:8.3f} {:8.3f}".format(math.degrees(pitch), math.degrees(yaw), math.degrees(roll), mag[0], mag[1], mag[2]))
        # return pitch, yaw, roll, mag[0], mag[1], mag[2] 
        return accel, gyro, mag

