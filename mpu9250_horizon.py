import sys              # sysモジュールの呼び出し
import wiringpi as wi   # wiringPiモジュールの呼び出し
import time             # timeライブラリの呼び出し
import datetime         # datetimeモジュールの呼び出し
import os
from mpu9250_horizon_def import *

SAMPLING_TIME = 0.1
TIMES = 100

if __name__ == '__main__':

# bus = smbus.SMBus(1)

    resetRegister()
    powerWakeUp()
    gyroCoefficient = gyroRange / float(0x8000)   # coefficient : sensed decimal val to dps val.
    accelCoefficient = accelRange / float(0x8000)   # coefficient : sensed decimal val to g val
    magCoefficient16 = magRange / 32760.0         # coefficient : sensed decimal val to μT val (16bit)
    magCoefficient14 = magRange / 8190.0          # coefficient : sensed decimal val to μT val (14bit)
    setAccelRange(accelRange, False)
    setGyroRange(gyroRange, False)
    setMagRegister('100Hz', '16bit')
    while True:
#     for _i in range(TIMES):
        try:
            date = datetime.datetime.now()  # now()メソッドで現在日付・時刻のdatetime型データの変数を取得 世界時：UTCnow
            now = time.time()     # 現在時刻の取得
            acc = getAccel(accelCoefficient)         # 加速度値の取得
            gyr = getGyro(gyroCoefficient)         # ジャイロ値の取得
            mag = getMag(magCoefficient14, magCoefficient16)         # 磁気値の取得
            roll  = cal_roll(acc)
            pitch = cal_pitch(acc)
            value = '%s, %6.3f, %6.3f' % (date, roll, pitch)
            print(value)
            # 指定秒数の一時停止
            sleepTime = SAMPLING_TIME - (time.time() - now)
            if sleepTime < 0.0:
                continue
            time.sleep(sleepTime)
            
        except KeyboardInterrupt:
            print('end')
            break
