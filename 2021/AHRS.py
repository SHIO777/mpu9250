# reference
# https://github.com/STMicroelectronics-CentralLabs/ST_Drone_FCU_F401/blob/master/STM32%20FW%20Project/Official%20latest%20release%20221117/Src/ahrs.c

import math


q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0

#float gx_off, gy_off, gz_off, mx_mag, my_mag, mz_mag

wbx, wby, wbz = 0.0, 0.0, 0.0
by, bz = 1.0, 0.0
exInt, eyInt, ezInt = 0.0, 0.0, 0.0


ahrs_init_flag = 0
acc_over = 0
# int count
# int gTHR
# int ahrs_kp

"""AHRS.h"""
COE_MDPS_TO_RADPS = 1.745329252e-5
SENSOR_SAMPLING_TIME = 0.00625
AHRS_KP_BIG = 10.0      # 0.4 is tested, so slow in calibrated.
AHRS_KP_NORM = 0.4
AHRS_KI = 0.1

"""flight_control.h"""
MIN_THR = 200       # DC motor configuration


def ahrs_fusion_ag(a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z):
    global q0, q1, q2, q3
    if gTHR < MIN_THR:
        ahrs_kp = AHRS_KP_BIG
    else:
        ahrs_kp = AHRS_KP_NORM

    axf, ayf, azf = a_x, a_y, a_z

    # mdps convert to rad/s
    gxf = g_x * COE_MDPS_TO_RADPS
    gyf = g_y * COE_MDPS_TO_RADPS
    gzf = g_z * COE_MDPS_TO_RADPS

    # auxiliary variables to reduce number of repeated operations
    q0q0 = q0 * q0
    q0q1 = q0 * q1
    q0q2 = q0 * q2
    q0q3 = q0 * q3
    q1q1 = q1 * q1
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q2q2 = q2 * q2
    q2q3 = q2 * q3
    q3q3 = q3 * q3

    # normalise the accelerometer measurement
    norm = 1 / math.sqrt(axf*axf+ayf*ayf+azf*azf)

    axf = axf * norm
    ayf = ayf * norm
    azf = azf * norm

    # estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2)
    vy = 2 * (q0q1 + q2q3)
    vz = q0q0 - q1q1 - q2q2 + q3q3

    ex = ayf*vz - azf*vy
    ey = azf*vx - axf*vz
    ez = axf*vy - ayf*vx

    # integral error scaled integral gain
    exInt = exInt + ex * AHRS_KI * SENSOR_SAMPLING_TIME
    eyInt = eyInt + ey * AHRS_KI * SENSOR_SAMPLING_TIME
    ezInt = ezInt + ez * AHRS_KI * SENSOR_SAMPLING_TIME

    # adjusted gyroscope measurements
    gxf = gxf + ahrs_kp * ex + exInt
    gyf = gyf + ahrs_kp * ey + eyInt
    gzf = gzf + ahrs_kp * ez + ezInt

    # integrate quaternion rate and normalise
    halfT = 0.5 * SENSOR_SAMPLING_TIME
    q0 = q0 + (-q1*gxf - q2*gyf - q3*gzf)*halfT
    q1 = q1 + (q0*gxf + q2*gzf - q3*gyf)*halfT
    q2 = q2 + (q0*gyf - q1*gzf + q3*gxf)*halfT
    q3 = q3 + (q0*gzf + q1*gyf - q2*gxf)*halfT

    # normalise quaternion
    norm = 1 / math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q0 *= norm
    q1 *= norm
    q2 *= norm
    q3 *= norm

    ahrs->q.q0 = q0
    ahrs->q.q1 = q1
    ahrs->q.q2 = q2
    ahrs->q.q3 = q3
    