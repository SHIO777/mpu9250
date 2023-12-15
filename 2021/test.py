
# def a(a):
#     if a == 'a':
#         print('ok!')
#     else:
#         raise Exception('a parameter "%s" is not defined' % a)


# # a('b')

# a = 0x00
# b = 0x10
# # print(b)
# # print(b << 2)
# # 0x10 = 0d16 = 0b10000
# # 0b10000 << 8 = 0b 1 0000 0000 0000 = 0d2^12 = 0d4096

# # print(bin(0x01 << 15))

# unsigned_data = -32
# d = unsigned_data & (0x01 << 15)
# # print(bin(d))
# if d:
#     print('yes')

# # print(bin(-1))
# # print(bin(0xffff))


# def u2s(unsigned_number):
#     if unsigned_number & (0x01 << 15):
#         value = -1 * ((unsigned_number ^ 0xffff) + 1)
#         print(bin(unsigned_number))
#         print(bin(value))
#         print(value)
#         return value
#     return unsigned_number

# u2s(-1)


# print(2**-0.5)


"""enumerate関数について"""

# imu_devs   = ["ACCELEROMETER","GYROSCOPE","MAGNETOMETER"]
# imu_labels = ["x-dir","y-dir","z-dir"]
# imu_units  = ["g","g","g","dps","dps","dps","uT","uT","uT"]
# imu_values = []
# for i in range(0, 9):
#     imu_values.append(i+10)
# # print(imu_values)


# for i, imu in enumerate(imu_values):
#     print("{0}: {1:.2f} {2}".format(imu_labels[i%3], imu, imu_units[i]))


"""データのキャリブレーションについて"""

# _sum = [0, 0, 0]
# _data = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
# _a = []

# for i in range(10):
#     _sum[0] += _data[i]
#     _sum[1] += _data[i] * 2
#     _a.append(i)

# print(_sum)
# print(_a)

# import time, datetime
# print(time.time())
# print(datetime.datetime.now())


"""appendについて"""

a = []

b = [1, 2, 3]
c = [4, 5, 6]
d = [7, 8, 9]
a.append(b)
a.append(c)
a.append(d)

print(a)
# print(len(a))

print(a[0][1])