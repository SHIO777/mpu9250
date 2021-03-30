
def a(a):
    if a == 'a':
        print('ok!')
    else:
        raise Exception('a parameter "%s" is not defined' % a)


# a('b')

a = 0x00
b = 0x10
# print(b)
# print(b << 2)
# 0x10 = 0d16 = 0b10000
# 0b10000 << 8 = 0b 1 0000 0000 0000 = 0d2^12 = 0d4096

# print(bin(0x01 << 15))

unsigned_data = -32
d = unsigned_data & (0x01 << 15)
# print(bin(d))
if d:
    print('yes')

# print(bin(-1))
# print(bin(0xffff))


def u2s(unsigned_number):
    if unsigned_number & (0x01 << 15):
        value = -1 * ((unsigned_number ^ 0xffff) + 1)
        print(bin(unsigned_number))
        print(bin(value))
        print(value)
        return value
    return unsigned_number

u2s(-1)
