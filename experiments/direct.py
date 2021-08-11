#! /usr/bin/env python3

import hid

d = hid.device()
d.open(0x0483, 0x5750)
d.set_nonblocking(1)

SIGNATURE               = 0x55  # 85
                                #  2 (did something when sent 6, 1, 2, 3, 4, 5, 6)
CMD_SERVO_MOVE          = 0x03  #  3
CMD_ACTION_GROUP_RUN    = 0x06  #  6 (not-implemented)
CMD_ACTION_STOP         = 0x07  #  7 (not-implemented)
CMD_ACTION_COMPLETE     = 0x08  #  8 (not-implemented)
CMD_ACTION_SPEED        = 0x0b  # 11 (not-implemented)
CMD_GET_BATTERY_VOLTAGE = 0x0f  # 15 (misslabeld in text of document to be 11)
CMD_MULT_SERVO_UNLOAD   = 0x14  # 20 (not-implemented)
CMD_MULT_SERVO_POS_READ = 0x15  # 21 (not-implemented)


def send(cmd, data=[]):
    msg = [
        0,  # no idea
        SIGNATURE, # header
        SIGNATURE, # header
        len(data) + 2, # data length
        cmd, # command
    ]
    if len(data) > 0:
        msg.extend(data) # parameters
    print('SEND', msg)
    d.write(msg)


def voltage():
    send(CMD_GET_BATTERY_VOLTAGE)
    m = d.read(64, 50)  # max_length, timeout?
    assert len(m) >= 6, m
    assert m[0] == SIGNATURE, m
    assert m[1] == SIGNATURE, m
    assert m[3] == CMD_GET_BATTERY_VOLTAGE, m
    assert m[2] == 4, m
    return (m[5] * 256 + m[4]) / 1000.0


def move(servo, position, duration=1000):
    data = [
        1,  # number of servos to be controlled
        duration >> 0 & 0xff,  # lower 8 bits of time
        duration >> 8 & 0xff,  # higher 8 bits of time
        servo, # servo ID
        position >> 0 & 0xff, # lower 8 bits of angle
        position >> 8 & 0xff,  # higher 8 bits of angle
        # repeat servo, low + high for however many servos
    ]
    send(CMD_SERVO_MOVE, bytearray(data))


# def read(cmd):
#     data = [
#         6,  # number of servos to be reading
#         1, 2, 3, 4, 5, 6, # servo index
#     ]
#     send(cmd, data)
#     return d.read(128, 50)


import code
code.interact(local=locals())
