import math, serial, time
import numpy as np

OP_SYNC = 0x10
OP_DELAY = 0x20
OP_DRIVE = 0x30
OP_DRIVE_REV_L = 0x01
OP_DRIVE_REV_R = 0x02
OP_SENSE_RAW = 0x40
OP_SENSE_RAW_SONAR = 0x01
OP_SENSE_RAW_ACCEL = 0x02
OP_SENSE_RAW_GYRO = 0x04
OP_SENSE_RAW_MAG = 0x08

SONAR_OFFSET = 30

def flag(c, flag):
    if c: return flag
    else: return 0

class Robot(serial.Serial):
    def __init__(self, device = '/dev/ttyUSB0', baudrate = 115200):
        serial.Serial.__init__(self, device, baudrate = baudrate)
        time.sleep(1)
        self.sonar = None
        self.accel = np.empty([3])
        self.gyro = np.empty([3])
        self.mag = np.empty([3])

    def delay(self, t):
        t = int(1000 * t)
        buf = bytearray(3)
        while t > 0xffff:
            self.delay(0xffff)
            t -= 0xffff
        buf[0] = OP_DELAY
        buf[1] = t >> 8
        buf[2] = t & 0xff
        self.write(buf)

    def drive(self, vL, vR):
        buf = bytearray(3)
        buf[0] = OP_DRIVE | flag(vL < 0, OP_DRIVE_REV_L) | flag(vR < 0, OP_DRIVE_REV_R)
        buf[1] = int(255 * abs(vL))
        buf[2] = int(255 * abs(vR))
        self.write(buf)

    def _read_uint16(self):
        r = self.read(2)
        return (r[0] << 8) | r[1]

    def _read_int16_triple(self, buf):
        r = self.read(6)
        for i in range(3):
            x = (r[2*i] << 8) | r[2*i + 1]
            if x < 0x8000: buf[i] = x
            else:          buf[i] = x - 0x10000

    def sense(self, sonar = False, accel = False, gyro = False, mag = False):
        buf = bytearray(1)
        buf[0] = OP_SENSE_RAW \
            | flag(sonar, OP_SENSE_RAW_SONAR) \
            | flag(accel, OP_SENSE_RAW_ACCEL) \
            | flag(gyro, OP_SENSE_RAW_GYRO) \
            | flag(mag, OP_SENSE_RAW_MAG)
        self.write(buf)
        if sonar: self.sonar = self._read_uint16()
        if accel: self._read_int16_triple(self.accel)
        if gyro:  self._read_int16_triple(self.gyro)
        if mag:   self._read_int16_triple(self.mag)


"""
Make a function to turn the robot to a given absolute angle.
Make a function to drive in a straight line. Do you expect the robot to go straight be giving the same power to the two motors?
Drive the function in an equilateral triangle, attempting to get back to the starting point.
Make a function which learns its position by measuring the distances to two walls, possibly driving up close enough first.
Make a function which given an initial unknown location can drive to a provided location.
Make a function which can locate a box.
Make a function which can locate two boxes and park between them.
"""

class Controller():
    def __init__(self, robot):
        self.robot=robot
        pass

    def run(self):
        pass

if __name__=='__main__':
    robot = Robot()
    controller = Controller(robot)
