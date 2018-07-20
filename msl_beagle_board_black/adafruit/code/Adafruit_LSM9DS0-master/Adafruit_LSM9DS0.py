#!/usr/bin/python

# Copyright (c) 2015, Jack Weatherilt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from Adafruit_I2C import Adafruit_I2C
import math

class Adafruit_LSM9DS0(Adafruit_I2C):
    i2c = None

    # The same address is used for both the magnetometer and accelerometer, but
    # each has their own variable, to avoid confusion.
    LSM9DS0_MAG_ADDRESS	    =   0x1D
    LSM9DS0_ACCEL_ADDRESS	=	0x1D
    LSM9DS0_GYRO_ADDRESS    =   0x6B

    #LSM9DS0 gyrometer registers
    LSM9DS0_WHO_AM_I_G	      =	0x0F
    LSM9DS0_CTRL_REG1_G	      =	0x20
    LSM9DS0_CTRL_REG3_G	      =	0x22
    LSM9DS0_CTRL_REG4_G	      =	0x23
    LSM9DS0_OUT_X_L_G	      =	0x28
    LSM9DS0_OUT_X_H_G	      =	0x29
    LSM9DS0_OUT_Y_L_G	      =	0x2A
    LSM9DS0_OUT_Y_H_G	      =	0x2B
    LSM9DS0_OUT_Z_L_G	      =	0x2C
    LSM9DS0_OUT_Z_H_G	      =	0x2D

    # LSM9DS0 temperature addresses
    LSM9DS0_OUT_TEMP_L_XM	  =	0x05
    LSM9DS0_OUT_TEMP_H_XM	  =	0x06

    # Magnetometer addresses
    LSM9DS0_STATUS_REG_M	  =	0x07
    LSM9DS0_OUT_X_L_M         =	0x08
    LSM9DS0_OUT_X_H_M         =	0x09
    LSM9DS0_OUT_Y_L_M         =	0x0A
    LSM9DS0_OUT_Y_H_M         =	0x0B
    LSM9DS0_OUT_Z_L_M         =	0x0C
    LSM9DS0_OUT_Z_H_M         =	0x0D

    # Shared addresses
    LSM9DS0_WHO_AM_I_XM	      =	0x0F
    LSM9DS0_INT_CTRL_REG_M    =	0x12
    LSM9DS0_INT_SRC_REG_M	  =	0x13
    LSM9DS0_CTRL_REG1_XM      =	0x20
    LSM9DS0_CTRL_REG2_XM	  =	0x21
    LSM9DS0_CTRL_REG5_XM	  =	0x24
    LSM9DS0_CTRL_REG6_XM	  =	0x25
    LSM9DS0_CTRL_REG7_XM	  =	0x26

    # Accelerometer addresses
    LSM9DS0_OUT_X_L_A	      =	0x28
    LSM9DS0_OUT_X_H_A	      =	0x29
    LSM9DS0_OUT_Y_L_A	      =	0x2A
    LSM9DS0_OUT_Y_H_A	      =	0x2B
    LSM9DS0_OUT_Z_L_A	      =	0x2C
    LSM9DS0_OUT_Z_H_A	      =	0x2D

    # Various settings included in the Arduino library. I haven't used these,
    # to keep to a default setting for simplicity, however, users can change
    # the settings easily.
    LSM9DS0_ACCELRANGE_2G                = 0b000 << 3
    LSM9DS0_ACCELRANGE_4G                = 0b001 << 3
    LSM9DS0_ACCELRANGE_6G                = 0b010 << 3
    LSM9DS0_ACCELRANGE_8G                = 0b011 << 3
    LSM9DS0_ACCELRANGE_16G               = 0b100 << 3

    LSM9DS0_ACCELDATARATE_POWERDOWN      = 0b0000 << 4
    LSM9DS0_ACCELDATARATE_3_125HZ        = 0b0001 << 4
    LSM9DS0_ACCELDATARATE_6_25HZ         = 0b0010 << 4
    LSM9DS0_ACCELDATARATE_12_5HZ         = 0b0011 << 4
    LSM9DS0_ACCELDATARATE_25HZ           = 0b0100 << 4
    LSM9DS0_ACCELDATARATE_50HZ           = 0b0101 << 4
    LSM9DS0_ACCELDATARATE_100HZ          = 0b0110 << 4
    LSM9DS0_ACCELDATARATE_200HZ          = 0b0111 << 4
    LSM9DS0_ACCELDATARATE_400HZ          = 0b1000 << 4
    LSM9DS0_ACCELDATARATE_800HZ          = 0b1001 << 4
    LSM9DS0_ACCELDATARATE_1600HZ         = 0b1010 << 4

    LSM9DS0_MAGGAIN_2GAUSS               = 0b00 << 5
    LSM9DS0_MAGGAIN_4GAUSS               = 0b01 << 5
    LSM9DS0_MAGGAIN_8GAUSS               = 0b10 << 5
    LSM9DS0_MAGGAIN_12GAUSS              = 0b11 << 5

    LSM9DS0_MAGDATARATE_3_125HZ          = 0b000 << 2
    LSM9DS0_MAGDATARATE_6_25HZ           = 0b001 << 2
    LSM9DS0_MAGDATARATE_12_5HZ           = 0b010 << 2
    LSM9DS0_MAGDATARATE_25HZ             = 0b011 << 2
    LSM9DS0_MAGDATARATE_50HZ             = 0b100 << 2
    LSM9DS0_MAGDATARATE_100HZ            = 0b101 << 2

    LSM9DS0_GYROSCALE_245DPS             = 0b00 << 4
    LSM9DS0_GYROSCALE_500DPS             = 0b01 << 4
    LSM9DS0_GYROSCALE_2000DPS            = 0b10 << 4

    # Debug set to true for the moment, to find bugs
    def __init__(self, busnum=-1, debug=False):
        # Each feature is given a call name. Although The magnetometer and
        # accelerometer use the same address, they've been given different
        # names for clarity.
        self.mag    = Adafruit_I2C(self.LSM9DS0_MAG_ADDRESS, busnum, debug)
        self.accel  = Adafruit_I2C(self.LSM9DS0_ACCEL_ADDRESS, busnum, debug)
        self.gyro   = Adafruit_I2C(self.LSM9DS0_GYRO_ADDRESS, busnum, debug)

        # Magnetometer initialisation
        self.mag.write8(self.LSM9DS0_CTRL_REG5_XM, 0b11110000) # Temperature sensor enabled, high res mag, 50Hz
        self.mag.write8(self.LSM9DS0_CTRL_REG6_XM, 0b01100000) # +/- 12 gauss
        self.mag.write8(self.LSM9DS0_CTRL_REG7_XM, 0b00000000) # Normal mode, continuous-conversion mode

        # Accelerometer initialisation
        self.accel.write8(self.LSM9DS0_CTRL_REG1_XM, 0b01100111) # 100Hz, XYZ enabled
        self.accel.write8(self.LSM9DS0_CTRL_REG2_XM, 0b00100000) # +/- 16 g

        # Gyro initialisation
        self.gyro.write8(self.LSM9DS0_CTRL_REG1_G, 0b00001111) # Normal power mode, XYZ enabled
        self.gyro.write8(self.LSM9DS0_CTRL_REG4_G, 0b00110000) # Continuous update, 2000 dps

    def rawAccel(self):
        # Reading each induvidual byte is the simplest method of getting data,
        # however, it could cause inaccuracy due to values changing during readings
        # of different registers. A filter should be employed in the fusing of the
        # data taken
        accelX = self.accel.readU8(self.LSM9DS0_OUT_X_L_A) | self.accel.readU8(self.LSM9DS0_OUT_X_H_A) << 8
        accelY = self.accel.readU8(self.LSM9DS0_OUT_Y_L_A) | self.accel.readU8(self.LSM9DS0_OUT_Y_H_A) << 8
        accelZ = self.accel.readU8(self.LSM9DS0_OUT_Z_L_A) | self.accel.readU8(self.LSM9DS0_OUT_Z_H_A) << 8

        accelArr = [accelX, accelY, accelZ]

        # Values are signed and therefore must be checked
        for i in range(len(accelArr)):
            if accelArr[i] > 32767:
                accelArr[i] -= 65536

        return accelArr

    def rawMag(self):
        magX = self.mag.readU8(self.LSM9DS0_OUT_X_L_M) | self.mag.readU8(self.LSM9DS0_OUT_X_H_M) << 8
        magY = self.mag.readU8(self.LSM9DS0_OUT_Y_L_M) | self.mag.readU8(self.LSM9DS0_OUT_Y_H_M) << 8
        magZ = self.mag.readU8(self.LSM9DS0_OUT_Z_L_M) | self.mag.readU8(self.LSM9DS0_OUT_Z_H_M) << 8

        magArr = [magX, magY, magZ]

        for i in range(len(magArr)):
            if magArr[i] > 32767:
                magArr[i] -= 65536

        return magArr

    def rawGyro(self):
        gyroX = self.gyro.readU8(self.LSM9DS0_OUT_X_L_G) | self.gyro.readU8(self.LSM9DS0_OUT_X_H_G) << 8
        gyroY = self.gyro.readU8(self.LSM9DS0_OUT_Y_L_G) | self.gyro.readU8(self.LSM9DS0_OUT_Y_H_G) << 8
        gyroZ = self.gyro.readU8(self.LSM9DS0_OUT_Z_L_G) | self.gyro.readU8(self.LSM9DS0_OUT_Z_H_G) << 8

        gyroArr = [gyroX, gyroY, gyroZ]

        for i in range(len(gyroArr)):
            if gyroArr[i] > 32767:
                gyroArr[i] -= 65536

        return gyroArr

    # Read all the XYZ values from each sensor and fuse into one 2D array
    def rawAll(self):
        allData = []
        allData.append(self.rawAccel())
        allData.append(self.rawMag())
        allData.append(self.rawGyro())

        return allData

    # The documentation on reading temperature is not very clear, and it appears
    # that the sensor does not provide an ambient temperature reading, with no
    # absolute value, instead measuring change in temp inside the chip
    def rawTemp(self):
        temp = self.mag.readList(self.LSM9DS0_OUT_TEMP_L_XM) | self.mag.readList(self.LSM9DS0_OUT_TEMP_H_XM) << 8

        return temp
