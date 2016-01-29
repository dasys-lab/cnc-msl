from Adafruit_LSM9DS0 import Adafruit_LSM9DS0
import math

imu = Adafruit_LSM9DS0()

pi = 3.14159265358979323846 # Written here to increase performance/ speed
r2d = 57.2957795 # 1 radian in degrees
loop = 0.05 #
tuning = 0.98 # Constant for tuning Complimentary filter

# Converting accelerometer readings to degrees
ax = #x
ay = #y
az = #z

 xAngle = math.atan( ax / ( math.sqrt( ay**2 + az**2 )))
 yAngle = math.atan( ay / ( math.sqrt( ax**2 + az**2 )))
 zAngle = math.atan( sqrt( ax**2 + ay**2 ) / az)
