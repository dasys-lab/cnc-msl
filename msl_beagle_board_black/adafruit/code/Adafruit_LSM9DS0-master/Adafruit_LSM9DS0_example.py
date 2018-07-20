from Adafruit_LSM9DS0 import Adafruit_LSM9DS0

imu = Adafruit_LSM9DS0()

while True:
    # Get all data
    ACC = imu.rawAccel()
    GYR = imu.rawGyro()
    MAG = imu.rawMag()

    # Convert to strings for display
    strACC = ', '.join(str(num) for num in ACC)
    strGYR = ', '.join(str(num) for num in GYR)
    strMAG = ', '.join(str(num) for num in MAG)

    # Display in a pretty way
    print "Accel: " + strACC + " | Gyro: " + strGYR + " | Mag: " + strMAG
