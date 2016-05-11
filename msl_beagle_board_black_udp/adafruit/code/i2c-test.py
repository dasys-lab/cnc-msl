import Adafruit_GPIO.FT232H as FT232H
    
# Temporarily disable FTDI serial drivers.
FT232H.use_FT232H()
    
# Find the first FT232H device.
ft232h = FT232H.FT232H()
    
# Create an I2C device at address 0x70.
i2c = FT232H.I2CDevice(ft232h, 0x70)


