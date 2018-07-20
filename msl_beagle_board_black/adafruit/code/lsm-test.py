import Adafruit_GPIO.FT232H as FT232H
import time
import numpy
     
# Temporarily disable FTDI serial drivers.
FT232H.use_FT232H()
    
# Find the first FT232H device.
ft232h = FT232H.FT232H()
   
# Create an I2C device at address 0x70.
imuxm = FT232H.I2CDevice(ft232h, 0x1D)
imug = FT232H.I2CDevice(ft232h, 0x6B)

whoami = imuxm.readU8(0x0F)
print "WhoAmI: " + str(whoami)

#init?

imuxm.write8(0x1F, 0x00) #reg0: 
imuxm.write8(0x20, 0x57) #reg1:
imuxm.write8(0x21, 0x00) #reg2: 
imuxm.write8(0x22, 0x04) #reg3: 
imuxm.write8(0x23, 0x04) #reg4: 
imuxm.write8(0x24, 0x14) #reg5:
imuxm.write8(0x25, 0x00) #reg6: 
imuxm.write8(0x26, 0x00) #reg7: 

imuxm.write8(0x12, 0x09) # int reg


ACCx1 = imuxm.readU8(0x28)
ACCx2 = imuxm.readU8(0x29)
ACCx = imuxm.readS16(0x28)

ACCx2 << 8
num = numpy.int16(ACCx1 + ACCx2)

print "Zusammen gepackt: " + str(num)
print ACCx

time.sleep(2)

while True:
#	ACCx = imuxm.readS16(0x28)
#	ACCy = imuxm.readS16(0x2A)
#	ACCz = imuxm.readS16(0x2C)
#	MAGx = imuxm.readS16(0x08)
#	MAGy = imuxm.readS16(0x0A)
#	MAGz = imuxm.readS16(0x0C)
#	TEMP = imuxm.readS16(0x05)
	
	ACCx1 = imuxm.readU8(0x28)
	ACCx2 = imuxm.readU8(0x29) << 8
	
	num = numpy.int16(ACCx1 + ACCx2)
	print "Zusammen gepackt: " + str(num)

#	print "ACC: " + str(ACCx) + ", " + str(ACCy) + ", " + str(ACCz)
#	print "MAG: " + str(MAGx) + ", " + str(MAGy) + ", " + str(MAGz)
#	print "TEMP: " + str(TEMP)
	time.sleep(.500)
