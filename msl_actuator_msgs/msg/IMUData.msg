
    

#Magnetic Compass: X,Y,Z [raw data]
msl_msgs/Point3dInfo magnet

#Magnetic Sensivity: Magnetic Compass.(X|Y|Z) * Magnetic Sensitivity = [MilliGauss]
float32 magnetSens

#Accelaration: X,Y, Z [m/s²]
msl_msgs/Point3dInfo acceleration

#Accelaration Sensivity:
float32 accelSens

#Gyro: X,Y,Z [°/s]
msl_msgs/Point3dInfo gyro

#Gyro Sensivity: 
float32 gyroSens

#Temperature: [°C]
uint16 temperature

#Timestamp: probably [ns]
uint64 time

