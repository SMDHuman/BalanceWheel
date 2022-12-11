from mpu9250 import MPU9250
from machine import Pin, I2C

i2c = I2C(id = 1, scl = Pin(15), sda = Pin(14))
mpu = MPU9250(i2c)

n = 300

rawData = [mpu.gyro for i in range(n)]

calData = [sum([rawData[i][0] for i in range(n)]) / n,
           sum([rawData[i][1] for i in range(n)]) / n,
           sum([rawData[i][2] for i in range(n)]) / n]

print(calData)