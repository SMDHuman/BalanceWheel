from machine import Pin, PWM, I2C, UART
from utime import sleep, ticks_ms
from mpu9250 import MPU9250
from math import atan2, degrees, pi
from stepper import Stepper

class PID():
    def __init__(self, p, i, d, target, minOut, maxOut):
        self.targetBias = 0
        self.target = target
        self.pK = p
        self.iK = i
        self.dK = d
        self.error = 0
        self.sumError = 0
        self.lastError = 0
        self.minOut = minOut
        self.maxOut = maxOut
        
    def update(self, now):
        self.error = (self.target + self.targetBias) - now
        self.sumError += self.error
        out = (self.error * self.pK) + (self.sumError * self.iK) + ((self.error - self.lastError) * self.dK)
        self.lastError = self.error
        
        out = self.minOut if out < self.minOut else out
        out = self.maxOut if out > self.maxOut else out
        return(out)
        

accK =.03
lastTime = ticks_ms()
def readAngle(angles):
    global lastTime
    now = ticks_ms()
    gyro = mpu.gyro
    acc = mpu.acceleration
    #angles[0] += (gyro[0] - gyroBias[0])*100000 /16384
    angles[1] -= (gyro[1] - gyroBias[1])*pi
    angles[2] += (gyro[2] - gyroBias[2])*pi
    
    #angles[0] += (degrees(atan2(acc[1], acc[2])) - angles[0]) * accK
    angles[1] += (degrees(atan2(acc[0], acc[2])) - angles[1]) * accK
    angles[2] += (degrees(atan2(acc[0], acc[1])) - angles[2]) * accK
    
    angles[0] = round(angles[0], 2)
    angles[1] = round(angles[1], 2)
    angles[2] = round(angles[2], 2)
    
    lastTime = now
    return(angles)

def sendCMD(cmd, timeout=2000):
    print("CMD: " + cmd)
    bl.write(cmd)
    waitResp(timeout)
    
def waitResp(timeout=2000):
    prvMills = ticks_ms()
    resp = b""
    while (ticks_ms()-prvMills)<timeout:
        if bl.any():
            resp = b"".join([resp, bl.read(1)])
    print(resp)

led = Pin(2, Pin.OUT)
led.value(0)

bl = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

gyroBias = [-0.0235664, 0.003460463, 0.01118344]
i2c = I2C(id = 1, scl = Pin(15), sda = Pin(14))
mpu = MPU9250(i2c)
 
motorR = Stepper(18, 17, 21, 20, 19)
motorL = Stepper(10, 9, 13, 12, 11)

motorR.start()
motorL.start()

motorR.setMode(5)
motorL.setMode(5)

#sendCMD("AT\r\n")
#sendCMD("AT+VERSION\r\n")
#sendCMD("AT+NAMEHC-06")

directions = {'S' : [0, 0], 'F' : [1, 0], 'B' : [-1, 0],
              'R' : [0, 1], 'L' : [0, -1], 'I' : [1, 1],
              'G' : [1, -1], 'J' : [-1, 1], 'H' : [-1, -1]}
numbers = {"0":0, "1":1, "2":2, "3":3, "4":4, "5":5, "6":6, "7":7, "8":8, "9":9, "q":10}

biasAngle = 0
targetAngle = 89
tagretRPM = 0

speed = 40
speedBias = 0

gyroPID = PID(2, 0.3, 0, targetAngle, -100, 100)


rpmPID = PID(.1, 0.0001, .00, tagretRPM, -12, 12)


motorMax = 10
motorMin = -10

moveDir = [0, 0]
angles = [0, 90, 90]
for i in range(20):
    angles = readAngle(angles)
errorSumA = 0
errorOldA = 0
errorSumR = 0
errorOldR = 0
filteredAngles = angles.copy()
f = 0.5
oldTick = ticks_ms()
sliderData = 0
while(1):
    angles = readAngle(angles)
    nowTick = ticks_ms()
    if(nowTick - oldTick > 20):
        #print(rpmPID.target)
        oldTick = nowTick
    filteredAngles[0] = (angles[0] * (1 - f)) + filteredAngles[0] * f
    filteredAngles[1] = (angles[1] * (1 - f)) + filteredAngles[1] * f
    filteredAngles[2] = (angles[2] * (1 - f)) + filteredAngles[2] * f
    #print('a:', angles[1], 'f:', filteredAngles[1], 't:', targetAngle, 'e:', filteredAngles[1] - targetAngle)
    
    if(bl.any()):
        data = bl.read()
        #print(data)
        for d in str(data)[2:-1]:
            moveDir[1] = 0
            if(d == "S"):
                rpmPID.target = 0
                moveDir[1] = 0
            if(d == "F" or d == "I" or d == "G"):
                rpmPID.target = speed
            if(d == "B" or d == "J" or d == "H"):
                rpmPID.target = -speed
            if(d == "L" or d == "G" or d == "H"):
                moveDir[1] = -12
            if(d == "R" or d == "I" or d == "J"):
                moveDir[1] = 12
            if(d in numbers):
                sliderData = numbers[d]
                
        
    if(40 > abs(filteredAngles[1] - targetAngle) > 0 and 1):
        gyroPID.targetBias = rpmPID.update(moveDir[0]) + speedBias
        moveDir[0] = -gyroPID.update(filteredAngles[1])
        #print(moveDir[0])
        #print(moveDir)
        motorL.setRPM(moveDir[0] + moveDir[1])
        motorR.setRPM(moveDir[0] - moveDir[1])
    else:
        motorR.stop()
        motorL.stop()
        
    
    sleep(0.01)
