from machine import Pin, PWM


class Stepper():
    def __init__(self, stepP, dirP, m1 = None, m2 = None, m3 = None, enP = None):
        self.modeInitState = m1 != None
        self.enInitState = enP != None
        self.stepP = PWM(Pin(stepP))
        self.dirP = Pin(dirP, Pin.OUT)
        if(self.modeInitState): self.modePs = [Pin(m3, Pin.OUT), Pin(m2, Pin.OUT), Pin(m1, Pin.OUT)]
        if(self.enInitState): self.enP = Pin(enP, Pin.OUT)
        
        self.stepP.duty_u16(0)
        self.dirP.value(0)
        if(self.enInitState): self.enP.value(0)
        if(self.modeInitState):
            for mode in self.modePs:
                mode.value(0)
                
        self.currentMode = 0
            
    def lock(self): self.enP.value(0)
    def unlock(self): self.enP.value(1)
            
    def start(self): self.stepP.duty_u16(2**15)
    def stop(self): self.stepP.duty_u16(0)
        
    def setStep(self, step):
        self.dirP.value(step > 0)
        step = abs(int(step))
        if(step > 15):
            self.stepP.freq(step)
            self.start()
            
    def setRPM(self, rpm):
        self.setStep(rpm * 200)
        
    def setMode(self, mode):
        self.currentMode = mode
        mode = bin(min(max(mode, 0), 7))[2:]
        mode = '0' * (3 - len(mode)) + mode
        for m, p in zip(mode, self.modePs):
            p.value(int(m))
        
        