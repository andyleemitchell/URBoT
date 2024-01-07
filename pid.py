class PID:
    MANUAL = 0
    AUTOMATIC = 1

    DIRECT = 0
    REVERSE = 1

    # sampleTime is in milliseconds!!!
    def __init__(self, setpoint, outMin, outMax, sampleTime, kp=0, ki=0, kd=0):
        #self.input = input
        self.setpoint = setpoint
        self.outMin = outMin
        self.outMax = outMax
        self.sampleTime = sampleTime
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integralTerm = 0
        self.lastInput = 0
        self.inAuto = True
        self.controllerDirection = self.DIRECT

        self.prevOutput = 0

    def compute(self, input):
        if (not self.inAuto):
            return

        error = self.setpoint - input
        self.integralTerm += (self.ki * error)
        # windup check on integral term
        if (self.integralTerm > self.outMax):
            self.integralTerm = self.outMax
        elif (self.integralTerm < self.outMin):
            self.integralTerm = self.outMin

        dInput = input - self.lastInput

        output = self.kp * error + self.integralTerm + self.kd * dInput
        # windup check on output
        if (output > self.outMax):
            output = self.outMax
        elif (output < self.outMin):
            output = self.outMin

        self.lastInput = input

        self.prevOutput = output

        return output

    def setTunings(self, kp, ki, kd):
        if (kp < 0 or ki < 0 or kd < 0):
            return

        sampleTimeInSec = self.sampleTime / 1000
        self.kp = kp
        self.ki = ki * sampleTimeInSec
        self.kd = kd / sampleTimeInSec

        if (self.controllerDirection is self.REVERSE):
            self.kp = (0 - self.kp)
            self.ki = (0 - self.ki)
            self.kd = (0 - self.kd)

    def setSampleTime(self, newSampleTime):
        if (newSampleTime <= 0):
            return

        ratio = newSampleTime / self.sampleTime
        self.ki * ratio
        self.kd / ratio
        self.sampleTime = newSampleTime

    def setOutputLimits(self, min, max):
        if (min > max):
            return

        self.outMin = min
        self.outMax = max

        if (self.integralTerm > self.outMax):
            self.integralTerm = self.outMax
        elif (self.integralTerm < self.outMin):
            self.integralTerm = self.outMin

    def setMode(self, mode):
        newAuto = (mode == self.AUTOMATIC)
        if (newAuto == (not self.inAuto)):
            self.initialise()
        self.inAuto = newAuto

    def initialise(self):
        self.lastInput = self.input
        # not sure if this is right or not
        self.integralTerm = self.prevOutput
        if (self.integralTerm > self.outMax):
            self.integralTerm = self.outMax
        elif (self.integralTerm < self.outMin):
            self.integralTerm = self.outMin

    def setControllerDirection(self, direction):
        self.controllerDirection = direction
