from machine import Pin, PWM
from time import sleep

class TB6612FNG():
    speed_min = 0
    speed_max = 100
    duty_min = 0
    duty_max = 65535
    
    def __init__(self, STBY, in1, in2, pwm):
        self.STBY = Pin(STBY, Pin.OUT)
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.pwm = PWM(Pin(pwm), freq=20_000, duty_u16=0)
        self.current_speed = 0
        # stop motor at init
        self.in1.off()
        self.in2.off()
        self.STBY.on()
#         print("__init__ done")
        
    def set_speed(self, speed):
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
        self.current_speed = self.map_speed(speed)
        self.pwm.duty_u16(self.current_speed)
#         print("speed set to: ", self.current_speed, " from: ", speed)
        
    def map_speed(self, x):
        return (x - self.speed_min) * (self.duty_max - self.duty_min) // (self.speed_max - self.speed_min) + self.duty_min
    
    def forward(self):
        self.STBY.on()
        self.in1.on()
        self.in2.off()
        self.pwm.duty_u16(self.current_speed)
#         print("going forward")
    
    def backward(self):
        self.STBY.on()
        self.in1.off()
        self.in2.on()
        self.pwm.duty_u16(self.current_speed)
#         print("going backward")
        
    def standby(self):
        self.STBY.off()
#         print("going into standby")
        
    def ramp_up(self, forward=True):
        print("ramping up")
        for i in range(50, 101, 2):
            self.set_speed(i)
            self.forward() if forward else self.backward()
            sleep(0.05)
    
    def ramp_down(self, forward=True):
        print("ramping down")
        for i in range(101, 50, -2):
            self.set_speed(i)
            self.forward() if forward else self.backward()
            sleep(0.05)