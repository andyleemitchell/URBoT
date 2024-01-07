from machine import I2C, Pin, Timer
from math import sqrt, atan2, pi, copysign, sin, cos
from mpu9250 import MPU9250
from TB6612FNG import TB6612FNG
from time import sleep, ticks_ms, ticks_diff
from pid import PID

# setup motors
motor1 = TB6612FNG(STBY=4, in1=2, in2=3, pwm=0)
motor2 = TB6612FNG(STBY=4, in1=7, in2=8, pwm=9)

motor1.set_speed(100)
motor2.set_speed(100)


MPU = 0x68
id = 1
sda = Pin(14)
scl = Pin(15)
led = Pin("LED", Pin.OUT)
led.value(0)

# create the I2C
i2c = I2C(id=id, scl=scl, sda=sda)

# Scan the bus
print(i2c.scan())
m = MPU9250(i2c)

m.ak8963.calibrate(count=10)
pitch_bias = 0.0
roll_bias = 0.0

# For low pass filtering
filtered_x_value = 0.0 
filtered_y_value = 0.0

def get_reading()->float:
    ''' Returns the readings from the sensor '''
    global filtered_y_value, filtered_x_value
    x = m.acceleration[0] 
    y = m.acceleration[1]
    z = m.acceleration[2] 

    # Pitch and Roll in Radians
    roll_rad = atan2(-x, sqrt((z*z)+(y*y)))
    pitch_rad = atan2(z, copysign(y,y)*sqrt((0.01*x*x)+(y*y)))

    # Pitch and Roll in Degrees
    pitch = pitch_rad*180/pi
    roll = roll_rad*180/pi

    # Get soft_iron adjusted values from the magnetometer
    mag_x, mag_y, magz = m.magnetic

    filtered_x_value = low_pass_filter(mag_x, filtered_x_value)
    filtered_y_value = low_pass_filter(mag_y, filtered_y_value)

    az =  90 - atan2(filtered_y_value, filtered_x_value) * 180 / pi

    # make sure the angle is always positive, and between 0 and 360 degrees
    if az < 0:
        az += 360
        
    # Adjust for original bias
    pitch -= pitch_bias
    roll -= roll_bias

    heading = 0

    return x, y, z, pitch, roll, az, heading

def low_pass_filter(raw_value:float, remembered_value):
    ''' Only applied 20% of the raw value to the filtered value '''
    
    # global filtered_value
    alpha = 0.8
    filtered = 0
    filtered = (alpha * remembered_value) + (1.0 - alpha) * raw_value
    return filtered

def show():
    ''' Shows the Pitch, Rool and heading '''
    global setpoint, error, errSum, lastErr, lastTime, kp, ki, kd
    
    x, y, z, pitch, roll, az, heading_value = get_reading()
    print("Pitch",round(pitch,1), "Roll",round(roll, 1), "compass", az,"Heading", heading_value)
    
#     speed = output
    
# #     speed = abs(10*round(pitch,1))
#     motor1.set_speed(int(abs(speed)))
#     motor2.set_speed(int(abs(speed)))
#     if int(speed) > 0:
#         motor1.forward()
#         motor2.forward()
#     else:
#         motor1.backward()
#         motor2.backward()
#     sleep(0.2)

def doPID():
    x, y, z, pitch, roll, az, heading_value = get_reading()
    speed = pid.compute(pitch)
#     speed = abs(10*round(pitch,1))
    motor1.set_speed(int(abs(speed)))
    motor2.set_speed(int(abs(speed)))
    if int(speed) > 0:
        motor1.forward()
        motor2.forward()
    else:
        motor1.backward()
        motor2.backward()


setpoint = 0
kp, ki, kd = 1, 0, 0
sampleTime = 100 # ms
                            
# reset orientation to zero
x,y,z, pitch_bias, roll_bias, az, az_raw = get_reading()

# setup PID
pid = PID(setpoint=setpoint, 
          outMin=0, outMax=100, 
          sampleTime=sampleTime,
          kp=kp, ki=ki, kd=kd)

# setup timer
timer = Timer(period=sampleTime, mode=Timer.PERIODIC, callback=doPID)

# main loop
# the timer interrupt will handle all of the PID calculations
while True:
    continue