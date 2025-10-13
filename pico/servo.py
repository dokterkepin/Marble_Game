from machine import Pin, PWM
import sys

# Setup for two servos
servo1 = PWM(Pin(1))
servo2 = PWM(Pin(2))
servo3 = PWM(Pin(3))
servo4 = PWM(Pin(4))

servo1.freq(50)
servo2.freq(50)
servo3.freq(50)
servo4.freq(50)

# Initial angles
angle1 = 90
angle2 = 90
angle3 = 90
angle4 = 90

def set_servo_angle(servo, angle):
    """Convert angle (0-180) to PWM duty and apply to servo"""
    min_duty = 1638   
    max_duty = 8192
    duty = int(min_duty + (max_duty - min_duty) * angle / 180)
    servo.duty_u16(duty)

set_servo_angle(servo1, angle1)
set_servo_angle(servo2, angle2)
set_servo_angle(servo3, angle3)
set_servo_angle(servo4, angle4)

while True:
    line = sys.stdin.readline().strip()
    if line.startswith("S1:") and ";" in line:
        try:
            parts = line.split(";")
            a1 = int(parts[0].split(":")[1])
            a2 = int(parts[1].split(":")[1])
            a3 = int(parts[2].split(":")[1])
            a4 = int(parts[3].split(":")[1])
            print(a3, a4)
            if 0 <= a1 <= 180:
                angle1 = a1
                set_servo_angle(servo1, angle1)
            if 0 <= a2 <= 180:
                angle2 = a2
                set_servo_angle(servo2, angle2)
            if 0 <= a3 <= 180:
                angle3 = a3
                set_servo_angle(servo3, angle3)
            if 0 <= a4 <= 180:
                angle4 = a4
                set_servo_angle(servo4, angle4)
        except:
            pass
    
