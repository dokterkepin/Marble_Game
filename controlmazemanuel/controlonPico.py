from machine import Pin, PWM
import sys

# Setup for two servos
servo1 = PWM(Pin(2))
servo2 = PWM(Pin(3))
servo1.freq(50)
servo2.freq(50)

# Initial angles
angle1 = 90
angle2 = 90

def set_servo_angle(servo, angle):
    """Convert angle (0-180) to PWM duty and apply to servo"""
    min_duty = 1638   # 10% of 16-bit range (approx)
    max_duty = 8192   # 50% of 16-bit range (approx)
    duty = int(min_duty + (max_duty - min_duty) * angle / 180)
    servo.duty_u16(duty)

# Set initial positions
set_servo_angle(servo1, angle1)
set_servo_angle(servo2, angle2)

while True:
    line = sys.stdin.readline().strip()
    
    # Handle servo commands in format S1:angle1;S2:angle2
    if line.startswith("S1:") and ";" in line:
        try:
            parts = line.split(";")
            a1 = int(parts[0].split(":")[1])
            a2 = int(parts[1].split(":")[1])
            
            # Clamp angles
            if 0 <= a1 <= 180:
                angle1 = a1
                set_servo_angle(servo1, angle1)
            if 0 <= a2 <= 180:
                angle2 = a2
                set_servo_angle(servo2, angle2)
            
        except:
            pass
    

