from machine import Pin, PWM
import sys

servo = PWM(Pin(2))
servo.freq(50)

# Startwinkel
winkel = 90

def set_servo_angle(angle):
    min_duty = 1638
    max_duty = 8192
    duty = int(min_duty + (max_duty - min_duty) * angle / 180)
    servo.duty_u16(duty)

set_servo_angle(winkel)

while True:
    line = sys.stdin.readline().strip()
    print(line)
    if line.startswith("ANGLE:"):
        try:
            winkel = int(line.split(":")[1])
            print(winkel)
            if 0 <= winkel <= 180:
                set_servo_angle(winkel)
        except:
            pass
    elif line.startswith("POSE:"):
        try:
            coordinates = int(line.split(":")[1])
            print(coordinates)
        except:
            pass
