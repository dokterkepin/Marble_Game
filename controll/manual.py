from pynput import keyboard
import serial

ser = serial.Serial("/dev/ttyACM0", 115200)  

# Servo Setup
servo1_angle = 90
servo2_angle = 90
servo3_angle = 95
servo4_angle = 90

step = 1
min_angle = 60
max_angle = 105

def send_angles(s1, s2, s3, s4):
    cmd = f"S1:{s1};S2:{s2};S3:{s3};S4:{s4}\n"
    ser.write(cmd.encode())
    print(f"Servo1: {s1}, Servo2: {s2}, Servo3: {s3}, Servo4: {s4}")

def on_press(key):
    global servo1_angle, servo2_angle, servo3_angle, servo4_angle
    try:
        if key == keyboard.Key.up:
            servo1_angle = max(min_angle, servo1_angle - step)
        elif key == keyboard.Key.down:
            servo1_angle = min(max_angle, servo1_angle + step)
        elif key == keyboard.Key.right:
            servo2_angle = min(max_angle, servo2_angle + step)
        elif key == keyboard.Key.left:
            servo2_angle = max(min_angle, servo2_angle - step)

        elif key.char == 'w':
            servo3_angle = min(max_angle, servo3_angle + step)
        elif key.char == 's':
            servo3_angle = max(min_angle, servo3_angle - step)
        elif key.char == 'd':
            servo4_angle = min(max_angle, servo4_angle + step)
        elif key.char == 'a':
            servo4_angle = max(min_angle, servo4_angle - step)
        send_angles(servo1_angle, servo2_angle, servo3_angle, servo4_angle)
    except AttributeError:
        pass

def run_manual_loop():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

if __name__ == "__main__":
    run_manual_loop()