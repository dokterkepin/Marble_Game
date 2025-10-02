import threading
from pynput import keyboard
import serial

# Serial Setup (Windows COM-Port anpassen)
ser = serial.Serial("COM3", 115200)  # Beispiel: COM3

# Servo Setup
servo1_angle = 90
servo2_angle = 90
step = 1
min_angle = 60
max_angle = 105

def send_angles(s1, s2):
    """Send angles to Arduino via serial"""
    cmd = f"S1:{s1};S2:{s2}\n"
    ser.write(cmd.encode())
    print(f"Servo1: {s1}, Servo2: {s2}")

def keyboard_thread():
    global servo1_angle, servo2_angle
    print("Use arrow keys to control servos. ESC to quit.")

    def on_press(key):
        global servo1_angle, servo2_angle
        try:
            if key == keyboard.Key.right:
                servo1_angle = max(min_angle, servo1_angle - step)
            elif key == keyboard.Key.left:
                servo1_angle = min(max_angle, servo1_angle + step)
            elif key == keyboard.Key.up:
                servo2_angle = min(max_angle, servo2_angle + step)
            elif key == keyboard.Key.down:
                servo2_angle = max(min_angle, servo2_angle - step)
            elif key == keyboard.Key.esc:
                send_angles(90, 90)  # Reset to neutral position
                return False
            send_angles(servo1_angle, servo2_angle)
        except AttributeError:
            pass

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

# Start Keyboard Thread
t = threading.Thread(target=keyboard_thread, daemon=True)
t.start()

# Keep main thread alive
try:
    while t.is_alive():
        t.join(0.1)
except KeyboardInterrupt:
    pass

ser.close()
print("Program exited.")
