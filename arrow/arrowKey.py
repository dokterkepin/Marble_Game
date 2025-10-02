import serial
from pynput import keyboard

ser = serial.Serial("/dev/ttyACM0", 115200)

angle = 90     
step = 3        
min_angle = 0
max_angle = 180


def send_angle(a):
    cmd = f"ANGLE:{a}\n"
    ser.write(cmd.encode())

send_angle(angle)
print("Use left/right arrow keys to balance. Press ESC to quit.")

# Key press handling
def on_press(key):
    global angle
    try:
        if key == keyboard.Key.left:
            angle = max(min_angle, angle - step)
            send_angle(angle)
        elif key == keyboard.Key.right:
            angle = min(max_angle, angle + step)
            send_angle(angle)
        elif key == keyboard.Key.esc:
            return False
    except AttributeError:
        pass

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()


ser.close()
