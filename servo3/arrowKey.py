import serial
import keyboard
import time

# Adjust the COM port (check in Device Manager!)
ser = serial.Serial("COM3", 115200) # type: ignore

angle = 90  # Start in the middle position
step = 3    # How many degrees to change per key press
min_angle = 60   # Minimum angle to prevent the platform from tilting too much
max_angle = 120  # Maximum angle to prevent over-tilting

# Function to send the angle command to the servo via serial
def send_angle(a):
    cmd = f"ANGLE:{a}\n"
    ser.write(cmd.encode())

# Send initial angle to servo
send_angle(angle)

print("Use left/right arrow keys to balance. Press ESC to quit.")

try:
    while True:
        if keyboard.is_pressed("left"):
            # Decrease angle, but don't go below minimum
            angle = max(min_angle, angle - step)
            send_angle(angle)
            time.sleep(0.05)
        elif keyboard.is_pressed("right"):
            # Increase angle, but don't exceed maximum
            angle = min(max_angle, angle + step)
            send_angle(angle)
            time.sleep(0.05)
        elif keyboard.is_pressed("esc"):
            # Exit the loop if ESC is pressed
            break
finally:
    # Close the serial connection when done
    ser.close()
