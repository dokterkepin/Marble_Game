import cv2
import numpy as np
import serial
import threading
from pynput import keyboard

ser = serial.Serial("/dev/ttyACM0", 115200)

# Camera Setup
cap = cv2.VideoCapture(0) 
lower_orange = np.array([10, 120, 120])
upper_orange = np.array([25, 255, 255])

# Control Setup
angle = 90
step = 3
min_angle = 0
max_angle = 180

def send_angle(a):
    cmd = f"ANGLE:{a}\n"
    #ser.write(cmd.encode())
    print(f"send angle: {a}")

def keyboard_thread():
    global angle
    print("Use left/right arrow keys to balance. Press ESC to quit.")
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
                cap.release()
                cv2.destroyAllWindows()
                return False
        except AttributeError:
            pass
    
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

t = threading.Thread(target=keyboard_thread, daemon=True)
t.start()

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    f = cv2.GaussianBlur(mask, (9, 9), 2, 2)
    edges = cv2.Canny(f, 50, 150)

    circles = cv2.HoughCircles(
        edges,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=20,
        param1=50,
        param2=25,      
        minRadius=10,   
        maxRadius=100
    )
    output = frame.copy()
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            cv2.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 2)  
            cv2.circle(output, (i[0], i[1]), 2, (0, 0, 255), 3)   
            x, y, r = i
            msg = f"POS: {x},{y}\n"
            ser.write(msg.encode())
            print(f"Ball at ({i[0]}, {i[1]}), radius {i[2]}")

    cv2.imshow('Detected Circle', output)
    if cv2.waitKey(1) == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
cv2.imshow('Detected Circle', output)
ser.close()
