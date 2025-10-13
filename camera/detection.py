import cv2
import numpy as np
import serial
import time

ser = serial.Serial("/dev/ttyACM0", 115200)  

def pid_controller_x(cx, x, kp=0.05, ki=0.0, kd=0.01):
    if not hasattr(pid_controller_x, "prev_error"):
        pid_controller_x.prev_error = 0
        pid_controller_x.integral = 0
        pid_controller_x.prev_time = time.monotonic_ns()

    error_x = cx - x
    current_time = time.monotonic_ns()
    dt = (current_time - pid_controller_x.prev_time) * 1e-9
    if dt <= 0: 
        dt = 1e-9

    pid_controller_x.integral += error_x * dt
    derivative = (error_x - pid_controller_x.prev_error) / dt

    angle_x = 93 - ((kp * error_x) + (ki * pid_controller_x.integral) + (kd * derivative))
    angle_x = int(max(0, min(180, angle_x)))
    
    pid_controller_x.prev_error = error_x
    pid_controller_x.prev_time = current_time
    return angle_x


def pid_controller_y(cy, y, kp=0.05, ki=0.0, kd=0.01):
    if not hasattr(pid_controller_y, "prev_error"):
        pid_controller_y.prev_error = 0
        pid_controller_y.integral = 0 
        pid_controller_y.prev_time = time.monotonic_ns()

    error_y = cy - y
    current_time = time.monotonic_ns()
    dt = (current_time - pid_controller_y.prev_time) * 1e-9
    if dt <= 0: 
        dt = 1e-9

    pid_controller_y.integral += error_y * dt
    derivative = (error_y - pid_controller_y.prev_error) / dt

    angle_y = 95 - ((kp * error_y) + (ki * pid_controller_y.integral) + (kd * derivative))
    angle_y = int(max(0, min(180, angle_y)))
    
    pid_controller_y.prev_error = error_y
    pid_controller_y.prev_time = current_time
    return angle_y

def send_angles(s1, s2, s3, s4):
    cmd = f"S1:{s1};S2:{s2};S3:{s3};S4:{s4}\n"
    ser.write(cmd.encode())

def run_detection_loop():
    # ser = serial.Serial("/dev/ttyACM0", 115200)
    cap = cv2.VideoCapture("/dev/video2") 
    # cap = cv2.VideoCapture(0)
    lower_orange = np.array([10, 120, 120])
    upper_orange = np.array([25, 255, 255])

    # Red Maze
    lower_red = np.array([10, 120, 100])
    upper_red = np.array([20, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([255, 255, 255])

    # Red Marker
    # lower_red = np.array([0, 120, 80])
    # upper_red = np.array([10, 180, 255])

    # lower_red2 = np.array([170, 150, 120])
    # upper_red2 = np.array([180, 255, 255])

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ball_mask = cv2.inRange(hsv, lower_orange, upper_orange)
        maze_mask1 = cv2.inRange(hsv, lower_red, upper_red)
        maze_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        maze_mask = cv2.bitwise_or(maze_mask1, maze_mask2)

        kernel = np.ones((25, 25), np.uint8)
        maze_noise = cv2.morphologyEx(maze_mask, cv2.MORPH_CLOSE, kernel)
        maze_noise = cv2.morphologyEx(maze_noise, cv2.MORPH_OPEN, kernel)
        ball_noise = cv2.morphologyEx(ball_mask, cv2.MORPH_OPEN, kernel)
        ball_noise = cv2.morphologyEx(ball_noise, cv2.MORPH_CLOSE, kernel)

        maze_blur = cv2.medianBlur(maze_noise, 5)
        ball_blur = cv2.GaussianBlur(ball_mask, (5, 5), 0)
        
        ball_contour, _ = cv2.findContours(ball_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        maze_contours, _ = cv2.findContours(maze_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        display = frame.copy()

        if ball_contour:
            largest_ball = max(ball_contour, key=cv2.contourArea)
            (bx, by), radius = cv2.minEnclosingCircle(largest_ball)
            center = (int(bx), int(by))
            radius = int(radius)
            cv2.circle(display, center, radius, (255, 0, 0), 2)

        if maze_contours:
            largest_maze = max(maze_contours, key=cv2.contourArea)
            rect_maze = cv2.minAreaRect(largest_maze)
            (mx, my), (wm, hm), maze_angle = rect_maze
            cv2.drawContours(display, maze_contours, -1, (0, 255, 0), 2)
            # cv2.putText(display, f"Maze Angle: {maze_angle:.2f} deg", (10, 30),
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # print("ball: ", by, bx)
        # print("maze: ", my, mx)
        pid_angle_x = pid_controller_x(mx, bx)
        pid_angle_y = pid_controller_y(my, by)
        send_angles(pid_angle_y, pid_angle_x, 95, 90)

        cv2.imshow("video", display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_detection_loop()
