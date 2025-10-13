import cv2
import numpy as np

def pid_controller(cx, cy, x, y, kp=0.05):
    x = int(x)
    y = int(y)
    cx = int(cx)
    cy = int(cy)
    error_x = cx - x
    error_y = cy - y
    angle_x = 90 + kp * error_x
    angle_y = 90 - kp * error_y

    angle_x = max(0, min(180, angle_x))
    angle_y = max(0, min(180, angle_y))
    return angle_x, angle_y

def run_detection_loop():
    # ser = serial.Serial("/dev/ttyACM0", 115200)
    # cap = cv2.VideoCapture("/dev/video2") 
    cap = cv2.VideoCapture(0)
    lower_orange = np.array([10, 120, 120])
    upper_orange = np.array([25, 255, 255])

    lower_red = np.array([0, 120, 100])
    upper_red = np.array([20, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

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
            cv2.drawContours(display, ball_contour, -1, (255, 0, 0), 2)

        if maze_contours:
            largest_maze = max(maze_contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest_maze)
            (mx, my), (w, h), maze_angle = rect
            cv2.drawContours(display, maze_contours, -1, (0, 255, 0), 2)
            cv2.putText(display, f"Maze Angle: {maze_angle:.2f} deg", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("video", display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_detection_loop()