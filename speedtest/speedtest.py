import cv2
import numpy as np
import time

# Open camera (Laptop camera = 0)
cap = cv2.VideoCapture(0)

# HSV color range for the ball (adjust if needed)
lower_ball = np.array([0, 60, 230])
upper_ball = np.array([20, 140, 255])

# Previous values for velocity calculation
prev_x, prev_y, prev_time = None, None, None

# Pixel-to-meter scaling (calibration required)
scale = 0.0005  # Example: plate 30 cm wide / image width 600 px = 0.0005 m/px

# Sliding window for velocity smoothing
window_size = 5
vx_window = []
vy_window = []

# Morphological kernel for noise removal
kernel = np.ones((5,5), np.uint8)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to HSV and create mask
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_ball, upper_ball)

    # Remove small noise and close gaps
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    output_frame = frame.copy()

    if contours:
        # Take the largest contour (assume it's the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # Accept only if area is reasonable (ignore noise/reflections)
        if 1000 < area < 7000:
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                radius = int(np.sqrt(area / np.pi))

                # Draw ball
                cv2.circle(output_frame, (cx, cy), radius, (0, 255, 0), 2)
                cv2.circle(output_frame, (cx, cy), 2, (0, 0, 255), 3)

                # Current timestamp
                current_time = time.time()
                if prev_x is not None and prev_time is not None:
                    dt = current_time - prev_time
                    if dt > 0:
                        # Compute raw velocity
                        vx = (cx - prev_x) * scale / dt
                        vy = (cy - prev_y) * scale / dt

                        # Sliding window smoothing
                        vx_window.append(vx)
                        vy_window.append(vy)
                        if len(vx_window) > window_size:
                            vx_window.pop(0)
                            vy_window.pop(0)

                        # Average velocity
                        vx_avg = sum(vx_window) / len(vx_window)
                        vy_avg = sum(vy_window) / len(vy_window)

                        print(f"Ball at ({cx},{cy}), "
                              f"v_raw=({vx:.3f},{vy:.3f}) m/s, "
                              f"v_avg=({vx_avg:.3f},{vy_avg:.3f}) m/s")

                # Store current values for next iteration
                prev_x, prev_y, prev_time = cx, cy, current_time

    # Show output frame
    cv2.imshow('Ball Detection + Velocity', output_frame)

    # Quit with 'q'
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
