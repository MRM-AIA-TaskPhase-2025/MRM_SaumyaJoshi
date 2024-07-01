import cv2
import numpy as np

def nothing(x):
    pass

# Initialize video capture
videoCapture = cv2.VideoCapture(0)  # Change the index if needed (e.g., 1 or 2)

# Create a window for trackbars
cv2.namedWindow("Trackbars")
cv2.createTrackbar("H_MIN", "Trackbars", 21, 179, nothing)
cv2.createTrackbar("H_MAX", "Trackbars", 90, 179, nothing)
cv2.createTrackbar("S_MIN", "Trackbars", 100, 255, nothing)
cv2.createTrackbar("S_MAX", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V_MIN", "Trackbars", 100, 255, nothing)
cv2.createTrackbar("V_MAX", "Trackbars", 255, 255, nothing)

while True:
    ret, frame = videoCapture.read()
    if not ret:
        break

    # Preprocess the frame
    blurredFrame = cv2.GaussianBlur(frame, (15, 15), 0)
    hsvFrame = cv2.cvtColor(blurredFrame, cv2.COLOR_BGR2HSV)

    # Get current positions of trackbars
    h_min = cv2.getTrackbarPos("H_MIN", "Trackbars")
    h_max = cv2.getTrackbarPos("H_MAX", "Trackbars")
    s_min = cv2.getTrackbarPos("S_MIN", "Trackbars")
    s_max = cv2.getTrackbarPos("S_MAX", "Trackbars")
    v_min = cv2.getTrackbarPos("V_MIN", "Trackbars")
    v_max = cv2.getTrackbarPos("V_MAX", "Trackbars")

    # Define the lower and upper bounds for yellowish-green color in HSV space
    lower_yellow = np.array([h_min, s_min, v_min])
    upper_yellow = np.array([h_max, s_max, v_max])

    # Create a mask for yellowish-green color
    mask = cv2.inRange(hsvFrame, lower_yellow, upper_yellow)

    # Apply morphological operations to clean up the mask
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=3)

    # Perform Hough Circle Transform
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                               param1=100, param2=30, minRadius=10, maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # Draw the outer circle
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

    # Display the mask for debugging
    cv2.imshow("Yellow Mask", mask)
    # Display the frame with circles
    cv2.imshow("Detected Tennis Ball", frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all OpenCV windows
videoCapture.release()
cv2.destroyAllWindows()
