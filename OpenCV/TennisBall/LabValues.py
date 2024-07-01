import cv2
import numpy as np

def nothing(x):
    pass

# Initialize video capture
videoCapture = cv2.VideoCapture(0)  # Change the index if needed (e.g., 1 or 2)

# Create a window for trackbars
cv2.namedWindow("Trackbars")
cv2.createTrackbar("L_MIN", "Trackbars", 83, 255, nothing)
cv2.createTrackbar("L_MAX", "Trackbars", 246, 255, nothing)
cv2.createTrackbar("A_MIN", "Trackbars", 43, 255, nothing)
cv2.createTrackbar("A_MAX", "Trackbars", 144, 255, nothing)
cv2.createTrackbar("B_MIN", "Trackbars", 153, 255, nothing)
cv2.createTrackbar("B_MAX", "Trackbars", 221, 255, nothing)

while True:
    ret, frame = videoCapture.read()
    if not ret:
        break

    # Preprocess the frame
    blurredFrame = cv2.GaussianBlur(frame, (15, 15), 0)
    labFrame = cv2.cvtColor(blurredFrame, cv2.COLOR_BGR2Lab)

    # Get current positions of trackbars
    l_min = cv2.getTrackbarPos("L_MIN", "Trackbars")
    l_max = cv2.getTrackbarPos("L_MAX", "Trackbars")
    a_min = cv2.getTrackbarPos("A_MIN", "Trackbars")
    a_max = cv2.getTrackbarPos("A_MAX", "Trackbars")
    b_min = cv2.getTrackbarPos("B_MIN", "Trackbars")
    b_max = cv2.getTrackbarPos("B_MAX", "Trackbars")

    # Define the lower and upper bounds for yellowish-green color in LAB space
    lower_yellow = np.array([l_min, a_min, b_min])
    upper_yellow = np.array([l_max, a_max, b_max])

    # Create a mask for yellowish-green color
    mask = cv2.inRange(labFrame, lower_yellow, upper_yellow)

    # Apply morphological operations to clean up the mask
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=3)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw contours and apply HoughCircles on the mask with contours
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:  # Adjust the area threshold as needed
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > 10:  # Minimum radius threshold
                # Check if the contour has an approximately circular shape
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                if 0.7 < circularity < 1.3:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), 3)

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
