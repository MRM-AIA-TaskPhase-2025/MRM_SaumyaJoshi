import cv2
import numpy as np

def nothing(x):
    pass

# Initialize video capture
videoCapture = cv2.VideoCapture(0)  # Change the index if needed (e.g., 1 or 2)

# Create a window for trackbars
cv2.namedWindow("Trackbars")
cv2.createTrackbar("H_MIN", "Trackbars", 15, 179, nothing)
cv2.createTrackbar("H_MAX", "Trackbars", 35, 179, nothing)
cv2.createTrackbar("S_MIN", "Trackbars", 80, 255, nothing)
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

    # Define the lower and upper bounds for yellow color in HSV space
    lower_yellow = np.array([h_min, s_min, v_min])
    upper_yellow = np.array([h_max, s_max, v_max])

    # Create a mask for yellow color
    mask = cv2.inRange(hsvFrame, lower_yellow, upper_yellow)

    # Apply morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create an empty mask for the background
    background_mask = np.zeros_like(mask)

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
                    # Draw the contour on the background mask
                    cv2.drawContours(background_mask, [contour], -1, (255), thickness=2)
                    # Mark the center of the circle
                    cv2.circle(background_mask, (int(x), int(y)), 2, (255), 3)

    # Create a 3-channel black image
    background_removed = np.zeros_like(frame)
    
    # Copy only the contours from the background mask to the black frame
    background_removed[background_mask == 255] = [255, 255, 255]

    # Display the mask for debugging
    cv2.imshow("Yellow Mask", mask)
    # Display the frame with the background removed and only outlines
    cv2.imshow("Background Removed", background_removed)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all OpenCV windows
videoCapture.release()
cv2.destroyAllWindows()
