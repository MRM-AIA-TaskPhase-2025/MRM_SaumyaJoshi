import cv2
import numpy as np

# Dictionary of available ArUco markers
ARUCO_DICT = {
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def detect_markers(image):
    aruco_type_list = []
    
    for aruco_type, dictionary_id in ARUCO_DICT.items():
        arucoDict = cv2.aruco.getPredefinedDictionary(dictionary_id)

        # Detect markers without DetectorParameters
        corners, ids, _ = cv2.aruco.detectMarkers(image, arucoDict)

        if ids is not None and len(ids) > 0:
            aruco_type_list.append(aruco_type)
            print(f"Markers detected using {aruco_type} dictionary")

            for i in range(len(ids)):
                markerId = ids[i][0]
                markerCorner = corners[i][0]

                cv2.polylines(image, [markerCorner.astype(int)], True, (0, 255, 0), 2)

                cX = int(np.mean(markerCorner[:, 0]))
                cY = int(np.mean(markerCorner[:, 1]))

                cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)
                cv2.putText(image, str(aruco_type) + " " + str(int(markerId)),
                            (int(markerCorner[0, 0] - 5), int(markerCorner[0, 1])), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255))
            
    return aruco_type_list

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)

    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)

    if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.025, matrix_coefficients, distortion_coefficients)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
             
    return frame

if __name__ == "__main__":
    intrinsic_camera = np.array([[519.3251048,0,349.09987307],[0,514.40756404,250.43849652],[0,0,1]])
    distortion = np.array([0.26675745,-2.31755081,0.01089678,-0.00866381,4.52638704])

    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, image = cap.read()
        if not ret:
            break

        aruco_types_detected = detect_markers(image)
        for aruco_type in aruco_types_detected:
            image = pose_estimation(image, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)
             
        cv2.imshow('Estimated Pose', image)
                
        if cv2.waitKey(50) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
