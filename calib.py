import cv2
import numpy as np

# ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
aruco_params = cv2.aruco.DetectorParameters()

# Chessboard parameters
chessboard_size = (7, 5)  # Number of inner corners
square_size = 0.025  # Size of a square in meters

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Prepare object points
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Camera matrix and distortion coefficients
camera_matrix = None
dist_coeffs = None

# RTSP stream URL (replace with your actual RTSP URL)
rtsp_url = "rtsp://localhost:8554/test"

# Create a VideoCapture object
cap = cv2.VideoCapture(rtsp_url)

calibration_frames = 10
frames_processed = 0

while True:
    for i in range(10):
        ret, frame = cap.read()
    if not ret:
        #print("Failed to receive frame")
        continue



    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if camera_matrix is None and frames_processed < calibration_frames:
        # Attempt to find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(frame, chessboard_size, corners, ret)
            frames_processed += 1
            print(f"Calibration: {frames_processed}/{calibration_frames}")
                # Display the frame
            cv2.imshow('CALIB', frame)

        if frames_processed == calibration_frames:
            print("Calibrating camera...")
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            print("Camera calibrated!")
            print("Camera matrix:")
            print(camera_matrix)
            print("Distortion coefficients:")
            print(dist_coeffs)
    elif camera_matrix is not None:
        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            # Estimate pose for each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

            for i in range(len(ids)):
                # Draw axis for each marker
                cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)

                # Convert rotation vector to Euler angles
                rot_mat, _ = cv2.Rodrigues(rvecs[i])
                euler_angles = cv2.RQDecomp3x3(rot_mat)[0]

                # Print marker ID and orientation
                print(f"Marker ID: {ids[i][0]}")
                print(f"Orientation - Roll: {euler_angles[0]:.2f}, Pitch: {euler_angles[1]:.2f}, Yaw: {euler_angles[2]:.2f}")

    # Display the frame
    cv2.imshow('RTSP Stream', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()