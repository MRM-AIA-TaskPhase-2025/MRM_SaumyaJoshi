#!/usr/bin/env python3

import sys
import rospy
import cv2
import os
import csv
import pandas as pd
from sensor_msgs.msg import Imu, Image, NavSatFix, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QSpacerItem, QSizePolicy, QPushButton, QMessageBox
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from std_srvs.srv import Empty, EmptyResponse

class IMUGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.display_euler = True  # Display Euler angles by default
        self.image_folder = 'screenshots'
        self.video_folder = 'videos'
        self.image_counter = 0
        self.is_recording = False
        self.recorded_video = None
        self.initUI()
        rospy.init_node('imu_display', anonymous=True)
        rospy.Subscriber('/imu/data', Imu, self.update_imu)
        rospy.Subscriber('/atom/camera/rgb/image_raw', Image, self.update_video)
        rospy.Subscriber('/rover/gps/fix', NavSatFix, self.update_gps)
        rospy.Subscriber('/scan', LaserScan, self.update_lidar)

    def initUI(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(20, 20, 20, 20)  # Adjust margins as needed
        layout.setSpacing(10)
        
        self.heading_label_imu = QLabel('IMU DATA:', self)
        self.heading_label_imu.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(self.heading_label_imu, alignment=Qt.AlignTop)

        self.imu_label = QLabel('IMU Data will be displayed here.', self)
        layout.addWidget(self.imu_label, alignment=Qt.AlignTop)
        
        layout.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Fixed))
        
        self.heading_label_video = QLabel('Rover Camera Feed:', self)
        self.heading_label_video.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(self.heading_label_video, alignment=Qt.AlignTop)

        self.video_label = QLabel(self)
        self.video_label.setText('Video feed will be displayed here.')
        layout.addWidget(self.video_label, alignment=Qt.AlignTop)
        
        layout.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Fixed))
        
        self.heading_label_gps = QLabel('GPS Data:', self)
        self.heading_label_gps.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(self.heading_label_gps, alignment=Qt.AlignTop)

        self.gps_label = QLabel('GPS Data will be displayed here.', self)
        layout.addWidget(self.gps_label, alignment=Qt.AlignTop)
        
        layout.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Fixed))
        
        self.heading_label_obstacle = QLabel('Obstacle Detection:', self)
        self.heading_label_obstacle.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(self.heading_label_obstacle, alignment=Qt.AlignTop)

        self.obstacle_label = QLabel('Obstacle status will be displayed here.', self)
        layout.addWidget(self.obstacle_label, alignment=Qt.AlignTop)
        
        layout.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Fixed))
        
        self.toggle_button = QPushButton('Toggle Orientation', self)
        self.toggle_button.clicked.connect(self.toggle_orientation)
        layout.addWidget(self.toggle_button, alignment=Qt.AlignTop)
        
        self.screenshot_button = QPushButton('Take Screenshot', self)
        self.screenshot_button.clicked.connect(self.take_screenshot)
        layout.addWidget(self.screenshot_button, alignment=Qt.AlignTop)
        
        self.record_button = QPushButton('Record Video', self)
        self.record_button.clicked.connect(self.record_video)
        layout.addWidget(self.record_button, alignment=Qt.AlignTop)
        
        layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        self.setLayout(layout)
        self.setWindowTitle('IMU, GPS, and Video Feed Display')
        self.show()

    def update_imu(self, data):
        orientation = data.orientation
        angular_velocity = data.angular_velocity
        linear_acceleration = data.linear_acceleration

        if self.display_euler:
            imu_text = (f"Euler Angles:\n"
                        f"Roll={self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)[0]}, "
                        f"Pitch={self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)[1]}, "
                        f"Yaw={self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)[2]}")
        else:
            imu_text = (f"Quaternion:\n"
                        f"x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

        imu_text += (f"\nAngular Velocity:\n"
                     f"x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}\n"
                     f"Linear Acceleration:\n"
                     f"x={linear_acceleration.x}, y={linear_acceleration.y}, z={linear_acceleration.z}")

        self.imu_label.setText(imu_text)

    def euler_from_quaternion(self, x, y, z, w):
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def update_video(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image_resized = cv2.resize(cv_image, (320, 240))  # Adjust dimensions as needed
            height, width, channel = cv_image_resized.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image_resized.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.video_label.setPixmap(QPixmap.fromImage(q_image))
            
            if self.is_recording:
                self.recorded_video.write(cv_image)
            
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def update_gps(self, data):
        #rospy.loginfo("Received GPS data.")
        gps_text = (f"Latitude: {data.latitude}\n"
                    f"Longitude: {data.longitude}\n"
                    f"Altitude: {data.altitude}")
        self.gps_label.setText(gps_text)

    def update_lidar(self, data):
        min_distance = min(data.ranges)
        
        if min_distance < 1.0:  # Adjust threshold as needed
            self.obstacle_label.setText("Obstacle Detected")
            self.obstacle_label.setStyleSheet("color: red;")
        else:
            self.obstacle_label.setText("Path Clear")
            self.obstacle_label.setStyleSheet("color: green;")

    def toggle_orientation(self):
        self.display_euler = not self.display_euler
        self.update_imu(rospy.wait_for_message('/imu/data', Imu))

    def take_screenshot(self):
        try:
            if not os.path.exists(self.image_folder):
                os.makedirs(self.image_folder)
            
            screenshot_path = os.path.join(self.image_folder, f'screenshot_{self.image_counter}.png')
            screenshot = QPixmap.grabWidget(self.video_label)
            screenshot.save(screenshot_path)
            self.image_counter += 1
            QMessageBox.information(self, 'Screenshot Taken', f'Screenshot saved as {screenshot_path}')
        
        except Exception as e:
            QMessageBox.warning(self, 'Error', f'Failed to take screenshot: {e}')

    def record_video(self):
        if not self.is_recording:
            try:
                if not os.path.exists(self.video_folder):
                    os.makedirs(self.video_folder)
                
                video_path = os.path.join(self.video_folder, f'recorded_video.mp4')
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                frame_width, frame_height = 320, 240  # Adjust dimensions as needed
                self.recorded_video = cv2.VideoWriter(video_path, fourcc, 10, (frame_width, frame_height))
                self.is_recording = True
                self.record_button.setText('Stop Recording')
                QMessageBox.information(self, 'Recording Started', f'Recording video to {video_path}')
            
            except Exception as e:
                QMessageBox.warning(self, 'Error', f'Failed to start recording: {e}')
        
        else:
            self.recorded_video.release()
            self.is_recording = False
            self.record_button.setText('Record Video')
            QMessageBox.information(self, 'Recording Stopped', 'Video recording stopped.')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = IMUGUI()
    sys.exit(app.exec_())
