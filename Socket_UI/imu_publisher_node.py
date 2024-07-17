#!/usr/bin/env python3

import socket
import rospy
from sensor_msgs.msg import Imu

def imu_publisher():
    rospy.init_node('imu_publisher', anonymous=True)
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    # Set up the socket
    UDP_IP = "0.0.0.0"  # Listen on all interfaces
    UDP_PORT = 5005  # Match this with the port set in the app

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))

    rospy.loginfo("IMU Publisher Node Started")

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        rospy.loginfo(f"Received data: {data} from {addr}")

        try:
            imu_data = [float(x) for x in data.decode('utf-8').strip().split(',')]

            # Ensure the received data has the expected number of fields
            if len(imu_data) != 38:
                raise ValueError(f"Unexpected number of fields: {len(imu_data)}")

            # Populate IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"

            # Assign Quaternion
            imu_msg.orientation.x = imu_data[5]
            imu_msg.orientation.y = imu_data[6]
            imu_msg.orientation.z = imu_data[7]
            imu_msg.orientation.w = imu_data[8]

            # Assign Angular Velocity
            imu_msg.angular_velocity.x = imu_data[27]
            imu_msg.angular_velocity.y = imu_data[28]
            imu_msg.angular_velocity.z = imu_data[29]

            # Assign Linear Acceleration
            imu_msg.linear_acceleration.x = imu_data[23]
            imu_msg.linear_acceleration.y = imu_data[24]
            imu_msg.linear_acceleration.z = imu_data[25]

            imu_pub.publish(imu_msg)
            rospy.loginfo("IMU data published")

        except Exception as e:
            rospy.logerr(f"Failed to process IMU data: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
