#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>


// Constants for Earth's radius in meters
constexpr double EARTH_RADIUS = 6371000.0;

// Convert degrees to radians
double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Convert radians to degrees
double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

// Class representing the rover
class Rover {
public:
    double currentLatitude; // Current latitude
    double currentLongitude; // Current longitude

    // Constructor to initialize the rover's position
    Rover() {
    // Subscribe to GPS fix topic to receive current location
    gpsFixSub = nh.subscribe<sensor_msgs::NavSatFix>(
        "/rover/gps/fix", 20, [this](const sensor_msgs::NavSatFix::ConstPtr& msg) {
            gpsFixCallback(msg);
        });

    // Subscribe to IMU topic to receive yaw values
    imuSub = nh.subscribe<sensor_msgs::Imu>(
        "/imu", 20, [this](const sensor_msgs::Imu::ConstPtr& msg) {
            imuCallback(msg);
        });
}

    // Callback function for receiving GPS fix
    void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Update rover's current location based on received GPS fix
        currentLatitude = msg->latitude;
        currentLongitude = msg->longitude;
    }

    // Callback function for receiving IMU data
    // Callback function for receiving IMU data
// Callback function for receiving IMU data
// Callback function for receiving IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Convert orientation quaternion to Euler angles
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(msg->orientation, orientation);
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // Update rover's current yaw value based on received IMU data
    currentYaw = toDegrees(yaw);
    std::cout << "Yaw (degrees): " << currentYaw << std::endl;
}



    // Method to calculate the bearing (angle) between two GPS coordinates
   double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = toRadians(lon2 - lon1);
    double y = sin(dLon) * cos(toRadians(lat2));
    double x = cos(toRadians(lat1)) * sin(toRadians(lat2)) -
               sin(toRadians(lat1)) * cos(toRadians(lat2)) * cos(dLon);
    double bearing = atan2(y, x);
    return toDegrees(bearing);
}


    double getCurrentYaw() {
        return currentYaw;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber gpsFixSub;
    ros::Subscriber imuSub;
    double currentYaw; // Current yaw value
};

// Calculate distance between two GPS coordinates using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(toRadians(lat1)) * cos(toRadians(lat2)) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS * c;
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "rover_controller_node");
    ros::NodeHandle nh;

    // Create a rover instance
    Rover rover;

    // Wait for initial GPS fix to set the rover's initial position
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok() && (rover.currentLatitude == 0.0 || rover.currentLongitude == 0.0)) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Publish Twist messages to move the rover
    ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",20);

    // Input target GPS coordinates
    double targetLatitude, targetLongitude;
    std::cout << "Enter target latitude: ";
    std::cin >> targetLatitude;
    std::cout << "Enter target longitude: ";
    std::cin >> targetLongitude; // Replace with your target longitude

    // Calculate initial bearing from rover's current position to target
    double initialBearing = rover.calculateBearing(rover.currentLatitude, rover.currentLongitude, targetLatitude, targetLongitude);
    std::cout << "Initial bearing: " << initialBearing << " degrees" << std::endl;

    // Loop to continuously publish Twist messages
  /*  while (ros::ok()) {
        // Get the current location of the rover
        double currentLatitude = rover.currentLatitude;
        double currentLongitude = rover.currentLongitude;

        // Calculate distance to target
        double distance = calculateDistance(targetLatitude, targetLongitude, currentLatitude, currentLongitude);
        std::cout << "Distance to target: " << distance << " meters" << std::endl;
        std::cout << "Current latitude: " << currentLatitude << std::endl;
        std::cout << "Current longitude: " << currentLongitude << std::endl;

        // Check if rover has reached target position
        if (distance < 0.7) {
            std::cout << "Rover reached target position." << std::endl;
            geometry_msgs::Twist twistMsg;
            twistMsg.linear.x = 0.0;
            twistMsg.angular.z = 0.0;
            cmdVelPub.publish(twistMsg);
            break; // Exit the loop
        }

        // Calculate current yaw
        double currentYaw = rover.getCurrentYaw();
        std::cout << "yaw:"  <<  currentYaw << std::endl;

        // Calculate angle error between initial bearing and current yaw
        double angleError = initialBearing - currentYaw;
        if (angleError > 180.0) {
            angleError -= 360.0;
        } else if (angleError < -180.0) {
            angleError += 360.0;
        }
         //angleError=std::abs(angleError);
    std::cout << "error: " << angleError << std::endl;
        // Adjust rover's orientation towards target
        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = 0.3; // Linear velocity (m/s)

        // Check if yaw is equal to initial bearing, if yes, set angular velocity to 0
        if (angleError<=0.1 && angleError>=-0.1) {
            twistMsg.angular.z = 0.0;
            break;
        } else {
            twistMsg.angular.z = angleError * 0.2; // Angular velocity (rad/s)
            
            
        }
         
        std::cout << "av:"  <<  twistMsg << std::endl;
        cmdVelPub.publish(twistMsg);

        ros::spinOnce();
        loop_rate.sleep();
    } */
/*while (ros::ok()) {
// Get the current location of the rover
        double currentLatitude = rover.currentLatitude;
        double currentLongitude = rover.currentLongitude;

        // Calculate distance to target
        double distance = calculateDistance(targetLatitude, targetLongitude, currentLatitude, currentLongitude);
        std::cout << "Distance to target: " << distance << " meters" << std::endl;
        std::cout << "Current latitude: " << currentLatitude << std::endl;
        std::cout << "Current longitude: " << currentLongitude << std::endl;

        // Check if rover has reached target position
        if (distance < 0.7) {
            std::cout << "Rover reached target position." << std::endl;
            geometry_msgs::Twist twistMsg;
            twistMsg.linear.x = 0.0;
            twistMsg.angular.z = 0.0;
            cmdVelPub.publish(twistMsg);
            break; // Exit the loop
        }

        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = 0.3; // Linear velocity (m/s)
        twistMsg.angular.z = 0.0;

        std::cout << "av:"  <<  twistMsg << std::endl;
        cmdVelPub.publish(twistMsg);

        ros::spinOnce();
        loop_rate.sleep();
} */

// Adjust rover's orientation towards initial bearing
while (ros::ok()) {
    // Get the current location of the rover
    double currentLatitude = rover.currentLatitude;
    double currentLongitude = rover.currentLongitude;

    // Calculate angle error between initial bearing and current yaw
    double currentYaw = rover.getCurrentYaw();
    double angleError = initialBearing - currentYaw;
    if (angleError > 180.0) {
        angleError -= 360.0;
    } else if (angleError < -180.0) {
        angleError += 360.0;
    }
    std::cout << "Yaw: " << currentYaw << " Angle error: " << angleError << std::endl;

    // Adjust rover's orientation towards initial bearing
    geometry_msgs::Twist twistMsg;
    twistMsg.linear.x = 0.0; // No linear movement during orientation adjustment
    
    if (angleError<=0.1 && angleError>=-0.1) {
            twistMsg.angular.z = 0.0;
            std::cout << twistMsg << std::endl;
            if (ros::ok()) {
            std::cout << "works1" << std::endl;
            cmdVelPub.publish(twistMsg);
            std::cout << "works2" << std::endl;
            break;
            } 
    }
            else {
            twistMsg.angular.z = angleError * 0.2; // Angular velocity (rad/s)
        }
             
     cmdVelPub.publish(twistMsg);



    

    ros::spinOnce();
    loop_rate.sleep();
}
/*
// Move rover linearly towards the target position while adjusting yaw
bool isYawAligned = false; // Flag to track if yaw is aligned with initial bearing

while (ros::ok()) {
    // Get the current location of the rover
    double currentLatitude = rover.currentLatitude;
    double currentLongitude = rover.currentLongitude;

    // Calculate distance to target
    double distance = calculateDistance(targetLatitude, targetLongitude, currentLatitude, currentLongitude);
    std::cout << "Distance to target: " << distance << " meters" << std::endl;
    std::cout << "Current latitude: " << currentLatitude << std::endl;
    std::cout << "Current longitude: " << currentLongitude << std::endl;

    // Check if rover has reached target position
    if (distance < 0.7) {
        std::cout << "Rover reached target position." << std::endl;
        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = 0.0;
        twistMsg.angular.z = 0.0;
        std::cout << twistMsg << std::endl;
 if (ros::ok()) {
    std::cout << "work1" << std::endl;
        cmdVelPub.publish(twistMsg);
        std::cout << "work2" << std::endl;

 } 
        break; // Exit the loop
    }

    // Calculate angle error between initial bearing and current yaw
    double currentYaw = rover.getCurrentYaw();

    double angleError;

    if ((currentLatitude >= 0 && currentLongitude >= 0) || (currentLatitude < 0 && currentLongitude < 0))  {
        angleError = initialBearing - currentYaw; 
        std::cout << "hi" << std::endl;}
    else {
        angleError = initialBearing - currentYaw + 180;
        std::cout << "bye" << std::endl;
    }

    if (angleError > 180.0) {
        angleError -= 360.0;
    } else if (angleError < -180.0) {
        angleError += 360.0;
    }
    
    std::cout << "Yaw: " << currentYaw << " Angle error: " << angleError << std::endl;

    // Adjust rover's orientation towards initial bearing while moving linearly
    geometry_msgs::Twist twistMsg;

    if (std::fabs(angleError) <= 0.1) {
        isYawAligned = true; // Yaw is aligned with initial bearing
        twistMsg.angular.z = 0.0; // Stop angular movement
    } else {
        isYawAligned = false; // Yaw is not aligned with initial bearing
        twistMsg.angular.z = angleError * 0.2; // Angular velocity (rad/s)
    }

    
    if (isYawAligned) {
        twistMsg.linear.x = 0.3; // Linear velocity (m/s)
        std::cout << twistMsg << std::endl;
    } else {
        twistMsg.linear.x = 0.0; // Stop linear movement until yaw is aligned
        std::cout << twistMsg << std::endl;
    }
if (ros::ok()) {
    std::cout << "works3" << std::endl;
    cmdVelPub.publish(twistMsg);}

    ros::spinOnce();
    loop_rate.sleep();
}

*/


    return 0; 
}
