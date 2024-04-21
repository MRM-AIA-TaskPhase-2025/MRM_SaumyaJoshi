#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"

// Constants for Earth's radius in meters
constexpr double EARTH_RADIUS = 6371000.0;

// Class representing the rover
class Rover {
public:
    double currentLatitude; // Current latitude
    double currentLongitude; // Current longitude

    // Constructor to initialize the rover's position
    Rover() {
        // Subscribe to GPS fix topic to receive current location
        gpsFixSub = nh.subscribe<sensor_msgs::NavSatFix>(
            "/rover/gps/fix", 10, boost::bind(&Rover::gpsFixCallback, this, _1));
    }

    // Callback function for receiving GPS fix
    void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Update rover's current location based on received GPS fix
        currentLatitude = msg->latitude;
        currentLongitude = msg->longitude;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber gpsFixSub;
};

// Convert degrees to radians
double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

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
    ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Input target GPS coordinates
    double targetLatitude, targetLongitude;
    std::cout << "Enter target latitude: ";
    std::cin >> targetLatitude;
    std::cout << "Enter target longitude: ";
    std::cin >> targetLongitude;

    // Loop to continuously publish Twist messages
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
        if (distance < 0.1) {
            std::cout << "Rover reached target position." << std::endl;
            geometry_msgs::Twist twistMsg;
            twistMsg.linear.x = 0.0;
            std::cout << "Twist message contents: " << twistMsg << std::endl;
            std::cout << "Publishing twist message" << std::endl;
            cmdVelPub.publish(twistMsg);
            std::cout << "Twist message published" << std::endl;

            break; // Exit the loop
        }

        // Publish the Twist message to move the rover forward
        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = 0.1; // Linear velocity (m/s)
        twistMsg.angular.z = 0.0; // Angular velocity (rad/s)
        cmdVelPub.publish(twistMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
