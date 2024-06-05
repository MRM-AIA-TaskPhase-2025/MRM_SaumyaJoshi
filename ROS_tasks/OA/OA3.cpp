#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
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
    double currentYaw;
    bool obstacleDetected; // Flag to switch to wall-following mode

    // Distance measurements from LaserScan
    double rightDist;
    double frontDist;
    double frontRightDist;

    // Hit point coordinates
    double hitLatitude;
    double hitLongitude;
    bool hitPointSet;
    bool crash;

    // Line equation parameters
    double slope; 
    double yIntercept;  
    double D1;

    enum class WallFollowState {
        FOLLOW_WALL,
        TURN_LEFT,
        TURN_RIGHT,
        FIND_WALL,
        NAVIGATE_AROUND_OBSTACLE
    };

    WallFollowState wallFollowState = WallFollowState::FOLLOW_WALL;
    bool navigatingAroundObstacle = false; // Flag to indicate if navigating around an obstacle

    // Constructor to initialize the rover's position
    Rover() : obstacleDetected(false), hitPointSet(false){
        // Subscribe to GPS fix topic to receive current location
        gpsFixSub = nh.subscribe<sensor_msgs::NavSatFix>(
            "/rover/gps/fix", 20, &Rover::gpsFixCallback, this);

        // Subscribe to IMU topic to receive yaw values
        imuSub = nh.subscribe<sensor_msgs::Imu>(
            "/imu", 20, &Rover::imuCallback, this);

        // Subscribe to LaserScan topic to receive distance values
        laserSub = nh.subscribe<sensor_msgs::LaserScan>(
            "/scan", 10, &Rover::laserCallback, this);

        // Initialize publisher for velocity commands
        cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    }

    // Callback function for receiving GPS fix
    void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Update rover's current location based on received GPS fix
        currentLatitude = msg->latitude;
        currentLongitude = msg->longitude;
    }

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

 
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Ensure there are ranges to process
        int numRanges = msg->ranges.size();
        if (numRanges == 0) {
            std::cout << "No range data available." << std::endl;
            return;
        }

        // Extract relevant ranges
        rightDist = msg->ranges[0]; // 90 degrees to the right
        frontDist = msg->ranges[89]; // Directly in front, assuming 180 degree scan range

        // Debug prints for initial values
        //std::cout << "Initial Ranges: rightDist: " << rightDist << ", frontDist: " << frontDist << std::endl;

        // Simple obstacle detection logic
        double minDistThreshold = 0.9; // Minimum distance to consider as an obstacle
        double bufferZone = 0;       // Buffer zone for more robust obstacle detection
        double sideLength = 0.5;       // Length of one side of the rover (in meters)
        bool obstacleDetectedInScan = false;
        crash = false;

        double angleIncrement = msg->angle_increment;
        double startAngle = msg->angle_min;

        // Debug prints for angle increment and start angle
        //std::cout << "Angle Increment: " << angleIncrement << ", Start Angle: " << startAngle << std::endl;

        for (int i = 0; i < numRanges; ++i) {
            double range = msg->ranges[i];
            double angle = startAngle + i * angleIncrement;

            // Calculate the effective threshold based on the angle
            double effectiveThreshold;
            //double absAngle = std::fabs(angle);

            // Normalize the angle within the range [0, 180]
            //absAngle = std::fmod(absAngle, M_PI);

            // Convert to degrees for easier handling
            double angleDegrees = angle * 180.0 / M_PI;

            // Debug print statements
            //std::cout << "Range[" << i << "]: " << range << ", Angle: " << angleDegrees << " degrees" << std::endl;
            if ((angleDegrees >= 0 && angleDegrees <= 9) || (angleDegrees >= 70 && angleDegrees <= 89)) {
                // Calculate for angles between 0 to 9 degrees and 70 to 89 degrees
                effectiveThreshold = minDistThreshold;
            } else {
                // For other angles, use the minimum threshold
                effectiveThreshold = std::sqrt(2.0) * minDistThreshold;
            }
            // Debug print statements
            //std::cout << "Effective Threshold: " << effectiveThreshold << std::endl;

            // Check if the range is below the effective threshold
            if (range < effectiveThreshold / 1.5) {
                crash = true;
            }
            if (range < effectiveThreshold) {
                obstacleDetectedInScan = true;
                break;
            }
        }

        // Update obstacle detection status
        if (obstacleDetectedInScan) {
            obstacleDetected = true;
            std::cout << "Obstacle detected" << std::endl;
            if (!hitPointSet) {
                hitLatitude = currentLatitude;
                hitLongitude = currentLongitude;
                hitPointSet = true;
                std::cout << "Hit point set at: (" << hitLatitude << ", " << hitLongitude << ")" << std::endl;
            }
        } else {
            obstacleDetected = false;
            navigatingAroundObstacle = false;
        }
    }


    
    // Method to calculate the bearing (angle) between two GPS coordinates
    double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
        double dLon = toRadians(lon2 - lon1);
        double y = sin(dLon) * cos(toRadians(lat2));
        double x = cos(toRadians(lat1)) * sin(toRadians(lat2)) -
                   sin(toRadians(lat1)) * cos(toRadians(lat2)) * cos(dLon);
        double bearing = atan2(x,y);
        return toDegrees(bearing);
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

    // Method for wall-following behavior
    void followWall() {
        double desiredDistance = 0.9; // Desired distance from the wall (meters)
        double k_p = 0.9; // Proportional gain

        geometry_msgs::Twist twistMsg;

        // Print current distance values for debugging
        std::cout << "frontDist: " << frontDist << ", frontRightDist: " << frontRightDist << ", rightDist: " << rightDist << std::endl;

        switch (wallFollowState) {
            case WallFollowState::FOLLOW_WALL:
                std::cout << "State: FOLLOW_WALL" << std::endl;
                if (frontDist < desiredDistance) {
                    // Obstacle in front, transition to TURN_LEFT state
                    wallFollowState = WallFollowState::TURN_LEFT;
                    std::cout << "Transition to TURN_LEFT" << std::endl;
                } else if (crash) {
                    // Very close to a corner, transition to TURN_RIGHT state
                    wallFollowState = WallFollowState::TURN_LEFT;
                    std::cout << "Transition to TURN_RIGHT" << std::endl;
                } else if (rightDist > desiredDistance * 2 && !std::isinf(rightDist)) {
                    // Lost the wall, transition to FIND_WALL state
                    //wallFollowState = WallFollowState::FIND_WALL;
                    navigatingAroundObstacle=false;
                    std::cout << "Transition to FIND_WALL" << std::endl;
                } else {
                    // Maintain distance from the wall on the right
                    double error = desiredDistance - rightDist;
                    twistMsg.linear.x = 0.3; // Move forward
                    twistMsg.angular.z = 0; // Adjust direction based on the error
                    std::cout << "Moving forward with error adjustment" << std::endl;
                }
                break;

            case WallFollowState::TURN_LEFT:
                std::cout << "State: TURN_LEFT" << std::endl;
                if (rightDist <= (desiredDistance + 0.05) && rightDist >= (desiredDistance - 0.05) && std::isinf(frontDist)) {
                    // Aligned with the wall and obstacle cleared, transition back to FOLLOW_WALL state
                    wallFollowState = WallFollowState::FOLLOW_WALL;
                    std::cout << "Transition to FOLLOW_WALL after aligning with wall and clearing obstacle" << std::endl;
                }
                twistMsg.linear.x = 0.0;
                twistMsg.angular.z = 0.5;
                break;

            case WallFollowState::TURN_RIGHT:
                std::cout << "State: TURN_RIGHT" << std::endl;
                if (rightDist >= desiredDistance / 2) {
                    // Turned enough to the right, transition back to FOLLOW_WALL state
                    wallFollowState = WallFollowState::FOLLOW_WALL;
                    std::cout << "Transition to FOLLOW_WALL after turning right" << std::endl;
                }
                twistMsg.linear.x = 0.0;
                twistMsg.angular.z = -1.0;
                break;

            case WallFollowState::FIND_WALL:
                std::cout << "State: FIND_WALL" << std::endl;
                if (rightDist <= desiredDistance * 2) {
                    // Wall found, transition back to FOLLOW_WALL state
                    wallFollowState = WallFollowState::FOLLOW_WALL;
                    std::cout << "Transition to FOLLOW_WALL" << std::endl;
                }
                twistMsg.linear.x = 0.0;
                twistMsg.angular.z = -0.5;
                break;
        }

        cmdVelPub.publish(twistMsg);
    }

    // Method to navigate to a target GPS location
    void navigateToTarget(double targetLatitude, double targetLongitude) {
        ros::Rate loop_rate(10); // 10 Hz

        double initialBearing = calculateBearing(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
        std::cout << "Initial bearing: " << initialBearing << " degrees" << std::endl;
        
        while (ros::ok()) {
            // Get the current location of the rover
            double currentLatitude = this->currentLatitude;
            double currentLongitude = this->currentLongitude;
            
            if (hitPointSet) {
                std::cout << hitLatitude << " " << hitLongitude << std::endl;
                slope = (targetLatitude - hitLatitude) / (targetLongitude - hitLongitude);
                yIntercept = hitLatitude - slope * hitLongitude;
                D1 = calculateDistance(targetLatitude, targetLongitude, hitLatitude, hitLongitude);
                std::cout << slope << " " << yIntercept << std::endl;
                std::cout << "Line equation: latitude = " << slope << " * longitude + " << yIntercept << std::endl; }

            // Calculate distance to target
            double distance = calculateDistance(targetLatitude, targetLongitude, currentLatitude, currentLongitude);
            std::cout << "Distance to target: " << distance << " meters" << std::endl;

            // Check if rover has reached target position
            double currentYaw = this->currentYaw;
            double angleError = initialBearing - currentYaw;

            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            std::cout << "Yaw: " << currentYaw << " degrees, Angle error: " << angleError << " degrees" << std::endl;

            geometry_msgs::Twist twistMsg;

            if (obstacleDetected || navigatingAroundObstacle) {
                initialBearing = calculateBearing(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
                std::cout << "Initial bearing: " << initialBearing << " degrees" << std::endl;
                std::cout << std::abs(currentLatitude - (slope * currentLongitude + yIntercept)) << "AAAAAAAAAAAA" << std::endl;
                std::cout << currentLatitude << " " << currentLongitude << std::endl;
                if ((distance < D1*0.8) && (std::abs(currentLatitude - (slope * currentLongitude + yIntercept)) < 0.000001)) {
                    std::cout << "INNNNNNNNNNNNNNNNNNN" << std::endl;
                    twistMsg.angular.z = angleError * 0.2; // Adjust yaw
                    twistMsg.linear.x = 0.0;
                    cmdVelPub.publish(twistMsg); 
                    navigatingAroundObstacle=false;}
                else {
                    followWall();
                    navigatingAroundObstacle = true; } // Set flag to indicate navigating around an obstacle
            } else if (distance <= 0.3 && std::fabs(angleError) <= 0.1) {
                std::cout << "Rover reached target position." << std::endl;
                //geometry_msgs::Twist twistMsg;
                twistMsg.linear.x = 0.0;
                twistMsg.angular.z = 0.0;
                std::cout << twistMsg << std::endl;
                cmdVelPub.publish(twistMsg);
                std::cout<<"Published" << std::endl;
                std::cout << "Enter target latitude: ";
                std::cin >> targetLatitude;
                std::cout << "Enter target longitude: ";
                std::cin >> targetLongitude; 
                ros::spinOnce();
                loop_rate.sleep();
                currentLatitude = this->currentLatitude;
                currentLongitude = this->currentLongitude;
                initialBearing = calculateBearing(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
                std::cout << currentLatitude << " " << currentLongitude << std::endl;
                std::cout << "Initial bearing: " << initialBearing << " degrees" << std::endl;
                WallFollowState wallFollowState = WallFollowState::FOLLOW_WALL;
                hitPointSet=false;
            } else if (distance > 0.3 && std::fabs(angleError) <= 0.1) {
                twistMsg.linear.x = 0.3; // Move forward
                twistMsg.angular.z = 0.0; // Go straight
                cmdVelPub.publish(twistMsg);    
            } else {
                twistMsg.angular.z = angleError * 0.2; // Adjust yaw
                twistMsg.linear.x = 0.0;
                cmdVelPub.publish(twistMsg);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber gpsFixSub;
    ros::Subscriber imuSub;
    ros::Subscriber laserSub;
    ros::Publisher cmdVelPub;
};

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "rover_controller_node");
    ros::NodeHandle nh;

    // Create a rover instance
    Rover rover;

    // Input target GPS coordinates
    double targetLatitude, targetLongitude;
    std::cout << "Enter target latitude: ";
    std::cin >> targetLatitude;
    std::cout << "Enter target longitude: ";
    std::cin >> targetLongitude;

    // Wait for initial GPS fix to set the rover's initial position
    ros::spinOnce();
    ros::Rate loop_rate(10); // 10 Hz
    loop_rate.sleep();

    // Start navigation to the target
    rover.navigateToTarget(targetLatitude, targetLongitude);

    return 0;
}

//((currentLatitude >= hitLatitude + 0.1 &&  currentLatitude <= hitLatitude - 0.1) || (currentLongitude >= hitLongitude + 0.1 && currentLongitude <= hitLongitude - 0.1 ))