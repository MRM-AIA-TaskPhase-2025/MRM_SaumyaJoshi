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
        TURN_LEF,
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
        //std::cout << "Yaw (degrees): " << currentYaw << std::endl;
    }

 
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        double minDistThreshold = 1.0; // Minimum distance to consider as an obstacle
        double bufferZone = 0;       // Buffer zone for more robust obstacle detection
        double sideLength = 0.5;       // Length of one side of the rover (in meters)
        bool obstacleDetectedInScan = false;
        crash = false;
        // Ensure there are minRange to process
        int numRanges = msg->ranges.size();
        if (numRanges == 0) {
            std::cout << "No range data available." << std::endl;
            return;
        }

        double angleIncrement = msg->angle_increment;
        double startAngle = msg->angle_min;

        // Debug prints for angle increment and start angle
        //std::cout << "Angle Increment: " << angleIncrement << ", Start Angle: " << startAngle << std::endl;

        // Find the minimum distance in the ranges array and its corresponding angle
        double minRange = std::numeric_limits<double>::infinity();
        int minIndex = -1;

        for (int i = 0; i < numRanges; ++i) {
            if (msg->ranges[i] < minRange) {
                minRange = msg->ranges[i];
                minIndex = i;
            }
        }

        if (minIndex == -1) {
            std::cout << "No valid range data available." << std::endl;
            obstacleDetected = false;
            //navigatingAroundObstacle = false;
            return;
        }

        // Calculate the angle corresponding to the minimum range
        double minAngle = startAngle + minIndex * angleIncrement;
        double minAngleDegrees = minAngle * 180.0 / M_PI;

        // Print the results
        //std::cout << "Minimum Range: " << minRange << ", Angle: " << minAngleDegrees << " degrees" << std::endl;

    // Calculate the effective threshold based on the angle of the minimum range
        double effectiveThreshold;
        // Extract relevant minRange
        rightDist = msg->ranges[0]; // 90 degrees to the right
        frontDist = msg->ranges[89]; // Directly in front, assuming 180 degree scan minRange
        //std::cout << "right: " << rightDist << " front: " << frontDist << std::endl;
        // Debug prints for initial values
        //std::cout << "Initial minRange: rightDist: " << rightDist << ", frontDist: " << frontDist << std::endl;

        // Simple obstacle detection logic
        

       // double angleIncrement = msg->angle_increment;
        //double startAngle = msg->angle_min;

        // Debug prints for angle increment and start angle
        //std::cout << "Angle Increment: " << angleIncrement << ", Start Angle: " << startAngle << std::endl;

        //for (int i = 0; i < numminmins; ++i) {
           // double minRange = msg->minRange[i];
            //double angle = startAngle + i * angleIncrement;

            // Calculate the effective threshold based on the angle
            //double effectiveThreshold;
            //double absAngle = std::fabs(angle);

            // Normalize the angle within the minRange [0, 180]X
            //absAngle = std::fmod(absAngle, M_PI);

            // Convert to degrees for easier handling
            //double minAngle = angle * 180.0 / M_PI;
            
            // Debug print statements
            //std::cout << "minRange[" << i << "]: " << minRange << ", Angle: " << minAngle << " degrees" << std::endl;
            if ((minAngleDegrees >= 0 && minAngleDegrees <= 9) || (minAngleDegrees >= 70 && minAngleDegrees <= 109) || (minAngleDegrees >= 160 && minAngleDegrees <= 180)) {
                // Calculate for angles between 0 to 9 degrees and 70 to 89 degrees
                effectiveThreshold = minDistThreshold;
                //std::cout << "minRange: " << minRange << std::endl; 
                if (minRange < 0.75) {
                crash = true;
            }
            } else {
                // For other angles, use the minimum threshold
                effectiveThreshold = std::sqrt(2.0) * minDistThreshold;
                //std::cout << "minRange: " << minRange << std::endl; 
                if (minRange < effectiveThreshold / 1.5) {
                crash = true;
            }
            }
            // Debug print statements
            //std::cout << "Effective Threshold: " << effectiveThreshold << std::endl;

            // Check if the minRange is below the effective threshold
            
            if (minRange < effectiveThreshold) {
                obstacleDetectedInScan = true;
                //break;
            }
    

        // Update obstacle detection status
        if (obstacleDetectedInScan) {
            obstacleDetected = true;
            std::cout << "Obstacle detected" << std::endl;
            
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
        double desiredDistance = 1.0; // Desired distance from the wall (meters)
        double k_p = 2.0; // Proportional gain

        geometry_msgs::Twist twistMsg;

        // Print current distance values for debugging
        std::cout << "frontDist: " << frontDist << ", frontRightDist: " << frontRightDist << ", rightDist: " << rightDist << std::endl;

        switch (wallFollowState) {
            case WallFollowState::FOLLOW_WALL:
                std::cout << "State: FOLLOW_WALL" << std::endl;
                if (frontDist < desiredDistance) {
                    hitLatitude = currentLatitude;
                    hitLongitude = currentLongitude;
                    hitPointSet = true;
                    std::cout << "Hit point set at: (" << hitLatitude << ", " << hitLongitude << ")" << std::endl;
            
                    // Obstacle in front, transition to TURN_LEFT state
                    wallFollowState = WallFollowState::TURN_LEFT;
                    std::cout << "Transition to TURN_LEFT" << std::endl;
                } else if (crash) {
                    // Very close to a corner, transition to TURN_RIGHT state
                    wallFollowState = WallFollowState::TURN_LEF;
                    std::cout << "Transition" << std::endl;
                } 
                //else if (rightDist > desiredDistance * 1.5 && !std::isinf(rightDist)) {
                   // wallFollowState = WallFollowState::FIND_WALL;
                   // std::cout << "Transition to FIND_WALL" << std::endl;
                //}
                else if (rightDist > desiredDistance * 2) {
                    // Lost the wall, transition to FIND_WALL state
                    //wallFollowState = WallFollowState::FIND_WALL;
                    if(obstacleDetected) {
                        twistMsg.linear.x = 0.3;
                        twistMsg.angular.z = 0.0;
                    }
                    navigatingAroundObstacle=false;
                    //std::cout << "Transition to FIND_WALL" << std::endl;
                } else {
                    // Maintain distance from the wall on the right
                    double error = desiredDistance - rightDist;
                    twistMsg.linear.x = 0.3; // Move forward
                    twistMsg.angular.z = error * k_p; // Adjust direction based on the error
                    std::cout << "Moving forward with error adjustment" << std::endl;
                }
                break;

            case WallFollowState::TURN_LEFT:
                std::cout << "State: TURN_LEFT" << std::endl;
                if (rightDist <= (desiredDistance + 0.1) && rightDist >= (desiredDistance - 0.2) && std::isinf(frontDist)) {
                    // Aligned with the wall and obstacle cleared, transition back to FOLLOW_WALL state
                    wallFollowState = WallFollowState::FOLLOW_WALL;
                    std::cout << "Transition to FOLLOW_WALL after aligning with wall and clearing obstacle" << std::endl;
                }
                twistMsg.linear.x = 0.0;
                twistMsg.angular.z = 0.5;
                break;

            case WallFollowState::TURN_LEF:
                std::cout << "State: TURN_LEF" << std::endl;
                if (rightDist<desiredDistance && rightDist>0.4) {
                twistMsg.linear.x = 0.3;
                twistMsg.angular.z = 0.1;
                
                }
                else { twistMsg.linear.x = 0.0;
                twistMsg.angular.z = 0.5;
                }
                wallFollowState = WallFollowState::FOLLOW_WALL;
                break;

            case WallFollowState::FIND_WALL:
            std::cout << "State: FIND_WALL" << std::endl;
            twistMsg.linear.x = 0.0; // Move forward
            twistMsg.angular.z = -0.2; // Turn right slightly to find the wall
            std::cout << "Finding the wall by moving forward and turning right" << std::endl;
            wallFollowState = WallFollowState::FOLLOW_WALL;
                
            break;
        }

        cmdVelPub.publish(twistMsg);
    }

    // Method to navigate to a target GPS location
    void navigateToTarget(double targetLatitude, double targetLongitude) {
        ros::Rate loop_rate(10); // 10 Hz

        double initialBearing = calculateBearing(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
        //std::cout << "Initial bearing: " << initialBearing << " degrees" << std::endl;
        
        while (ros::ok()) {
            // Get the current location of the rover
            double currentLatitude = this->currentLatitude;
            double currentLongitude = this->currentLongitude;
            
            if (hitPointSet) {
                std::cout << hitLatitude << " " << hitLongitude << std::endl;
                slope = (targetLatitude - hitLatitude) / (targetLongitude - hitLongitude);
                yIntercept = hitLatitude - slope * hitLongitude;
                D1 = calculateDistance(targetLatitude, targetLongitude, hitLatitude, hitLongitude);
                //std::cout << slope << " " << yIntercept << std::endl;
                //std::cout << "Line equation: latitude = " << slope << " * longitude + " << yIntercept << std::endl; 
                }

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

            //std::cout << "Yaw: " << currentYaw << " degrees, Angle error: " << angleError << " degrees" << std::endl;

            geometry_msgs::Twist twistMsg;

            if (distance <= 0.3 ) {
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
            }
            else if (obstacleDetected || navigatingAroundObstacle) {
                initialBearing = calculateBearing(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
                //std::cout << "Initial bearing: " << initialBearing << " degrees" << std::endl;
                //std::cout << std::abs(currentLatitude - (slope * currentLongitude + yIntercept)) << "AAAAAAAAAAAA" << std::endl;
                //std::cout << currentLatitude << " " << currentLongitude << std::endl;
                 if ((distance < D1-0.4) && (std::abs(currentLatitude - (slope * currentLongitude + yIntercept)) < 0.000001)) {
                    std::cout << "INNNNNNNNNNNNNNNNNNN" << std::endl;
                    twistMsg.angular.z = angleError * 0.2; // Adjust yaw
                    twistMsg.linear.x = 0.0;
                    cmdVelPub.publish(twistMsg); 
                    navigatingAroundObstacle=false;}
                else { 
                    std::cout << "HIIIIIIIIIIIII" << std::endl;
                    followWall();
                    navigatingAroundObstacle = true; } // Set flag to indicate navigating around an obstacle
            }
            else if (distance > 0.3 && std::fabs(angleError) <= 0.1) {
                twistMsg.linear.x = 0.3; // Move forward
                twistMsg.angular.z = 0.0; // Go straight
                cmdVelPub.publish(twistMsg); } 
            else {
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
    ros::init(argc, argv, "OA3");
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