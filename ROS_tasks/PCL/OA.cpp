#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <limits>
#include <thread>
#include <chrono>

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

    const double OBSTACLE_THRESHOLD = 1.0; // Minimum distance to consider an obstacle
    const double TURN_ANGLE = M_PI / 4.0;  // Angle to turn

    double currentLatitude; // Current latitude
    double currentLongitude; // Current longitude
    double currentYaw;
    bool obstacleDetected; // Flag to switch to wall-following mode

    double rightDist;
    double frontDist;
    double frontRightDist;

    double hitLatitude;
    double hitLongitude;
    bool hitPointSet;
    bool crash;

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
    //std::thread timerThread;

    Rover() : obstacleDetected(false), hitPointSet(false) {
        // Subscriber to PointCloud2 topic
        point_cloud_sub = nh.subscribe("/zed2_cam/depth/points", 1, &Rover::pointCloudCallback, this);

        // Publisher to /cmd_vel for controlling robot movement

        // Publisher for ground-removed point cloud
        ground_removed_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_removed_points", 1);

        // Subscribe to GPS fix topic to receive current location
        gpsFixSub = nh.subscribe<sensor_msgs::NavSatFix>(
            "/rover/gps/fix", 20, &Rover::gpsFixCallback, this);

        // Subscribe to IMU topic to receive yaw values
        imuSub = nh.subscribe<sensor_msgs::Imu>(
            "/imu", 20, &Rover::imuCallback, this);

        // Initialize publisher for velocity commands
        cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    }

    /*void resetNavigatingAroundObstacle() {
    std::this_thread::sleep_for(std::chrono::seconds(5)); // Adjust the duration as needed
    navigatingAroundObstacle = false;
    std::cout << "NavigatingAroundObstacle reset to false after a few extra seconds." << std::endl;
}

    void startTimer() {
        if (timerThread.joinable()) {
            timerThread.join(); // Wait for the previous thread to finish if it's still running
        }
        timerThread = std::thread(&Rover::resetNavigatingAroundObstacle, this);
    } */

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

    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_no_ground) {
    if (cloud_no_ground->points.empty()) {
        ROS_WARN("Point cloud is empty after ground removal.");
        return;
    }
    crash = false;
    frontDist = std::numeric_limits<double>::max();
    rightDist = std::numeric_limits<double>::max();
    double left_distance = std::numeric_limits<double>::max();

    for (const auto& point : cloud_no_ground->points) {
        double distance = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));

        if (point.y > 0) {
            if (fabs(point.x) < fabs(point.y)) {
                frontDist = std::min(frontDist, distance);
            } else if (point.x > 0) {
                rightDist = std::min(rightDist, distance);
            } else {
                left_distance = std::min(left_distance, distance);
            }
        }
    }
    
    if (frontDist<1 || rightDist<1.5) {
        obstacleDetected=true;
        //if (timerThread.joinable()) 
          //  timerThread.join(); 
          }
    else {
        obstacleDetected = false;
        //startTimer();
         }

    if (frontDist<0.4 || rightDist<0.4)
        crash=true;

    ROS_INFO("Front distance: %f, Right distance: %f, Left distance: %f",
             frontDist, rightDist, left_distance);

}

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Print raw point cloud size
    ROS_INFO("Received Point Cloud with %zu points", cloud->points.size());

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_grid.filter(*cloud_filtered);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);  // Adjusted for better ground plane segmentation
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("No ground plane found");
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*cloud_no_ground);

    // Print ground-removed point cloud size
    ROS_INFO("Ground-Removed Point Cloud with %zu points", cloud_no_ground->points.size());

    // Publish the ground-removed point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_no_ground, output);
    output.header.frame_id = cloud_msg->header.frame_id; // Keep the same frame_id
    ground_removed_pub.publish(output);

    processPointCloud(cloud_no_ground);
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
        double k_p = 4.0; // Proportional gain

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
                    
                        twistMsg.linear.x = 0.3;
                        twistMsg.angular.z = -0.5;
                        navigatingAroundObstacle=false;
                    
                    
                    //std::cout << "Transition to FIND_WALL" << std::endl;
                } else {
                    // Maintain distance from the wall on the right
                    double error = desiredDistance - rightDist;
                    twistMsg.linear.x = 0.5; // Move forward
                    twistMsg.angular.z = -error * k_p; // Adjust direction based on the error
                    std::cout << "Moving forward with error adjustment" << std::endl;
                }
                break;

            case WallFollowState::TURN_LEFT:
                std::cout << "State: TURN_LEFT" << std::endl;
                if (rightDist <= (desiredDistance + 0.3) && rightDist >= (desiredDistance - 0.3)) {
                    // Aligned with the wall and obstacle cleared, transition back to FOLLOW_WALL state
                    wallFollowState = WallFollowState::FOLLOW_WALL;
                    std::cout << "Transition to FOLLOW_WALL after aligning with wall and clearing obstacle" << std::endl;
                }
                twistMsg.linear.x = 0.0;
                twistMsg.angular.z = 0.8;
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
            navigatingAroundObstacle=false;
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
    ros::Subscriber point_cloud_sub;
    ros::Publisher ground_removed_pub;
    ros::Subscriber gpsFixSub;
    ros::Subscriber imuSub;
    ros::Publisher cmdVelPub;
};

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "OA");
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

