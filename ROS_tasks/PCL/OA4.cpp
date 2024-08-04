#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <limits>

class Rover {
public:
    Rover();
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

private:
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_no_ground);
    void publishControlCommands(double front_distance, double right_distance, double left_distance);

    ros::Subscriber point_cloud_sub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher ground_removed_pub;

    const double OBSTACLE_THRESHOLD = 1.0; 
    const double TURN_ANGLE = M_PI / 4.0;  
};

Rover::Rover() {
    ros::NodeHandle nh;

    // Subscriber to PointCloud2 topic
    point_cloud_sub = nh.subscribe("/zed2_cam/depth/points", 1, &Rover::pointCloudCallback, this);

    // Publisher to /cmd_vel for controlling robot movement
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Publisher for ground-removed point cloud
    ground_removed_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_removed_points", 1);
}

void Rover::processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_no_ground) {
    if (cloud_no_ground->points.empty()) {
        ROS_WARN("Point cloud is empty after ground removal.");
        return;
    }

    double front_distance = std::numeric_limits<double>::max();
    double right_distance = std::numeric_limits<double>::max();
    double left_distance = std::numeric_limits<double>::max();

    for (const auto& point : cloud_no_ground->points) {
        double distance = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));

        if (point.y > 0) {
            if (fabs(point.x) < fabs(point.y)) {
                front_distance = std::min(front_distance, distance);
            } else if (point.x > 0) {
                right_distance = std::min(right_distance, distance);
            } else {
                left_distance = std::min(left_distance, distance);
            }
        }
    }

    ROS_INFO("Front distance: %f, Right distance: %f, Left distance: %f",
             front_distance, right_distance, left_distance);

    publishControlCommands(front_distance, right_distance, left_distance);
}

void Rover::publishControlCommands(double front_distance, double right_distance, double left_distance) {
    geometry_msgs::Twist cmd_vel;

    if (front_distance < OBSTACLE_THRESHOLD) {
        ROS_WARN("Obstacle detected in front! Turning left...");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = TURN_ANGLE;
    } else if (right_distance < OBSTACLE_THRESHOLD) {
        ROS_WARN("Obstacle detected on the right! Moving forward...");
        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = 0.0;
    } else if (left_distance < OBSTACLE_THRESHOLD) {
        ROS_WARN("Obstacle detected on the left! Moving forward...");
        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = 0.0;
    } else {
        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = 0.0;
    }

    cmd_vel_pub.publish(cmd_vel);
}

void Rover::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");

    Rover rover;

    ros::spin();

    return 0;
}
