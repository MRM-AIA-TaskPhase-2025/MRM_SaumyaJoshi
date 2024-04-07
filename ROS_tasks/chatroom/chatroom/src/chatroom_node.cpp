#include <ros/ros.h>
#include "chatroom/ChatMessage.h"

void messageCallback(const chatroom::ChatMessage::ConstPtr& msg) {
    // Process the incoming message and display it
    ROS_INFO("[%s] %s", msg->sender.c_str(), msg->message.c_str());
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "chatroom_node");
    ros::NodeHandle nh;

    // Subscribe to the message topic
    ros::Subscriber sub = nh.subscribe("/chatroom_topic", 10, messageCallback);

    // Spin and process incoming messages
    ros::spin();

    return 0;
}

