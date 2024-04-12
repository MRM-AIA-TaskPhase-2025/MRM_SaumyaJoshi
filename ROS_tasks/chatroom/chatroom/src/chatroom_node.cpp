#include <ros/ros.h>
#include "chatroom/ChatMessage.h"

void messageCallback(const chatroom::ChatMessage::ConstPtr& msg) {
    // process the incoming msg and display it
    ROS_INFO("[%s] %s", msg->sender.c_str(), msg->message.c_str());
}

int main(int argc, char** argv) {
    // initialize ROS node
    ros::init(argc, argv, "chatroom_node");
    ros::NodeHandle nh;

    // subscribe to the topic
    ros::Subscriber sub = nh.subscribe("/chatroom_topic", 10, messageCallback);

    // process incoming msgs
    ros::spin();

    return 0;
}

