// user_node_1.cpp
#include <ros/ros.h>
#include "chatroom/ChatMessage.h"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "user_node_3");

    // Check if the node initialization was successful
    if (!ros::master::check()) {
        std::cerr << "Failed to initialize ROS node." << std::endl;
        return -1;
    }

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create a publisher to send messages to the chatroom
    ros::Publisher chatroom_pub = nh.advertise<chatroom::ChatMessage>("/chatroom_topic", 10);

    // Loop to read user input and send messages to the chatroom
    std::string message;
    while (ros::ok()) {
        // Prompt the user to enter a message
        std::cout << "Enter message: ";
        std::getline(std::cin, message);

        // Create a ChatMessage object with the user's message
        chatroom::ChatMessage chat_msg;
        chat_msg.sender = "user3"; // Set the sender
        chat_msg.message = message; // Set the message

        // Publish the message to the chatroom
        chatroom_pub.publish(chat_msg);

        // Sleep briefly to allow other nodes to process messages
        ros::Duration(0.1).sleep();
    }

    return 0;
}

