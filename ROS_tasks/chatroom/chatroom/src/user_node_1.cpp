#include <ros/ros.h>
#include "chatroom/ChatMessage.h"
#include <iostream>
#include <string>

void messageCallback(const chatroom::ChatMessage::ConstPtr& msg) {
    // Check if the sender is not user3 (user_node_3)
    //if (msg->sender != "user1") {
        // Process the incoming msg and display it
        ROS_INFO("[%s] %s", msg->sender.c_str(), msg->message.c_str());
   // }
}

int main(int argc, char** argv) {
    // initialize publisher node
    ros::init(argc, argv, "user_node_1");

    // to check if node initialization was successful
    if (!ros::master::check()) {
        std::cerr << "Failed to initialize ROS node." << std::endl;
        return -1;
    }

    // creating node handle to comm with ROS
    ros::NodeHandle nh;

    // creating publisher to send messages 
    ros::Publisher chatroom_pub = nh.advertise<chatroom::ChatMessage>("/chatroom_topic", 10);

    // subscribe to the topic
    ros::Subscriber sub = nh.subscribe("/chatroom_topic", 10, messageCallback);

    // process incoming msgs
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    // to read user input and send messages to chatroom
    std::string message;
    
    while (ros::ok()) {
        std::cout << "Enter message: ";
        std::getline(std::cin, message);

        // creating ChatMessage object 
        chatroom::ChatMessage chat_msg;
        chat_msg.sender = "user1"; // Set the sender
        chat_msg.message = message; // Set the message

        // publishing the message 
        chatroom_pub.publish(chat_msg);

        // sleep briefly to allow other nodes to process msgs
        ros::Duration(0.1).sleep();
    }

    spinner.stop(); // Stop the spinner before exiting

    return 0;
}
