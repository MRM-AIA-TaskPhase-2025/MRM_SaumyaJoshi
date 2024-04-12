#include <ros/ros.h>
#include "chatroom/ChatMessage.h"
#include <iostream>
#include <string>

void messageCallback(const chatroom::ChatMessage::ConstPtr& msg) { 
    // to check if the sender is not user1
    if (msg->sender != "user1") {
        // process the incoming msg and display it
        ROS_INFO("[%s] %s", msg->sender.c_str(), msg->message.c_str()); //c_str() returns a pointer to a null terminated str
    }
}


int main(int argc, char** argv) {  //argc = no. of command line argments, argv= array of strings containing the arguments
    // initialize publisher node
    ros::init(argc, argv, "user_node_1"); //ros namespace, init fn, members accessed by :: scope resolution op

    // to check if node initialization was successful
    if (!ros::master::check()) { // to check availabilty of ros master
        std::cerr << "Failed to initialize ROS node." << std::endl; //cerr for no buffer(immediately prints)
        return -1;
    }

    // creating node handle to comm with ROS
    ros::NodeHandle nh; //registers with ros master, access to parameter server

    // creating publisher to send messages 
    ros::Publisher chatroom_pub = nh.advertise<chatroom::ChatMessage>("/chatroom_topic", 10); //no. of msgs that can be buffered in the publisher's outgoing message queue before older msgs are dropped

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
        chat_msg.sender = "user1"; // set the sender
        chat_msg.message = message; // set the message

        // publishing the message 
        chatroom_pub.publish(chat_msg);

        // sleep briefly to allow other nodes to process msgs
        ros::Duration(0.1).sleep();
    }

    spinner.stop(); // Stop the spinner before exiting

    return 0;
}


