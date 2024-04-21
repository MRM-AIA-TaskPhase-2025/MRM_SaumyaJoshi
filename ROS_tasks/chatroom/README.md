A description of the task and the approach undertaken:
This task involves developing a chatroom system where users can view and send messages that all other users in the chatroom can see using ROS in C++.

Step 1: I had previously installed ROS on my computer so I set up a special workspace where I could create my chatroom package.
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

Step 2: I created a chatroom package which contains all my project codes and files.
cd ~/catkin_ws/src
catkin_create_pkg chatroom std_msgs rospy roscpp

Step 3: In the source folder I added my publisher and subscriber nodes codes (written in C++ using ROS)
cd ~/catkin_ws/src/chatroom/src
touch chatroom_node.cpp
nano chatroom_node.cpp
similarly,make usernodes and write the codes in it.

Step 4: I created special types of messages for the chatroom, like messages for chatting with each other and for keeping track of who's in the chatroom. This is in the msg file.

Step 5: I made launch files to help start everything up together, (launch all the node codes)

Step 6: I tried running it multiple times and debugged all the errors after identification.
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roscore
roslaunch chatroom chatroom.launch
rosrun chatroom user_node_1
rosrun chatroom user_node_2
rosrun chatroom user_node_3

ROS Topics published:at
/chatroom_topic

ROS Messages and services used:
Messages: 
std_msgs/String

Services: 
/chatroom_node/get_loggers
/chatroom_node/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/user_node_1/get_loggers
/user_node_1/set_logger_level
/user_node_2/get_loggers
/user_node_2/set_logger_level
/user_node_3/get_loggers
/user_node_3/set_logger_level

RQT Graph
present in folder

A link to a screen-recorded video of the nodes running uploaded on YouTube:
https://youtu.be/yfEaOmuuTJg
