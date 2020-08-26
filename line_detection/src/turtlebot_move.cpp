#include <ros/ros.h>
#include <iostream>
#include "line_detection/direction.h"
#include "geometry_msgs/Twist.h"
using namespace std;
ros::Publisher pub;
void msgCallback(const line_detection::direction msg){
    geometry_msgs::Twist velocity;
    if (msg.dir ==0){
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.15;
        pub.publish(velocity);
        ROS_INFO_STREAM("Turning Left");
        ROS_INFO_STREAM(velocity.linear.x);
    }
    else if (msg.dir == 1){
        velocity.linear.x = 0.15;
        velocity.angular.z = 0;
        pub.publish(velocity);
        ROS_INFO_STREAM("Go Straight");
    }
    else if (msg.dir == 2){
        velocity.linear.x = 0.1;
        velocity.angular.z = -0.15;
        pub.publish(velocity);
        ROS_INFO_STREAM("Turning Right");
    }
    else if (msg.dir == 3){
        velocity.linear.x = 0;
        velocity.angular.z = 0.25;
        pub.publish(velocity);
        ROS_INFO_STREAM("Searching ...");
    }


}

int main(int argc, char** argv){
    ros::init(argc, argv, "turtlebot_move");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("direction", 1, msgCallback);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    ros::spin();
    return 0;
}

