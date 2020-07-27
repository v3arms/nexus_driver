#pragma once


#include <ros/ros.h>
#include <ros/exception.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>


#include "rs232.h"


const int SERIALBUF_SIZE   = 1024;
const int ROS_RATE         = 10;
const int BAUD_RATE        = 19200;
const int TOPIC_QUEUE_SIZE = 1000;
const int TTYUSB0_COM_PORT = 16;
const int NEXUS_WHEELSPAN  = 250;
const int NEXUS_MAX_LINSPD = 127;
const int NEXUS_MAX_ANGSPD = 127;
const int SENDBUF_SIZE     = 3; 


class NexusDriver {
public :
    NexusDriver(ros::NodeHandle& nh, int comport_number = TTYUSB0_COM_PORT);
    ~NexusDriver();
    void setStateDesired(const geometry_msgs::Twist::ConstPtr& twist);
    void publishState();
    void getState();

    void sleep()              {_ros_rate.sleep();}
    void setRate(int rate_hz) {_ros_rate = ros::Rate(rate_hz);}
    
private :
    ros::NodeHandle& _node_handle;
    ros::Subscriber _twist_desired;
    ros::Publisher  _twist_true;
    ros::Rate       _ros_rate;

    geometry_msgs::Twist _spd_true;
    geometry_msgs::Twist _spd_desired; 
    char           *_serialbuf, *_sendbuf;
    int             _comport_number;

    void callback(const geometry_msgs::Twist::ConstPtr& twist);

};
