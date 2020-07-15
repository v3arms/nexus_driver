#pragma once


#include <ros/ros.h>
#include <ros/exception.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>


#include "rs232.h"


const int SERIALBUF_SIZE   = 1024; //ints
const int ROS_RATE         = 10;
const int BAUD_RATE        = 19200;
const int TOPIC_QUEUE_SIZE = 1000;
const int TTYUSB0_COM_PORT = 16;
const int NEXUS_WHEELSPAN  = 250;


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
    int*            _serialbuf;
    int             _comport_number;

    void callback(const geometry_msgs::Twist::ConstPtr& twist);

};


/*
int bytesToInt(unsigned char bytes[], int offset) {
    int ret = 0;
    for (int i = offset; i < offset + 4; i++) {
        ret <<= 8;
        ret |= (int)bytes[i] & 0xFF;
    }
    return ret;
}
*/
