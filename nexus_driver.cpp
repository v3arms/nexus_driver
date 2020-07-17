#include "nexus_driver.h"


NexusDriver::NexusDriver(ros::NodeHandle& nh, int comport_number)
: _comport_number(comport_number)
, _serialbuf(new char[SERIALBUF_SIZE]) 
, _sendbuf(new char[SENDBUF_SIZE])
, _ros_rate(ros::Rate(ROS_RATE))
, _node_handle(nh)
, _twist_true(_node_handle.advertise<geometry_msgs::Twist>("nexus/cur_vel", TOPIC_QUEUE_SIZE))
, _twist_desired(_node_handle.subscribe("nexus/cmd_vel", TOPIC_QUEUE_SIZE, &NexusDriver::callback, this))
{
    if (RS232_OpenComport(comport_number, BAUD_RATE , "8N1", false) == 1)
        throw ros::Exception("nexus_driver : Failed to open comport");
    ROS_INFO_STREAM("nexus_driver : Connection opened succesfully.");
}   


NexusDriver::~NexusDriver() {
    RS232_CloseComport(_comport_number);
    delete[] _serialbuf;
    delete[] _sendbuf;
    ROS_INFO_STREAM("nexus_driver : Connection closed.");
}


void NexusDriver::publishState() {
    _twist_true.publish(_spd_true);
}


void NexusDriver::setStateDesired(const geometry_msgs::Twist::ConstPtr& twist) {
    if (twist->linear.x != 0 && twist->angular.z != 0)
        throw ros::Exception("Complex movement is currently not supported. Set nonzero linear or angular speed, but not both.");
    
    _sendbuf[0] = static_cast<unsigned char>(twist->linear.x);
    _sendbuf[1] = static_cast<unsigned char>(twist->angular.z);
    if (RS232_SendBuf(_comport_number, (unsigned char*)_sendbuf, 2) != 2)
        throw ros::Exception("Unable to send state. Maybe connection error.");
    usleep(50000);
    ROS_INFO_STREAM("Command sent succesfully.");
}


/*
void NexusDriver::getState() {
    if (RS232_PollComport(_comport_number, (unsigned char*)_serialbuf, 2) != 2) {
        ROS_INFO_STREAM("getState : Not enough bytes. Clearing RX");
        usleep(50000);
        RS232_flushRX(_comport_number);
    }
    _spd_true.linear.x  = (double)_serialbuf[0];
    _spd_true.angular.z = (double)_serialbuf[1];
    ROS_INFO_STREAM("Got state succesfully.");
}
*/


void NexusDriver::callback(const geometry_msgs::Twist::ConstPtr& twist) {
    setStateDesired(twist);
}
