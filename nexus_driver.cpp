#include "nexus_driver.h"


NexusDriver::NexusDriver(ros::NodeHandle& nh, int comport_number)
: _comport_number(comport_number)
, _serialbuf(new int[SERIALBUF_SIZE]) 
, _ros_rate(ros::Rate(ROS_RATE))
, _node_handle(nh)
, _twist_true(_node_handle.advertise<geometry_msgs::Twist>("nexus_vel_state", TOPIC_QUEUE_SIZE))
, _twist_desired(_node_handle.subscribe("nexus/cmd_vel", TOPIC_QUEUE_SIZE, &NexusDriver::callback, this))
{
    if (RS232_OpenComport(comport_number, BAUD_RATE , "6O2", true) == 1)
        throw ros::Exception("nexus_driver : Failed to open comport");
    ROS_INFO_STREAM("nexus_driver : Connection opened succesfully.");
}   


NexusDriver::~NexusDriver() {
    RS232_CloseComport(_comport_number);
    delete[] _serialbuf;
    ROS_INFO_STREAM("nexus_driver : Connection closed.");
}


void NexusDriver::publishState() {
    _twist_true.publish(_spd_true);
}


void NexusDriver::setStateDesired(const geometry_msgs::Twist::ConstPtr& twist) {
    if (twist->linear.x != 0 && twist->angular.z != 0)
        throw ros::Exception("Complex movement is currently not supported. Set nonzero linear or angular speed, but not both.");
    
    int sendbuf[2];
    sendbuf[0] = (int)twist->linear.x, sendbuf[1] = (int)twist->angular.z;
    if (RS232_SendBuf(_comport_number, (unsigned char*)sendbuf, 2 * sizeof(int)) != 2 * sizeof(int))
        throw ros::Exception("Unable to send state. Connection error.");
    ROS_INFO_STREAM("Command sent succesfully.");
    _spd_true.linear.x  = (double)_serialbuf[0];
    _spd_true.angular.z = (double)_serialbuf[1];
}


void NexusDriver::getState() {
    if (RS232_PollComport(_comport_number, (unsigned char*)_serialbuf, 2 * sizeof(int)) != 2 * sizeof(int))
        throw ros::Exception("Unable to get state. No data received yet.");
    ROS_INFO_STREAM("Got state succesfully.");

}

/*
void NexusDriver::loop(int argc, char *argv[]) {
    ros::init(argc, argv, "nexus_driver");
    while(ros::ok()) {
        updateState();
        ros::spinOnce();
        _ros_rate.sleep();
    }
} */


void NexusDriver::callback(const geometry_msgs::Twist::ConstPtr& twist) {
    setStateDesired(twist);
}
