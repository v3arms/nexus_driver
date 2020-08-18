#include "nexus_driver.h"


NexusDriver::NexusDriver(ros::NodeHandle& nh, int comport_number)
: _comport_number(comport_number)
, _recvbuf(new int16_t[SERIALBUF_SIZE])
, _sendbuf(new int16_t[SERIALBUF_SIZE])
, _ros_rate(ros::Rate(ROS_RATE))
, _node_handle(nh)
, _bufsize(SERIALBUF_SIZE)
, _baudrate(BAUD_RATE)
, _topic_queue_size(TOPIC_QUEUE_SIZE)
, _wheelbase(WHEELBASE)
, _red_ratio(RED_RATIO)
, _encoder_ppr(ENCODER_PPR)
, _firmw_delta_t(FIRMWARE_DELTA_T)
{
    if (openAsComPort(_comport_device, ))

    if (RS232_OpenComport(comport_number, _baudrate , "8N1", false) == 1)
        throw ros::Exception("nexus_driver : Failed to open comport");
    ROS_INFO_STREAM("nexus_driver : Connection opened successfully.");

    _odom  = _node_handle.advertise<nav_msgs::Odometry>("nexus/odom", _topic_queue_size);
    _twist = _node_handle.subscribe("nexus/cmd_vel", _topic_queue_size, &NexusDriver::set_state_callback, this);
}   


NexusDriver::~NexusDriver() {
    RS232_CloseComport(_comport_number);
    delete[] _recvbuf;
    delete[] _sendbuf;
    ROS_INFO_STREAM("nexus_driver : Connection closed.");
}


void NexusDriver::setWheelState(int16_t w1_pwm, int16_t w2_pwm) {
    _sendbuf[0] = w1_pwm;
    _sendbuf[1] = w2_pwm;
    if (RS232_SendBuf(_comport_number, (unsigned char*)_sendbuf, 2 * sizeof(int16_t)) != 2)
        throw ros::Exception("nexus_driver : Unable to send state. Maybe connection error.");
    ROS_INFO_STREAM("nexus_driver : Command sent successfully.");
}


void NexusDriver::getWheelState() {
    int nbytes = 0;
    if ((nbytes = RS232_PollComport(_comport_number, (unsigned char*)_recvbuf, 4)) != 4) {
        ROS_INFO_STREAM("getState : Not enough bytes. Got " + std::to_string(nbytes));
    }
    usleep(100000);
    _odom_w1_ps =  _recvbuf[0];
    _odom_w2_ps =  _recvbuf[1];
    ROS_INFO_STREAM("Wheel state [pulses] : " + std::to_string(_recvbuf[0]) + " " + std::to_string(_recvbuf[1]));
}


double NexusDriver::pulses2Spd(int16_t pulses, int delta_t) {
    double delta_phi = 2 * PI / (_encoder_ppr * _red_ratio) * pulses;
    double spd_mps   = delta_phi / (delta_t / MILLIS_PER_SEC) * (_wheel_diam / (2.0 * 1000));
    return spd_mps;
}


double NexusDriver::pulses2Dist(int16_t pulses) {
    double delta_phi = 2 * PI / (_encoder_ppr * _red_ratio) * pulses;
    return delta_phi * _wheel_diam / (2.0 * 1000);
}


void NexusDriver::updateOdometry() {
    double d1          = pulses2Dist(_odom_w1_ps),
           d2          = pulses2Dist(_odom_w2_ps),
           delta_theta = (d2  - d1) / (_wheelbase / 1000.0);

    // q = [cos(delta_theta / 2), sin(delta_theta / 2) * v3d]
    _odom_msg.pose.pose.orientation.z += sin(delta_theta / 2.0);
    _odom_msg.pose.pose.orientation.w += cos(delta_theta / 2.0);

    _odom_msg.pose.pose.position.x    += cos(delta_theta) * (d1 + d2) / 2.0;
    _odom_msg.pose.pose.position.y    += sin(delta_theta) * (d1 + d2) / 2.0;

    _odom_msg.twist.twist.linear.x     = (d1 + d2) / (2.0 * _firmw_delta_t);
    _odom_msg.twist.twist.angular.z    = delta_theta / _firmw_delta_t;
}


void NexusDriver::publishOdometry() {

}


void NexusDriver::set_state_callback(const geometry_msgs::Twist::ConstPtr& twist) {

}


void pid_callback(const std_msgs::Float64::ConstPtr& effort) {

}