#include "nexus_driver.h"


NexusDriver::NexusDriver(ros::NodeHandle& nh)
: _recvbuf(new int16_t[SERIALBUF_SIZE])
, _sendbuf(new int16_t[SERIALBUF_SIZE])
, _ros_rate(ros::Rate(ROS_RATE))
, _node_handle(nh)
, _bufsize(SERIALBUF_SIZE)
, _baudrate(BAUDRATE)
, _topic_queue_size(TOPIC_QUEUE_SIZE)
, _wheelbase(WHEELBASE)
, _wheeldiam(WHEELDIAM)
, _red_ratio(RED_RATIO)
, _encoder_ppr(ENCODER_PPR)
, _firmw_delta_t(FIRMWARE_DELTA_T)
, _comport_device_name("/dev/ttyUSB0")
{
    _comport_device_fd = openAsComPort(_comport_device_name);
    if (_comport_device_fd == -1)
        throw ros::Exception("nexus_driver : Failed to open comport");

    ROS_INFO_STREAM("nexus_driver : Connection opened successfully.");

    _odom  = _node_handle.advertise<nav_msgs::Odometry>("nexus/odom", _topic_queue_size);
    _twist = _node_handle.subscribe("nexus/cmd_vel", _topic_queue_size, &NexusDriver::set_state_callback, this);
}   


NexusDriver::~NexusDriver() {
    if (_comport_device_fd != 0)
        close(_comport_device_fd);
    
    delete[] _recvbuf;
    delete[] _sendbuf;
    ROS_INFO_STREAM("nexus_driver : Connection closed.");
}


void NexusDriver::setWheelState(int16_t w1_pwm, int16_t w2_pwm) {
    _sendbuf[0] = w1_pwm;
    _sendbuf[1] = w2_pwm;
    
    if (write(_comport_device_fd, _sendbuf, 2 * sizeof(int16_t)) != 2 * sizeof(int16_t))
        throw ros::Exception("nexus_driver : Unable to send state. Maybe connection error.");
    ROS_INFO_STREAM("nexus_driver : Command sent successfully.");
}


void NexusDriver::getWheelState() {
    int nbytes = 0;

    if (read(_comport_device_fd, _recvbuf, 2 * sizeof(int16_t)) != 2 * sizeof(int16_t))
        throw ros::Exception("nexus_driver : Wrong number of bytes was read. Problems..");

    ROS_INFO_STREAM("Wheel state [pulses] : " + std::to_string(_recvbuf[0]) + " " + std::to_string(_recvbuf[1]));
    ROS_INFO_STREAM("Wheel spd   [mmps]   : " + std::to_string(pulses2Spd(_recvbuf[0], 50000)) + " " + std::to_string(pulses2Spd(_recvbuf[1], 50000)));
}


double NexusDriver::pulses2Spd(int16_t pulses, int delta_t) {
    double delta_phi = 2 * PI / (_encoder_ppr * _red_ratio) * pulses;
    double spd_mps   = delta_phi / (delta_t / (double)MILLIS_PER_SEC) * (_wheeldiam / (2.0 * 1000));
    return spd_mps;
}


double NexusDriver::pulses2Dist(int16_t pulses) {
    double delta_phi = 2 * PI / (_encoder_ppr * _red_ratio) * pulses;
    return delta_phi * _wheeldiam / (2.0 * 1000);
}


void NexusDriver::checkOdom() {
    double d1          = pulses2Dist(_odom_w1_ps),
           d2          = pulses2Dist(_odom_w2_ps),
           delta_theta = (d2  - d1) / (_wheelbase / 1000.0);
    
    odom_x += cos(delta_theta) * (d1 + d2) / 2.0;
    odom_y += sin(delta_theta) * (d1 + d2) / 2.0;
    odom_theta += delta_theta;
    odom_linspd = (d1 + d2) / (2.0 * _firmw_delta_t);
    odom_angspd = delta_theta / _firmw_delta_t;

    // ROS_INFO_STREAM("spd   [mmps]   : " + std::to_string(pulses2Spd(_recvbuf[0], 50000)) + " " + std::to_string(pulses2Spd(_recvbuf[1], 50000)));
    ROS_INFO_STREAM("POS | x = " + std::to_string(odom_x) + ", y = " + std::to_string(odom_y) + ", th = " + std::to_string(odom_theta));
    ROS_INFO_STREAM("SPD | lin = " + std::to_string(odom_linspd) + ", ang = " + std::to_string(odom_angspd));

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
    setWheelState(twist->linear.x, twist->linear.y);
}


void pid_callback(const std_msgs::Float64::ConstPtr& effort) {

}
