#include "nexus_driver.h"


SerialConnection::SerialConnection(const char* device, int vtime, int vmin, int baudrate)
: vtime(vtime)
, vmin(vmin)
, baudRate(baudrate)
, comportFd(openAsComPort(device, vtime, vmin, baudrate))
{
    if (comportFd == -1)
        throw std::runtime_error("SerialConnection | Failed to open device.");
}


SerialConnection::~SerialConnection()
{
    close(comportFd);
}


int SerialConnection::readBatch(void* recv) const
{
    return read(comportFd, recv, vmin);
}


int SerialConnection::writeBatch(void* send) const
{
    return write(comportFd, send, vmin);
}


WheelSpecs::WheelSpecs(double base, double diam, int redRatio, int encoderPPR)
: base(base)
, diam(diam)
, redRatio(redRatio)
, encoderPPR(encoderPPR)
{}


double WheelSpecs::pulses2Spd(int pulses, int delta_t) const
{
    double delta_phi = 2 * PI / (encoderPPR * redRatio) * pulses;
    double spd_mps   = delta_phi / (delta_t / (double)MILLIS_PER_SEC) * (diam / (2.0 * 1000));
    return spd_mps;
}


double WheelSpecs::pulses2Dist(int pulses) const
{
    double delta_phi = 2 * PI / (encoderPPR * redRatio) * pulses;
    return delta_phi * diam / (2.0 * 1000);
}


NexusDriver::NexusDriver(const WheelSpecs* whl, const SerialConnection* con, size_t topicSize)
: wheel(whl)
, serial(con)
, topicSize(topicSize)
, nh(ros::NodeHandle())
, effortAsOdomPub1(nh.advertise<nav_msgs::Odometry>("nexus/left_wheel/effortAsOdom", topicSize))
, effortAsOdomPub2(nh.advertise<nav_msgs::Odometry>("nexus/right_wheel/effortAsOdom", topicSize))
, wheelState1(nh.advertise<std_msgs::Float64>("nexus/left_wheel/state", topicSize))
, wheelState2(nh.advertise<std_msgs::Float64>("nexus/right_wheel/state", topicSize))
, controlEffort1(nh.subscribe<std_msgs::Float64> 
        (
            "nexus/left_wheel/control_effort", 
            topicSize, 
            boost::bind(&NexusDriver::controlEffortCallback, this, _1, Wheel::Left)
        ))
, controlEffort2(nh.subscribe<std_msgs::Float64> 
        (
            "nexus/right_wheel/control_effort", 
            topicSize, 
            boost::bind(&NexusDriver::controlEffortCallback, this, _1, Wheel::Right)
        ))
, effortAsOdomSub1(nh, "nexus/left_wheel/effortAsOdom", 1)
, effortAsOdomSub2(nh, "nexus/right_wheel/effortAsOdom", 1)
, synchronizer(effortAsOdomSub1, effortAsOdomSub2, 10)
, rate(ROS_RATE)
{
    synchronizer.registerCallback(boost::bind(&NexusDriver::sendToSerial, this, _1, _2));
}


NexusDriver::~NexusDriver()
{
    ROS_DEBUG_STREAM("NexusDriver | Connection closed.");
}

void NexusDriver::controlEffortCallback(const std_msgs::Float64::ConstPtr& pwm, const Wheel& wheel)
{
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = pwm->data;
    if (wheel == Wheel::Left)
        effortAsOdomPub1.publish(msg);
    if (wheel == Wheel::Right)
        effortAsOdomPub2.publish(msg);
}


void NexusDriver::sendToSerial(const nav_msgs::Odometry::ConstPtr& w1_pwm, const nav_msgs::Odometry::ConstPtr& w2_pwm)
{
    int16_t buf[2] = {
        static_cast<int16_t>(w1_pwm->pose.pose.position.x), 
        static_cast<int16_t>(w2_pwm->pose.pose.position.x)
    };
    int c = serial->writeBatch(buf);
    if (c != 4)
        throw std::runtime_error("NexusDriver::sendEffort | write batch error : " + std::to_string(c) + " bytes was written instead of 4");

#ifndef DEBUG
    ROS_DEBUG_STREAM("ToSerial   | " << buf[0] << " " << buf[1]);
#endif
}


void NexusDriver::getWheelState()
{
    int16_t buf[2] = {};

    int c = serial->readBatch(buf);
    if (c != 4)
        throw std::runtime_error("NexusDriver::getWheelState | read batch error : " + std::to_string(c) + " bytes was read instead of 4");

    std_msgs::Float64 m0, m1;
    m0.data = wheel->pulses2Spd(buf[0], deltaT);
    m1.data = wheel->pulses2Spd(buf[1], deltaT);
    wheelState1.publish(m0);
    wheelState2.publish(m1);
#ifndef DEBUG
    ROS_DEBUG_STREAM("FromSerial | " << buf[0] << " " << buf[1]);
#endif
}


void NexusDriver::run(size_t delta_t_ms)
{
    rate = ros::Rate(1000.0 / delta_t_ms);
    deltaT = delta_t_ms;

    while(true)
    {
#ifndef DEBUG
        try
        {
            getWheelState();
        }
        catch (const std::runtime_error& e)
        {
            ROS_ERROR_STREAM(e.what());
        }
#else
        getWheelState();
#endif
        rate.sleep();
    }
}


/*
void NexusDriver::checkOdom() {
    double d1          = pulses2Dist(_cur_pulses[0]),
           d2          = pulses2Dist(_cur_pulses[1]),
           delta_theta = (d2  - d1) / (_wheelbase / 1000.0);
    
    odom_x += cos(delta_theta) * (d1 + d2) / 2.0;
    odom_y += sin(delta_theta) * (d1 + d2) / 2.0;
    odom_theta += delta_theta;
    odom_linspd = (d1 + d2) / (2.0 * _firmw_delta_t);
    odom_angspd = delta_theta / _firmw_delta_t;

    // ROS_INFO_STREAM("spd   [mmps]   : " + std::to_string(pulses2Spd(_recvbuf[0], 50000)) + " " + std::to_string(pulses2Spd(_recvbuf[1], 50000)));
    ROS_DEBUG_STREAM("ODOM_POS | x = " + std::to_string(odom_x) + ", y = " + std::to_string(odom_y) + ", th = " + std::to_string(odom_theta));
    ROS_DEBUG_STREAM("ODOM_SPD | lin = " + std::to_string(odom_linspd) + ", ang = " + std::to_string(odom_angspd));

}


void NexusDriver::updateOdometry() {
    double d1          = pulses2Dist(_cur_pulses[0]),
           d2          = pulses2Dist(_cur_pulses[1]),
           delta_theta = (d2  - d1) / (_wheelbase / 1000.0);

    // q = [cos(delta_theta / 2), sin(delta_theta / 2) * v3d]
    _odom_msg.pose.pose.orientation.z += sin(delta_theta / 2.0);
    _odom_msg.pose.pose.orientation.w += cos(delta_theta / 2.0);

    _odom_msg.pose.pose.position.x    += cos(delta_theta) * (d1 + d2) / 2.0;
    _odom_msg.pose.pose.position.y    += sin(delta_theta) * (d1 + d2) / 2.0;

    _odom_msg.twist.twist.linear.x     = (d1 + d2) / (2.0 * _firmw_delta_t);
    _odom_msg.twist.twist.angular.z    = delta_theta / _firmw_delta_t;
}


/*