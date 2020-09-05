#ifndef NEXUS_DRIVER_
#define NEXUS_DRIVER_


#include <ros/ros.h>
#include <ros/exception.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
// #include <boost/bind.hpp>
#include "comport.h"
#include <cmath>


const int SERIALBUF_SIZE   = 1024;
const int ROS_RATE         = 10;
const int TOPIC_QUEUE_SIZE = 1000;
// const char* COMPORT_DEVICE = "/tty/USB0";

const int WHEELBASE        = 280;
const int WHEELDIAM        = 143;
const int RED_RATIO        = 64;
const int ENCODER_PPR      = 24;

const double FIRMWARE_DELTA_T = 0.05;

const int MILLIS_PER_SEC   = 1000000;
const double PI            = 3.14159265359;


class NexusDriver {
public :
    NexusDriver(ros::NodeHandle& nh);
    ~NexusDriver();

    void publishOdometry();
    void updateOdometry();
    double pulses2Spd(int16_t pulses, int delta_t);
    double pulses2Dist(int16_t pulses);

    void sleep()              {_ros_rate.sleep();}
    void setRate(int rate_hz) {_ros_rate = ros::Rate(rate_hz);}
    void setBaudRate(int rate_baud) {}
    void setWheelState(int16_t w1_pwm, int16_t w2_pwm);

    void getWheelState();
    int  getRate();
    
private :
    ros::NodeHandle&     _node_handle;
    ros::Subscriber      _twist;
    ros::Publisher       _odom;
    ros::Publisher       _to_pid[2];
    ros::Rate            _ros_rate;

    ros::Subscriber      _from_pid_float[2];
    ros::Publisher       _float2odom[2];
    message_filters::Subscriber<nav_msgs::Odometry> _w1_from_pid_odom, _w2_from_pid_odom;
    message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> _from_pid_sync;
    // Use odom or smth with header instead this*

    nav_msgs::Odometry   _odom_msg;

    int16_t             *_recvbuf, *_sendbuf;
    int                  _comport_number;
    int                  _bufsize;
    int                  _baudrate;
    int                  _topic_queue_size;

    int                  _wheelbase;
    int                  _red_ratio;
    int                  _encoder_ppr;
    int                  _wheeldiam;
    int16_t              _cur_pulses[2];
    double               _firmw_delta_t;
    
    const char*          _comport_device_name;
    int                  _comport_device_fd;

    double odom_x, odom_y, odom_theta, odom_linspd, odom_angspd;
    void checkOdom();

    void w1_from_pid_float(const std_msgs::Float64::ConstPtr& pwm);
    void w2_from_pid_float(const std_msgs::Float64::ConstPtr& pwm);

    void sendPwm(const nav_msgs::Odometry::ConstPtr& w1_pwm, const nav_msgs::Odometry::ConstPtr& w2_pwm);

};


#endif
