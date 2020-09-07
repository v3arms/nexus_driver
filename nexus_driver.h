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
#include "comport.h"
#include <cmath>
#include "nexus_specs.h"

const int ROS_RATE         = 10;
const int TOPIC_QUEUE_SIZE = 1000;
const int MILLIS_PER_SEC   = 1000000;
const double PI            = 3.14159265359;

enum Wheel {
    Left,
    Right
};

namespace mf = message_filters;


class WheelSpecs {
public:
    WheelSpecs(double base, double diam, int redRatio, int encoderPPR);
    // WheelSpecs(const char* filename);
    double pulses2Spd(int pulses, int delta_t_ms) const;
    double pulses2Dist(int pulses) const;

private:
    const double base;
    const double diam;
    const int redRatio;
    const int encoderPPR;
};


/* 
    provides setup for reading/writing batches via serial port
    batch is read/written in synchronous mode (block until read/write <size> bytes)
    vtime - max blocking time, read()(write()) returns after vtime seconds
    vmin  - min number of bytes to unblock
*/
class SerialConnection {
public:
    SerialConnection(const char* device, int vtime = 10, int vmin = 4, int baudrate = BAUDRATE);
    ~SerialConnection();

    int readBatch(void* recv)  const;
    int writeBatch(void* send) const;
    // void setBatchSize(size_t size);
private:
    const int vtime;
    const int vmin; 
    const int baudRate;
    const int comportFd;

};


class NexusDriver {
public:
    NexusDriver(const WheelSpecs* whl, const SerialConnection* con, size_t topicSize = 1000);
    ~NexusDriver();
    void run(size_t delta_t);
    
private:
    const WheelSpecs *wheel;
    const SerialConnection *serial;
    size_t deltaT;
    
    const size_t topicSize;

    ros::NodeHandle nh;
    ros::Rate rate;
    ros::Publisher effortAsOdomPub1, effortAsOdomPub2;
    ros::Publisher wheelState1, wheelState2;
    ros::Subscriber controlEffort1, controlEffort2;
    mf::Subscriber<nav_msgs::Odometry> effortAsOdomSub1, effortAsOdomSub2;
    mf::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> synchronizer;

    void controlEffortCallback(const std_msgs::Float64::ConstPtr& pwm, const Wheel& wheel);
    void sendToSerial(const nav_msgs::Odometry::ConstPtr& w1_pwm, const nav_msgs::Odometry::ConstPtr& w2_pwm);
    void getWheelState();
};


#endif
