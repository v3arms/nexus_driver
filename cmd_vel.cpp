#include <ros/ros.h>
#include <ros/exception.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <cmath>


const int topicSize = 1000;
const int rate      = 60;


void cmd_vel_callback_dummy(const geometry_msgs::Twist::ConstPtr& twist, ros::Publisher& pub_l, ros::Publisher& pub_r) 
{
    std_msgs::Float64 wl, wr;
    wl.data = twist->linear.x;
    wr.data = twist->linear.y;
    pub_l.publish(wl);
    pub_r.publish(wr);
}


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "nexus/cmd_vel");
    // ros::Rate rate(60);

    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub_l = nh.advertise<geometry_msgs::Twist>("nexus/left_wheel/desired", topicSize);
    ros::Publisher cmd_vel_pub_r = nh.advertise<geometry_msgs::Twist>("nexus/right_wheel/desired", topicSize);
    
    ros::Subscriber cmd_vel = nh.subscribe<geometry_msgs::Twist>(
        "nexus/cmd_vel", 
        topicSize, 
        boost::bind(&cmd_vel_callback_dummy, _1, cmd_vel_pub_l, cmd_vel_pub_r)
    );

    ros::spin();
}


