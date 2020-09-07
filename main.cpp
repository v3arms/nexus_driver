#include "nexus_driver.h"


#define DEBUG 1


int main(int argc, char *argv[]) {
    /*
    ros::init(argc, argv, "nexus_driver");
    ros::NodeHandle nh;
    try {
        NexusDriver *drv = new NexusDriver(nh);
        drv->setRate(25);
        for (int i = 0; ros::ok(); i = (i + 1) % 40) {
            try {
                ros::spinOnce();
                drv->sleep();
                drv->getWheelState();
            } catch (ros::Exception& e) {
                ROS_INFO_STREAM(e.what());
                if (DEBUG) {
                    delete drv;
                    return 1;
                }
            }
        }
        delete drv;
    } catch (ros::Exception& e) {
        ROS_INFO_STREAM(e.what());
        ROS_INFO_STREAM("Cannot close fd!");
    }
    */
    return 0;
}
