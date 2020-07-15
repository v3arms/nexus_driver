#include "nexus_driver.h"


#define DEBUG 0


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nexus_driver");
    ros::NodeHandle nh;
    NexusDriver drv(nh);
    drv.setRate(10);
    for (int i = 0; ros::ok(); i = (i + 1) % 40) {
        try {
            if (i == 0) 
                drv.publishState();
            ros::spinOnce();
            drv.sleep();
            drv.getState();
        } catch (ros::Exception& e) {
            ROS_INFO_STREAM(e.what());
            if (DEBUG)
                return 1;
        }
    }
    return 0;
}
