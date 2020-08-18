#include "nexus_driver.h"


#define DEBUG 0


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nexus_driver");
    ros::NodeHandle nh;
    try {
        NexusDriver drv(nh);
        drv.setRate(3);
        for (int i = 0; ros::ok(); i = (i + 1) % 40) {
            try {
                ros::spinOnce();
                drv.sleep();
                drv.getWheelState();
            } catch (ros::Exception& e) {
                ROS_INFO_STREAM(e.what());
                if (DEBUG)
                    return 1;
            }
        }
    } catch (ros::Exception& e) {
        ROS_INFO_STREAM(e.what());
    }
    
    return 0;
}
