#define DEBUG 


#include "nexus_driver.h"
#include <iostream>


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nexus_driver");
    SerialConnection *con;
    try {
        con = new SerialConnection("/dev/ttyUSB0", 1, 4);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 0;
    }
    WheelSpecs w(
        static_cast<double>(WHEELBASE),
        static_cast<double>(WHEELDIAM),
        RED_RATIO,
        ENCODER_PPR
    );
    NexusDriver driver(&w, con);
    
        
    driver.run(20000);
    delete con;
    return 0;
}
