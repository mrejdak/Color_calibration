#include <ros/ros.h>
#include "color_calibration.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "color_calibration/calibration");
    ros::NodeHandle nh("~");

    ColorCalibration Calibration(&nh);
    
    ros::spin();
    return 0;
}
