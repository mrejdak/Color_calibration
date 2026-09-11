#include "color_correction.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "color_calibration/correction");
    ros::NodeHandle nh("~");

    ColorCorrection color_correction(&nh);
    
    ros::spin();
    return 0;
}
