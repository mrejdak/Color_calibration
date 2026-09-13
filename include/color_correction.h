#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>


class ColorCorrection {
public:
    ColorCorrection(ros::NodeHandle* nh);
    ~ColorCorrection();
    void imageCallback(const sensor_msgs::ImageConstPtr&);
    cv::Mat readColorCalibrationMatrix();
private:
    void processingLoop();
    void processFrame(const sensor_msgs::ImageConstPtr& image_msg);

    ros::Subscriber image_sub;
    ros::Publisher chart_pub;
    
    cv::Mat color_calibration_mat;
    std::string calibration_matrix_path;

    sensor_msgs::ImageConstPtr latest_image_msg;
    std::mutex latest_image_mutex;
    std::condition_variable latest_image_cv;
    std::thread processing_thread;
    bool has_pending_image = false;
    bool stop_processing = false;
    
    float gamma;
};