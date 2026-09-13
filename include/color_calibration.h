// ROS
#include <ros/ros.h>

// CV2
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

#include <mutex>
#include <cmath>

struct Patch {
    cv::Point2f center;
    std::vector<cv::Point> contour;
};

struct ColorSample
{
    int row;                // 0..3
    int col;                // 0..5
    // center and roi are stored for visualization and debugging
    cv::Point2f center;     // in warped image coordinates
    cv::Rect roi;           // sampling region
    cv::Vec3d mean_bgr;     // mean color (double)
};

struct SampleWithConfidence
{
    std::array<ColorSample, 24> samples;
    float confidence;
};

class ColorCalibration {
public:
    ColorCalibration(ros::NodeHandle* nh);
    void imageCallback(const sensor_msgs::ImageConstPtr&);
    float findChartWithConfidence(const cv::Mat&, cv::Mat&, cv::Mat&);
    bool inAspectRatioThreshold(const std::vector<cv::Point>&);
    std::array<ColorSample, 24> getSamples(const cv::Mat&);
    void saveCalibrationMatrix(const cv::Mat&);
private:

    ros::Subscriber image_sub;
    ros::Publisher chart_pub;
    std::string calibration_matrix_path;

    int calibration_matrix_rows;
    int k_best;
    int total_samples;
    float confidence_threshold;
    float gamma;
    bool visualize_patches;
    bool visualize_chart;

    /*
    Color chart layout in patches
    */
    const int ROWS = 4;
    const int COLS = 6;

    /*
    Acceptable dimentions ratio of a color chart (rectangle)
    */
    const float MIN_RATIO = 1.4;
    const float REAL_RATIO = 1.55;
    const float MAX_RATIO = 1.7;
    
    /*
    Dimentions of color chart after warping (ratio of 1.55)
    */
    const int WARPED_WIDTH = 620;
    const int WARPED_HEIGHT = 400;
    
    /*
    Offsets from chart edges to color patches (values after warping)
    */
    const int VERTICAL_OFFSET = 25;
    const int HORIZONTAL_OFFSET = 30;

    /*
    Dimentions of one color patch on color chart (after warping)
    */
    const int PATCH_HEIGHT = 75;
    const int PATCH_WIDTH = 80;
    const int PATCH_OFFSET = 15; // offset between two patches

    
    /*
    Reference colors of patches    
    */
    cv::Mat REF_PATCHES = (cv::Mat_<float>(24, 3) <<
    // ROW 0
    115.f,  82.f,  68.f,
    194.f, 150.f, 130.f,
     98.f, 122.f, 157.f,
     87.f, 108.f,  67.f,
    133.f, 128.f, 177.f,
    103.f, 189.f, 170.f,

    // ROW 1
    214.f, 126.f,  44.f,
     80.f,  91.f, 166.f,
    193.f,  90.f,  99.f,
     94.f,  60.f, 108.f,
    157.f, 188.f,  64.f,
    224.f, 163.f,  46.f,

    // ROW 2
     56.f,  61.f, 150.f,
     70.f, 148.f,  73.f,
    175.f,  54.f,  60.f,
    231.f, 199.f,  31.f,
    187.f,  86.f, 149.f,
     8.f, 133.f, 161.f,

    // ROW 3
    243.f, 243.f, 242.f,
    200.f, 200.f, 200.f,
    160.f, 160.f, 160.f,
    122.f, 122.f, 121.f,
     85.f,  85.f,  85.f,
    52.f,  52.f,  52.f
    ) * (1.0f / 255.0f);

    /*
    Vector for accepted samples
    */
    std::vector<SampleWithConfidence> accepted_samples;
    std::mutex vector_mutex;
    
};