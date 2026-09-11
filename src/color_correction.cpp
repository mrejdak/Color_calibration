#include "color_correction.h"


ColorCorrection::ColorCorrection(ros::NodeHandle* nh) {
    ros::param::param<std::string>("color_calibration_matrix_path", calibration_matrix_path, "");
    nh->param("gamma", gamma, 2.4f);
    if(calibration_matrix_path.empty()) {
        ROS_ERROR("[COLOR CORRECTION] 'color_calibration_matrix_path' param not set");
    }
    if(gamma <= 0.0f) {
        ROS_ERROR("[COLOR CORRECTION] Invalid gamma (%f), using default 2.4", gamma);
        gamma = 2.4f;
    }
    color_calibration_mat = readColorCalibrationMatrix();

    std::string input_topic = "/zed2i/zed_node/rgb/image_rect_color";
    std::string output_topic = "/color_correction/corrected_image";
    nh->param("input_topic", input_topic, input_topic);
    nh->param("output_topic", output_topic, output_topic);

    image_sub = nh->subscribe<sensor_msgs::Image>(
        input_topic,
        1,
        &ColorCorrection::imageCallback,
        this,
        ros::TransportHints().tcpNoDelay(true));
    chart_pub = nh->advertise<sensor_msgs::Image>(output_topic, 1);
    processing_thread = std::thread(&ColorCorrection::processingLoop, this);
    ROS_INFO("[COLOR CORRECTION] Initialized (input: %s, output: %s)", input_topic.c_str(), output_topic.c_str());
}

ColorCorrection::~ColorCorrection() {
    {
        std::lock_guard<std::mutex> lock(latest_image_mutex);
        stop_processing = true;
    }
    latest_image_cv.notify_one();
    if (processing_thread.joinable()) {
        processing_thread.join();
    }
}

void ColorCorrection::imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
    {
        std::lock_guard<std::mutex> lock(latest_image_mutex);
        latest_image_msg = image_msg;
        has_pending_image = true;
    }
    latest_image_cv.notify_one();
}

void ColorCorrection::processingLoop() {
    while (ros::ok()) {
        sensor_msgs::ImageConstPtr image_msg;

        {
            std::unique_lock<std::mutex> lock(latest_image_mutex);
            latest_image_cv.wait(lock, [this]() { return stop_processing || has_pending_image; });

            if (stop_processing) {
                break;
            }

            image_msg = latest_image_msg;
            has_pending_image = false;
        }

        if (image_msg) {
            processFrame(image_msg);
        }
    }
}

void ColorCorrection::processFrame(const sensor_msgs::ImageConstPtr& image_msg) {
    cv::Mat image = cv_bridge::toCvShare(image_msg, "bgr8")->image;

    if (color_calibration_mat.size() != cv::Size(3, 3) && color_calibration_mat.size() != cv::Size(3, 4)) {
        ROS_ERROR_THROTTLE(20, "[COLOR CORRECTION] Calibration matrix has wrong size, cannot apply correction");
        chart_pub.publish(image_msg);
        return;
    }

    cv::Mat img_rgb32f;
    image.convertTo(img_rgb32f, CV_32FC3, 1.0 / 255.0);
    cv::cvtColor(img_rgb32f, img_rgb32f, cv::COLOR_BGR2RGB);

    cv::pow(img_rgb32f, gamma, img_rgb32f);

    cv::Mat corrected_rgb32f;
    cv::transform(img_rgb32f, corrected_rgb32f, color_calibration_mat);

    cv::min(corrected_rgb32f, 1.0, corrected_rgb32f);
    cv::max(corrected_rgb32f, 0.0, corrected_rgb32f);

    cv::pow(corrected_rgb32f, 1.0 / gamma, corrected_rgb32f);

    cv::Mat corrected_bgr8;
    corrected_rgb32f.convertTo(corrected_bgr8, CV_8UC3, 255.0);
    cv::cvtColor(corrected_bgr8, corrected_bgr8, cv::COLOR_RGB2BGR);

    sensor_msgs::ImagePtr correctedImg = cv_bridge::CvImage(image_msg->header, "bgr8", corrected_bgr8).toImageMsg();
    chart_pub.publish(correctedImg);
}


cv::Mat ColorCorrection::readColorCalibrationMatrix() {
    cv::Mat color_calibration_mat;

    cv::FileStorage fs(calibration_matrix_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("[COLOR CORRECTION] Failed to open file for reading: %s", calibration_matrix_path.c_str());
        return color_calibration_mat;
    }

    fs["color_calibration_matrix"] >> color_calibration_mat;

    cv::FileNode gamma_node = fs["calibration_gamma"];
    if (!gamma_node.empty()) {
        float calibration_gamma = 0.0f;
        gamma_node >> calibration_gamma;
        if (calibration_gamma <= 0.0f) {
            ROS_ERROR("[COLOR CORRECTION] Invalid calibration_gamma in file: %s", calibration_matrix_path.c_str());
            fs.release();
            return cv::Mat();
        }
        if (std::abs(calibration_gamma - gamma) > 1e-3f) {
            ROS_ERROR("[COLOR CORRECTION] Gamma mismatch (matrix=%.3f, runtime=%.3f). Re-run calibration with the same gamma.", calibration_gamma, gamma);
            fs.release();
            return cv::Mat();
        }
    } else {
        ROS_WARN("[COLOR CORRECTION] Loaded legacy matrix without calibration_gamma metadata. Recalibration is recommended.");
    }

    fs.release();

    if (color_calibration_mat.empty()) {
        ROS_ERROR("[COLOR CORRECTION] Calibration matrix not found in file: %s", calibration_matrix_path.c_str());
    } else if (color_calibration_mat.size() != cv::Size(3, 3) && color_calibration_mat.size() != cv::Size(3, 4)) {
        ROS_ERROR("[COLOR CORRECTION] Calibration matrix must be 3x3 or 3x4, got %dx%d", color_calibration_mat.rows, color_calibration_mat.cols);
        color_calibration_mat.release();
    } else {
        if (color_calibration_mat.type() != CV_32F) {
            color_calibration_mat.convertTo(color_calibration_mat, CV_32F);
        }
        ROS_INFO("[COLOR CORRECTION] Calibration matrix loaded from: %s", calibration_matrix_path.c_str());
    }

    return color_calibration_mat;
}
