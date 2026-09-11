#include "color_calibration.h"



ColorCalibration::ColorCalibration(ros::NodeHandle* nh) {
    ros::param::param<std::string>("color_calibration_matrix_path", calibration_matrix_path, "");
    nh->param("gamma", gamma, 2.4f);
    if(calibration_matrix_path.empty()) {
        ROS_ERROR("[COLOR CALIBRATION] 'color_calibration_matrix_path' param not set");
    }
    if(gamma <= 0.0f) {
        ROS_ERROR("[COLOR CALIBRATION] Invalid gamma (%f), using default 2.4", gamma);
        gamma = 2.4f;
    }
    if(CALIBRATION_MATRIX_ROWS == 3 || CALIBRATION_MATRIX_ROWS == 4) {
        std::string input_topic = "/zed2i/zed_node/rgb/image_rect_color";
        std::string output_topic = "/color_calibration/debug";
        nh->param("input_topic", input_topic, input_topic);
        nh->param("output_topic", output_topic, output_topic);

        image_sub = nh->subscribe<sensor_msgs::Image>(input_topic, 1, &ColorCalibration::imageCallback, this);
        chart_pub = nh->advertise<sensor_msgs::Image>(output_topic, 1);
        ROS_INFO("[COLOR CALIBRATION] Initialized (input: %s, output: %s)", input_topic.c_str(), output_topic.c_str());
    }
    else {
        ROS_ERROR("[COLOR CALIBRATION] ERROR: Illegal CALIBRATION_MATRIX_ROWS config");
    }
}

void ColorCalibration::imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
    cv::Mat image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

    cv::Mat image32f;
    image.convertTo(image32f, CV_32F, 1.0 / 255.0);

    // inverse gamma (decode to linear)
    cv::pow(image32f, gamma, image32f);

    cv::Mat debug_image;
    cv::Mat H;
    float confidence = findChartWithConfidence(image, debug_image, H);

    if(confidence < CONFIDENCE_THRESHOLD){
        sensor_msgs::ImagePtr chartDrawn = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
        chartDrawn->header.stamp = image_msg->header.stamp;
        chart_pub.publish(chartDrawn);
        return;
    }

    cv::Mat chart;
    cv::warpPerspective(image32f, chart, H, cv::Size(WARPED_WIDTH, WARPED_HEIGHT));
    
    const std::array<ColorSample, 24> samples = getSamples(chart);

    std::unique_lock<std::mutex> lock{vector_mutex, std::defer_lock};
    lock.lock();

    accepted_samples.push_back(SampleWithConfidence{samples, confidence});

    /* 
        Visualization of warped image with ROI and center points marked
    */
    if(VISUALIZE_PATCHES) {
        lock.unlock();

        cv::Mat chart_vis;
        chart.convertTo(chart_vis, CV_8UC3, 255.0);

        if(chart.size() == cv::Size(620, 400))
        for (const auto& p : samples) {
            cv::rectangle(chart_vis, p.roi, cv::Scalar(0,255,0), 1);
            cv::circle(
                chart_vis,
                p.center,
                5,
                cv::Scalar(
                    p.mean_bgr[0] * 255.0,
                    p.mean_bgr[1] * 255.0,
                    p.mean_bgr[2] * 255.0
                ),
                -1
            );
            cv::circle(chart_vis, p.center, 2, cv::Scalar(0,0,255), -1);
        }
        sensor_msgs::ImagePtr chartImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", chart_vis).toImageMsg();
        chartImage->header.stamp = image_msg->header.stamp;
        chart_pub.publish(chartImage);
        return;
    }

    /*
        Averaging top K of accepted_samples
    */
    if (accepted_samples.size() == TOTAL_SAMPLES) {
        std::sort(accepted_samples.begin(), accepted_samples.end(), 
            [](const SampleWithConfidence& a, const SampleWithConfidence& b){ return a.confidence > b.confidence; });

        accepted_samples.resize(K_BEST);

        // Average samples across all K_BEST samples
        std::array<ColorSample, 24> averaged_samples;
        for(int i = 0; i < 24; i++) {
            cv::Vec3d sum_bgr(0.0, 0.0, 0.0);
            
            for(const auto& sample_with_conf : accepted_samples) {
                sum_bgr += sample_with_conf.samples[i].mean_bgr;
            }
            
            cv::Vec3d avg_bgr = sum_bgr / static_cast<double>(accepted_samples.size());
            
            averaged_samples[i] = ColorSample{
                .row = samples[i].row,
                .col = samples[i].col,
                .center = samples[i].center,
                .roi = samples[i].roi,
                .mean_bgr = avg_bgr
            };
        }

        cv::Mat observed_patches(24, CALIBRATION_MATRIX_ROWS, CV_32F);
        cv::Mat color_calibration_mat;
        
        for(int i = 0; i < 24; i++) {
            // Reverse order as the reference patches are in rgb
            observed_patches.at<float>(i, 0) = averaged_samples[i].mean_bgr[2];
            observed_patches.at<float>(i, 1) = averaged_samples[i].mean_bgr[1];
            observed_patches.at<float>(i, 2) = averaged_samples[i].mean_bgr[0];
            if(CALIBRATION_MATRIX_ROWS == 4) observed_patches.at<float>(i, 3) = 1.0f;
        }

        cv::Mat ref_patches_linear;
        cv::pow(REF_PATCHES, gamma, ref_patches_linear);

        cv::solve(observed_patches, ref_patches_linear, color_calibration_mat, cv::DECOMP_SVD);

        saveCalibrationMatrix(color_calibration_mat);
        lock.unlock();
        
        // stop this node after calibration
        ros::shutdown();
        return;
    }
    lock.unlock();
}

void ColorCalibration::saveCalibrationMatrix(const cv::Mat& color_calibration_mat) {
    cv::FileStorage fs(calibration_matrix_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        ROS_ERROR("[COLOR CALIBRATION] Failed to open file for writing: %s", calibration_matrix_path.c_str());
        return;
    }

    fs << "color_calibration_matrix" << color_calibration_mat;
    fs << "calibration_gamma" << gamma;
    fs.release();

    ROS_INFO("[COLOR CALIBRATION] Calibration matrix saved to: %s", calibration_matrix_path.c_str());
}

static std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point>& quad)
{
    std::vector<cv::Point2f> pts;
    for (const auto& p : quad)
        pts.emplace_back(p);

    std::sort(pts.begin(), pts.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) {
                  return a.y < b.y;
              });

    std::vector<cv::Point2f> top(pts.begin(), pts.begin() + 2);
    std::vector<cv::Point2f> bottom(pts.begin() + 2, pts.end());

    std::sort(top.begin(), top.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) {
                  return a.x < b.x;
              });

    std::sort(bottom.begin(), bottom.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) {
                  return a.x < b.x;
              });

    // TL, TR, BR, BL
    return { top[0], top[1], bottom[1], bottom[0] };
}


bool ColorCalibration::inAspectRatioThreshold(const std::vector<cv::Point>& quad)
{
    cv::RotatedRect r = cv::minAreaRect(quad);
    float w = r.size.width;
    float h = r.size.height;

    float ratio = std::max(w,h) / std::min(w,h);

    return (ratio > MIN_RATIO && ratio < MAX_RATIO);
}

float ColorCalibration::findChartWithConfidence(const cv::Mat& image, cv::Mat& Output, cv::Mat& H) {

    cv::Mat gray, blur, edges;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);
    cv::Canny(blur, edges, 60, 120);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> quads;

    for (const auto& c : contours) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(c, approx, 0.02 * cv::arcLength(c, true), true);

        if (approx.size() != 4) continue;
        if (!cv::isContourConvex(approx)) continue;
        if (!inAspectRatioThreshold(approx)) continue;

        double area = cv::contourArea(approx);
        if (area < 10000) continue;

        quads.push_back(approx);
    }
    
    if (quads.empty()) {
        ROS_WARN_THROTTLE(1.0, "[COLOR CALIBRATION] No board candidate found");
        Output = image;
        return 0.0f;
    }
    
    if(VISUALIZE_CHART){
        /*
            Visualization of image with all accepted rectangles drawn
        */
        cv::Mat debug = image.clone();
        
        for (auto& q : quads) {
            std::vector<std::vector<cv::Point>> tmp;
            tmp.push_back(q);

            cv::drawContours(
                debug,
                tmp,
                -1,
                cv::Scalar(0, 255, 0),
                2
            );
        }
        Output = debug;
        return 0.0f;
    }

    auto bestQuad = *std::max_element(
        quads.begin(), quads.end(),
        [](const std::vector<cv::Point>& a,
           const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        }
    );

    std::vector<cv::Point2f> srcPts = orderCorners(bestQuad);

    cv::Size canonicalSize(WARPED_WIDTH, WARPED_HEIGHT);
    
    std::vector<cv::Point2f> dstPts = {
        {0.f, 0.f},
        {float(canonicalSize.width - 1), 0.f},
        {float(canonicalSize.width - 1), float(canonicalSize.height - 1)},
        {0.f, float(canonicalSize.height - 1)}
    };

    H = cv::getPerspectiveTransform(srcPts, dstPts);

    cv::Mat warped;
    cv::warpPerspective(image, warped, H, canonicalSize);

    Output = warped;

    // confidence score based on:
    // 1) chart aspect ratio closeness to canonical aspect
    // 2) chart area closeness to 1/4 image area
    // 3) perspective distortion (difference in opposite edge lengths)
    const cv::RotatedRect rect = cv::minAreaRect(bestQuad);
    const float rect_w = rect.size.width;
    const float rect_h = rect.size.height;
    const float measured_ratio = std::max(rect_w, rect_h) / std::max(1e-6f, std::min(rect_w, rect_h));
    const float aspect_error = std::abs(measured_ratio - REAL_RATIO) / std::max(REAL_RATIO, 1e-6f);
    const float aspect_score = 1.0f - std::min(1.0f, aspect_error);

    const float image_area = static_cast<float>(image.cols * image.rows);
    const float quad_area = static_cast<float>(cv::contourArea(bestQuad));
    const float measured_area_ratio = quad_area / std::max(image_area, 1e-6f);
    const float target_area_ratio = 0.2f;
    const float sigma = 0.2f;
    const float area_score = std::exp(-std::pow(measured_area_ratio - target_area_ratio, 2) / (2.0f * sigma * sigma));

    const float top_len = cv::norm(srcPts[0] - srcPts[1]);
    const float bottom_len = cv::norm(srcPts[2] - srcPts[3]);
    const float left_len = cv::norm(srcPts[0] - srcPts[3]);
    const float right_len = cv::norm(srcPts[1] - srcPts[2]);

    const float horizontal_distortion = std::abs(top_len - bottom_len) / std::max(std::max(top_len, bottom_len), 1e-6f);
    const float vertical_distortion = std::abs(left_len - right_len) / std::max(std::max(left_len, right_len), 1e-6f);
    const float diag1 = cv::norm(srcPts[0] - srcPts[2]);
    const float diag2 = cv::norm(srcPts[1] - srcPts[3]);
    const float diag_distortion = std::abs(diag1 - diag2) / std::max(std::max(diag1, diag2), 1e-6f);
    const float perspective_score = 1.0f - std::min(1.0f, 0.33f * horizontal_distortion + 0.33f * vertical_distortion + 0.34f * diag_distortion);

    const float confidence =
        0.35f * aspect_score +
        0.4f * area_score +
        0.25f * perspective_score;

    ROS_INFO("[COLOR CALIBRATION] Chart found with confidence: %.3f", confidence);
    return confidence;
}

std::array<ColorSample, 24> ColorCalibration::getSamples(const cv::Mat& warped) {

    std::array<ColorSample, 24> samples;

    for(int i = 0; i < ROWS; i++) {
        for(int j = 0; j < COLS; j++) {
            cv::Point2d center(
                j * (PATCH_WIDTH + PATCH_OFFSET) + HORIZONTAL_OFFSET + PATCH_WIDTH / 2,
                i * (PATCH_HEIGHT + PATCH_OFFSET) + VERTICAL_OFFSET + PATCH_HEIGHT / 2
            );

            cv::Rect ROI(
                static_cast<int>(center.x - PATCH_WIDTH  / 4),
                static_cast<int>(center.y - PATCH_HEIGHT / 4),
                PATCH_WIDTH / 2,
                PATCH_HEIGHT / 2
            );
    
            // Clamp ROI to image bounds
            ROI &= cv::Rect(0, 0, warped.cols, warped.rows);
    
            cv::Scalar mean = cv::mean(warped(ROI));

            samples[i*COLS + j] = ColorSample{.row = i, .col = j, .center = center, .roi = ROI, .mean_bgr = cv::Vec3d(mean[0], mean[1], mean[2])};
        }
    }

    return samples;
}