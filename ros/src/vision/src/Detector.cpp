#include "Detector.hpp"
#include "CameraInput.hpp"
#include "Distance.hpp"
#include "ros/console.h"

Detector::Detector(CameraInput& camera_input)
    :   input(camera_input),
        x(0),
        y(0),
        z(0),
        angle(0.0),
        object_found(false),
        accuracy(0)
{}

Detector::~Detector()
{}

/**
 * Global setup for all detection methods. Should
 * be in the same order as defined inn header. Break
 * out if anything fails.
 * @return true if all setups are successful.
 * @return false if anything fails.
 */
bool Detector::setup()
{
    // Color

    // Match
    match_image = cv::imread("/var/polaris/ros/src/vision/images/person.png", cv::IMREAD_GRAYSCALE);
    if(match_image.empty()) {
        ROS_INFO("Empty match image!");
        return false;
    }
    surf = cv::xfeatures2d::SURF::create(MIN_HESSIAN);
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    surf->detectAndCompute(match_image, cv::noArray(), image_points, image_descriptors);

    // Cascade

    // Line


    return true;
}

 /**
 * Calls detection methods, compairs results, and
 * updates values accordingly.
 * @return true if nothing fails.
 * @return false if anyting fails.
 */
bool Detector::update()
{
    detectColor();
    detectMatch();
    detectCascade();
    detectLine();
    
    return true;
}

bool Detector::detectColor()
{
    return true;
}

/**
 * Feature matching using opencv methods SURF and KNN. Compairs descriptors taken
 * from a source image to the current frame.The order of steps taken on a successful
 * detection are:
 *  1. In setup, detect key points of source image using SURF.
 *  2. Detect key points of current frame using SURF.
 *  3. Compair descriptors and generate matches using KNN.
 *  3. Filter matches using Lowe's ratio test
 *  4. Get the perspective transform between the two sets of points.
 *  5. Fix the scaling and orientation of matching points.
 *  6.  
 * @return true 
 * @return false 
 */
bool Detector::detectMatch()
{
    cv::Mat frame_gray, frame_descriptors;
    std::vector<cv::KeyPoint> frame_points;
    frame_gray = input.getFrameFront();
    cv::cvtColor(input.getFrameFront(), frame_gray, cv::COLOR_BGR2GRAY);
    surf->detectAndCompute(frame_gray, cv::noArray(), frame_points, frame_descriptors);

    if(!frame_descriptors.empty())
    {
        std::vector<std::vector<cv::DMatch>> knn_matches;
        std::vector<cv::DMatch> good_matches;
        matcher->knnMatch(image_descriptors, frame_descriptors, knn_matches, 2);

        for(uint32_t i=0; i<knn_matches.size(); i++) {
            if(knn_matches[i][0].distance < RATIO_THRESH * knn_matches[i][1].distance) {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
    
        if(good_matches.size() > MIN_MATCH_THRESH)
        {
            std::vector<cv::Point2f> src_points;
            std::vector<cv::Point2f> dst_points;
            for(cv::DMatch d : good_matches) {
                src_points.push_back(image_points[d.queryIdx].pt);
                dst_points.push_back(frame_points[d.trainIdx].pt);  
            }
            cv::Mat H = cv::findHomography(src_points, dst_points, cv::RANSAC);

            if (!H.empty())
            {
                std::vector<cv::Point2f> obj_corners(4);
                obj_corners[0] = cvPoint(0,0);
                obj_corners[1] = cvPoint(match_image.cols, 0);
                obj_corners[2] = cvPoint(match_image.cols, match_image.rows);
                obj_corners[3] = cvPoint(0, match_image.rows);
                std::vector<cv::Point2f> scene_corners(4);

                cv::perspectiveTransform(obj_corners, scene_corners, H);

                cv::circle(frame_gray, scene_corners[0], 4, cv::Scalar::all(255), 2);
                cv::circle(frame_gray, scene_corners[1], 4, cv::Scalar::all(255), 2);
                cv::circle(frame_gray, scene_corners[2], 4, cv::Scalar::all(255), 2);
                cv::circle(frame_gray, scene_corners[3], 4, cv::Scalar::all(255), 2);

                // cv::Rect rect(scene_corners[0], scene_corners[2]);
                // cv::rectangle(frame_gray, rect, cv::Scalar::all(255), 2);

                // ROS_INFO("Distance: %d", Distance::getDistanceZ(rect, 20));

                // return true;
            }
        }
    }    

    cv::imshow("Feature Match", frame_gray);
    cv::waitKey(10); // REMOVE!! needed for imshow

    return false;
}

bool Detector::detectCascade()
{
    return true;
}

bool Detector::detectLine()
{
    return true;
}