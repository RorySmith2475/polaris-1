#ifndef DETECTOR_
#define DETECTOR_

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

class CameraInput;

class Detector
{
public:
    virtual bool setup();   
    virtual bool update();

protected:
    Detector(CameraInput& camera_input);
    ~Detector();

    virtual bool detectColor();


    virtual bool detectMatch();
    cv::Ptr<cv::xfeatures2d::SURF> surf;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::vector<cv::KeyPoint> image_points;
    cv::Mat image_descriptors;
    cv::Mat match_image;
    const uint16_t MIN_HESSIAN = 400;
    const float RATIO_THRESH = 0.5; // Lower is more accurate
    const uint8_t MIN_MATCH_THRESH = 20;
    
    virtual bool detectCascade();
    
    
    virtual bool detectLine();

    CameraInput& input;
    uint16_t x;
    uint16_t y;
    uint16_t z;
    float angle;
    bool object_found;
    uint8_t accuracy;
};

#endif
