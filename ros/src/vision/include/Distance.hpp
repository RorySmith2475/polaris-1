#ifndef DISTANCE
#define DISTANCE

#define FRONT_FOCAL 380.65
#define TOP_FOCAL 481.81
#define BOTTOM_FOCAL 0

#include "opencv2/opencv.hpp"

namespace Distance
{
    uint32_t getDistanceX(cv::Rect object, float realWidth, cv::Mat frame);
    uint32_t getDistanceY(cv::Rect object, float realHieght, cv::Mat frame);
    uint32_t getDistanceZ(cv::Rect object, float realWidth);
};

#endif