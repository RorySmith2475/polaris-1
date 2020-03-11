#include "Distance.hpp"

uint32_t Distance::getDistanceZ(cv::Rect object, float realWidth)
{   
    double focal = FRONT_FOCAL;
    
    /* Bounds checking to ensure no undefined values */
    uint32_t z_distance;
    if (object.width == 0) {
        z_distance = realWidth*focal;
    } else {
        // TODO: Determine appropriate value for scale_factor of 0
        z_distance = (realWidth*focal)/object.width;
    }

    return z_distance;
}

uint32_t Distance::getDistanceX(cv::Rect object, float realWidth, cv::Mat frame)
{   
    cv::Point center(object.x + object.width/2, object.y - object.height/2);
    uint32_t scale_factor = object.width/realWidth;
    uint32_t x_dis;

    /* Bounds checking to ensure no undefined values */
    if (scale_factor == 0) {
        x_dis = abs(center.x - frame.cols/2);
    } else {
        // TODO: Determine appropriate value for scale_factor of 0
        x_dis = abs(center.x - frame.cols/2)/scale_factor;
    }

    if (center.x - frame.cols/2 < 0){
            x_dis = -x_dis;
    }

    return x_dis;
}

uint32_t Distance::getDistanceY(cv::Rect object, float realHieght, cv::Mat frame)
{   
    cv::Point center(object.x + object.width/2, object.y - object.height/2);
    int scale_factor = object.height/realHieght;
    uint32_t y_dis;

    /* Bound checking to ensure no undefined values */
    if (scale_factor == 0) {
        y_dis = abs(center.y - frame.rows/2);
    } else {
        // TODO: Determine appropriate value for scale_factor of 0
        y_dis = abs(center.y - frame.rows/2)/scale_factor;
    }

    if (center.y - frame.rows/2 > 0){
            y_dis = -y_dis;
      }
    return y_dis;
}