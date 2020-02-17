#ifndef DETECTOR_
#define DETECTOR_

#include "opencv2/opencv.hpp"

class CameraInput;

class Detector
{
public:
    virtual bool setup();
    virtual bool update();

protected:
    // struct DetectionData {
    //     uint16_t x;             // cm
    //     uint16_t y;             // cm
    //     uint16_t z;             // cm
    //     float angle;            // degrees (radian?)
    //     bool object_found;      // {0,1}
    //     uint8_t accuracy;       // 0 -> 100
    // };

    Detector(CameraInput& camera_input);
    ~Detector();

    virtual bool detectColor();
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
