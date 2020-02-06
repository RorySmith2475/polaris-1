#ifndef DETECTOR_
#define DETECTOR_

#include "opencv2/opencv.hpp"

class CameraInput;

class Detector
{
public:
    Detector(CameraInput& camera_input);
    ~Detector();

    virtual bool setup();
    virtual bool update();

protected:
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
