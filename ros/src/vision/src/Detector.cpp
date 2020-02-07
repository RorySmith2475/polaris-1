#include "Detector.hpp"
#include "CameraInput.hpp"

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

bool Detector::setup()
{
    return true;
}

bool Detector::update()
{
    detectColor();
    detectCascade();
    detectLine();
    
    return true;
}

bool Detector::detectColor()
{
    return true;
}

bool Detector::detectCascade()
{
    return true;
}

bool Detector::detectLine()
{
    return true;
}