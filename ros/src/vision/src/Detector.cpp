#include "Detector.hpp"
#include "CameraInput.hpp"

Detector::Detector(CameraInput& camera_input)
    :   input(camera_input)
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