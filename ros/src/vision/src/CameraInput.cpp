#include "CameraInput.hpp"
#include "ros/console.h"

// TODO: Determine if a need for more than one camera exists. However, the implementation most
// likely should not be depending on a certain number of cameras attached.
CameraInput::CameraInput() : input_front(0)
{
    if (!input_front.isOpened()) {
        ROS_INFO("Error: Unable to open front camera");
    }
}

CameraInput::~CameraInput()
{}

bool CameraInput::update()
{
    if(!input_front.read(frame_front)) {
        return false;
    }

    return true;
}

cv::Mat CameraInput::getFrameFront() { return frame_front; }
