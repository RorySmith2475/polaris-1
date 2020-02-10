#ifndef PATHDETECTOR_
#define PATHDETECTOR_

#include "Detector.hpp"

class CameraInput;

class PathDetector : public Detector
{
public:
    PathDetector(CameraInput& camera_input);
    ~PathDetector();
};

#endif