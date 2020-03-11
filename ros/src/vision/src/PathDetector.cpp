#include "PathDetector.hpp"
#include "CameraInput.hpp"

#define LOW_HUE         10
#define HIGH_HUE        30
#define LOW_SAT         150
#define HIGH_SAT        255
#define LOW_VAL         100
#define HIGH_VAL        255

#define PATH

PathDetector::PathDetector(CameraInput& camera_input)
    :   Detector(camera_input)
{}

PathDetector::~PathDetector()
{}