#include "GateDetector.hpp"
#include "CameraInput.hpp"
#include "ros/console.h"

#define GATE_WIDTH      120
#define GATE_HEIGHT     60

GateDetector::GateDetector(CameraInput& camera_input)
    :   input(camera_input)
{}

GateDetector::~GateDetector()
{}