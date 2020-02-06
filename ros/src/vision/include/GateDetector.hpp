#ifndef GATEDETECTOR_
#define GATEDETECTOR_

#include "Detector.hpp"

class CameraInput;

class GateDetector : public Detector
{
public:
    GateDetector(CameraInput& input);
    ~GateDetector();
};

#endif
