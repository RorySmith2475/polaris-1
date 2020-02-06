#include <ros/ros.h>
#include <functional>
#include <list>

#include "vision/vector.h"
#include "vision/change_detection.h"

#include "EnableDetector.hpp"
#include "CameraInput.hpp"
#include "Detector.hpp"
#include "GateDetector.hpp"
#include "PathDetector.hpp"

// VisionSystem::EnabledDetector::NONE

class VisionSystem
{
private:
    // Ros fun stuff
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::ServiceServer changeDetection_;
    vision::gate gate_msg_;

    // Image capturing and detection systems.
    CameraInput camera_input;
    Detector detector;
    GateDetector gate;
    PathDetector path;

    std::list<std::reference_wrapper<Detector>> detectors;

    // Detectors that are Enabled
    EnabledDetector enabledDetectors_;

public:
    VisionSystem(ros::NodeHandle& nh)
        :   nh_(nh),
            camera_input(),
            detector(camera_input),
            gate(camera_input),
            path(camera_input),
            detectors{gate, path}

    {
        pub_ = nh.advertise<vision::vector>("/vision/vector", 1);
        changeDetection_ = nh.advertiseService("/vision/change_detection", &VisionSystem::changeDetectionCallback, this);
    }

	/**
	 * This is the service for changing the detection type of the detection system.
	 * @param request the requested detection type.
	 * @param response unused/empty.
	 * @return true if success or false if failure.
	 */
	bool changeDetectionCallback(vision::change_detection::Request& request, vision::change_detection::Response& response)
	{
		enabledDetectors_ = static_cast<EnabledDetector>(request.enabled_type);
		return true;
	}

    int setup()
    {

    }

    /* THIS IS THE MAIN LOOP OF THE VISION SYSTEM */
    int loop()
    {
        int status = 1;

        ros::Rate r(10); // Maybe Faster
        while(ros::ok() && status)
        {
            if(camera_input.update())
            {
               for(Detector& d : detectors)
               {
                   d.update();
               }
            }

            pub_.publish(msg_);
            ros::spinOnce();
            r.sleep();
        }

        return status;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh("~");

    VisionSystem visionSystem(nh);

    int status = visionSystem.setup();
    if(status) status = visionSystem.loop();

    ros::shutdown();
    return status;
}
