#include <ros/ros.h>
#include <functional>
#include <list>
#include <map>

#include "vision/vector.h"
#include "vision/vector_array.h"
#include "vision/set_detection.h"

#include "EnableDetector.hpp"
#include "GateDetector.hpp"
#include "CameraInput.hpp"
#include "DetectorState.hpp"
// #include "PathDetector.hpp"

#define POLLING_RATE_HZ 10

class VisionSystem
{
private:
    // ROS Framework required to instantiate the node and provide service and publishing interfaces
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::ServiceServer setDetection_;
    vision::vector_array msg_;

    // Image capturing and detection systems.
    CameraInput camera_input;
    GateDetector gate;

    // Collection of detectors together with their state (enabled to detect or not)
    std::map<std::reference_wrapper<Detector>,DetectorState> detectors;

public:
    VisionSystem(ros::NodeHandle& nh)
        :   nh_(nh),
            camera_input(),
            gate(camera_input)

    {
        
    
        detectors[gate] = DetectorState.ENABLED;
    }

	/**
	 * This is the service for changing the detection type of the detection system.
	 * @param request the requested detection type.
	 * @param response unused/empty.
	 * @return true if success or false if failure.
	 */
	bool setDetectionCallback(vision::set_detection::Request& request, vision::set_detection::Response& response)
	{
		enabledDetectors_ = static_cast<EnabledDetector>(request.enabled_type);
		return true;
	}

    /**
     * Configures the Vision node to such a state that it is ready to detect objects, handle service
     * requests, and publish data
     * 
     */
    int setup()
    {
        pub_ = nh.advertise<vision::vector>("/vision/vector_array", 1);

        setDetection_ = nh.advertiseService("/vision/set_detection", &VisionSystem::setDetectionCallback, this);
    }

    /**
     * The main loop of the Vision node.
     */
    int loop()
    {
        int status = 1;

        ros::Rate r(POLLING_RATE_HZ);
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
