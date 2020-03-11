#include <ros/ros.h>
#include <functional>
#include <list>
#include <map>

#include "vision/vector.h"
#include "vision/vector_array.h"
#include "vision/set_detection.h"

#include "GateDetector.hpp"
#include "CameraInput.hpp"
#include "Detector.hpp"
#include "DetectorState.hpp"
#include "DetectorType.hpp"
#include "PathDetector.hpp"

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
    PathDetector path;

    // Collection of detectors together with their state (enabled to detect or not)
    std::map<Detector*,DetectorState> detectors;

    // Message data to be published by ROS messages
    vision::vector_array publishMessageArray;
    vision::vector       publishMessageVector;

public:
    VisionSystem(ros::NodeHandle& nh)
        :   nh_(nh),
            camera_input(),
            gate(camera_input),
            path(camera_input)
    {
        //pub_ = nh.advertise<vision::vector_array>("/vision/vector_array", 1);

        //setDetection_ = nh.advertiseService("/vision/set_detection", &VisionSystem::setDetectionCallback, this);        
    
    }

	/**
	 * This is the service for changing the detection type of the detection system.
	 * @param request the requested detection type.
	 * @param response unused/empty.
	 * @return true if success or false if failure.
	 */
	bool setDetectionCallback(vision::set_detection::Request& request, vision::set_detection::Response& response)
	{
        DetectorType detectorType = static_cast<DetectorType>(request.detectorType);
        DetectorState detectorState = static_cast<DetectorState>(request.detectorState);

		return true;
	}

    /**
     * Configures the Vision node to such a state that it is ready to detect objects, handle service
     * requests, and publish data
     * 
     */
    int setup()
    {
        detectors.insert(std::make_pair(&gate, DETECTOR_DISABLED));
        detectors.insert(std::make_pair(&path, DETECTOR_DISABLED));

        pub_ = nh_.advertise<vision::vector_array>("/vision/vector_array", 1);
        setDetection_ = nh_.advertiseService("/vision/set_detection", &VisionSystem::setDetectionCallback, this);
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
            publishMessageArray.result.clear();

            if(camera_input.update())
            {
                for (std::map<Detector*,DetectorState>::iterator iter = detectors.begin(); iter != detectors.end(); ++iter)
                {
                    Detector* k = iter->first;
                    k->update();

                    // TODO: Populate messages with result
                }

                pub_.publish(publishMessageArray);
            }

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

    ROS_INFO("Shutting Down...");
    ros::shutdown();
    return status;
}