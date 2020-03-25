

#ifndef TARGET_TRACKING_FEATURETRIANGULATION_HPP
#define TARGET_TRACKING_FEATURETRIANGULATION_HPP

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>             //for sensor_msgs
#include <image_transport/image_transport.h>    //for ImageConstPtr&

//namespace target_tracking {


    class FeatureTriangulation {
    public:
        //constructor
        FeatureTriangulation();

        //destructor
        virtual ~FeatureTriangulation();

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    };

//} /* namespace */

#endif //TARGET_TRACKING_FEATURETRIANGULATION_HPP
