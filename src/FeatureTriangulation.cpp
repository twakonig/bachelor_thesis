#include "../include/target_tracking/FeatureTriangulation.hpp"

#include <iostream>
#include <ros/ros.h>
//#include <node_handle.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>             //for sensor_msgs
#include <image_transport/image_transport.h>    //for ImageConstPtr&


using namespace cv;

//namespace target_tracking {

FeatureTriangulation::FeatureTriangulation()
{
}

FeatureTriangulation::~FeatureTriangulation()
{
}

void FeatureTriangulation::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("Function worked");
}

//} /* namespace */



    int main(int argc, char **argv)
    {
        //initializing the node
        ros::init(argc, argv, "feature_triangulation");     //node name: "feature_triangulation"
        //starting the node
        ros::NodeHandle nh;     //calls ros::start()

        //creating an object of my class
        FeatureTriangulation object;

        //subscribe to /cam0/image_rect topic
        ros::Subscriber sub = nh.subscribe("/cam0/image_rect", 1000, &FeatureTriangulation::imageCallback, &object);

        //calls message callbacks, exits once ros::ok() returns false
        ros::spin();

        return 0;
    }


