//#include "../include/target_tracking/FeatureTriangulation.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace cv;

int main( int argc, char** argv ) {

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    namedWindow("view");
    startWindowThread();
    image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub = it.subscribe("/cam0/image_rect", 5, imageCallback);

    ros::spin();
    destroyWindow("view");
}
