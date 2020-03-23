

#include "../include/FeatureTriangulation.h"
#include <ros/ros.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>    //publish and subscribe to images
#include <opencv2/highgui/highgui.hpp>          //display images using opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>

using namespace cv;


namespace triangulation {



    //global objects and variables
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


//constructor and destructor still empty <-- TEST
    FeatureTriangulation::FeatureTriangulation(void) {
        std::cout << "Object created - TEST" << std::endl;
    }

    FeatureTriangulation::~FeatureTriangulation(void) {
        std::cout << "Object deleted - TEST" << std::endl;
    }


    //purpose: passed as argument to other code
    //gets an image, queries the camera pose via tf, called whenever new message arrives
    //callback needs to handle the type sensor_msgs/Image
    Mat FeatureTriangulation::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        /* RELATED DECLARATIONS
         * in main: subscriber to "/cam0/image_rect"
         * global variable //TransformListener object receives tf2 transformations
                            tf2_ros::Buffer tfBuffer;
                            tf2_ros::TransformListener tfListener(tfBuffer); -> in main or here??

         here:  geometry_msgs::TransformStamped inertialToCamera; //transformatioin passed as transl. vector and rot. quaternion
                try{
                    inertialToCamera = tfBuffer.lookupTransform("cam0", "inertial", ros::Time(0));
                }
                catch (tf2::transformException& ex) {
                    ROS_ERROR("%s",ex.what());
                }

                Mat importImage = cv_bridge::toCvShare(msg, "bgr8")->image;

                return importImage;

        */

    }



//extracts features given image -> link to imageCallback function??
//how to get single frames
    void FeatureTriangulation::extractFeatures(Mat frame) {
        //for example ORB features
        /*
         *
        std::vector<KeyPoint> keypoints;
        Mat keypoint_img;

        Ptr<ORB> detector = ORB::create(nFeatures);
        detector->detect(frame, keypoints);

         OR

         detector-> detectAndCompute (frame,....) //detects keypoints and computes descriptors

        //get keypoint coordinates, make a list
        //8point algorithm
        //or dafault matchig options




        //VISUALIZATION
        drawKeypoints(frame, keypoints, keypoint_img);

        imshow("Detected keypoints", keypoint_img);

        //visualize to test output -> can I just import single frame for testing??
        //interested in keypoint coordinates?

         */

    }


//imposes constraint on usability of image/camera pose/features.
//store image to use for feature triangulation once another good image is found
    void FeatureTriangulation::decideToStoreImage(Mat image) {
        //container to store images..list? array?

        //Mat featureImage = extractFeatures (image);
        //get transformation data
        /*
         *list<Mat> frameList;
         * nimgs = 0; //number of saved images
         *
         while(nimgs < imgThreshold){

            if(nfeatures > featureThreshold && (posv1 - posv2) > min_distance) {    //constraint

                append current_img to frameList
                ?compute transformation between frames?
                nimgs += 1;
            }
            else
                discard frame
            }
        }

         //other constraints on usability?

         */
    }

//two usable images with sufficient translation (baseline) between them
//match the features and initialize their 3d positions
    void FeatureTriangulation::matchFeatures(/*image1, image2, poseOffsetBetweenCameraInstances*/) {
/*
        //need descriptors for keypoints-> already do in extract features or do here?
        //Method A:Use a matcher from opencv library
            - imply min_distance for 'good matches'
            - use only good matches
        //Method B: use epipolar constraints between frames, solve linear systems of equations

        Matched keypoints -> get coordinates
        perform triangulation

        A: Triangulation via linearization and set of equations
        B: triangulatePoints via opencv
        --> need projection matrices

    */

    }


//**********************MAIN BEGIN************************

    int main(int argc, char **argv) {

        //initilization and node name
        ros::init(argc, argv, "feature_triangulation");

        //access point for communication (topics, services, parameters)
        ros::NodeHandle nodeHandle;

        //.subscribe(topic, #messages allowed to internally buffer, what to do with received message)
        //sends message to imageCallback function
        image_transport::ImageTransport it(nodeHandle);
        image_transport::Subscriber sub = it.subscribe("/cam0/image_rect", 1000, imageCallback);
        //ros::Subscriber subscriber = nodeHandle.subscribe("/cam0/image_rect", 1000, imageCallback);

        //TransformListener object receives tf2 transformations -> GLOBAL
        //tf2_ros::Buffer tfBuffer;
        //tf2_ros::TransformListener tfListener(tfBuffer);


        //block, work on in order to receive messages -> continue listening
        ros::spin();

        //destroyAllWindows();

        return 0;
    }

}   //namespace triangulation