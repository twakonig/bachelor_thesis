
#ifndef TARGET_TRACKING_FEATURETRIANGULATION_HPP
#define TARGET_TRACKING_FEATURETRIANGULATION_HPP

#include <vector>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>                   //for KeyPoint
#include <opencv2/features2d.hpp>                   //for ORB keypoint detector
#include <sensor_msgs/CameraInfo.h>                 //for sensor_msgs
#include <sensor_msgs/PointCloud.h>
#include <image_transport/image_transport.h>        //for ImageConstPtr&
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/sfm.hpp>


using namespace cv;


namespace target_tracking {

/*!
 * Class containing the algorithmic part of the package.
 */

    class FeatureTriangulation {
    public:
        //constructor
        FeatureTriangulation();

        //destructor
        virtual ~FeatureTriangulation();

        //METHODS of FeatureTriangulation class
        void handleTransformData(geometry_msgs::TransformStamped world_cam);

        void extractFeatures(Mat frame);

        bool decideToStoreImage();

        void matchFeatures();

        std::vector<Point3f> triangulation();

        void getRotAndProjMat();

        bool keyframeDetected();

        void detectTriangulatedPts();

        //void getKmatrix(Mat intrinsic_data);



    private:

        //internal variable to hold current amount of keyframes -> descriptors only calculated for keyframes
        int nKeyframes_;

        //if true, first frame
        bool firstFrame_;

        //for inspection, number of times frame is queried
        int countFrames_;

        //true if incoming frame is declared a keyframe -> passed to RosModule
        bool isKeyframe_;

        Mat cam_mat_;

        struct KeyFrame {
            //all data needed per keyframe
            geometry_msgs::TransformStamped worldToCam;
            Eigen::Vector3d transl_eigen;
           // Eigen::Matrix4d transformation_mat;
            std::vector<KeyPoint> keypoints;
            Mat descriptor;
            Mat descriptors_triPts;
            //for triangulation
            Mat transl_mat;
            Mat rot_mat;
            Mat proj_mat;
        };

        //not keyframe, actually just frame
        KeyFrame incoming_;

        //real keyframes
        KeyFrame currentkf_;
        KeyFrame previouskf_;

        std::vector<DMatch> good_matches_;

        std::vector<Point3f> pointcloud_;


        //optimized for incoming frame
        Mat rvec_;
        Mat tvec_;

        Mat frame_;

    };

} /* namespace */

#endif //TARGET_TRACKING_FEATURETRIANGULATION_HPP

