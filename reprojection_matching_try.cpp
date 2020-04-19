
#include "../include/target_tracking/FeatureTriangulation.hpp"


namespace target_tracking {

    //constructor
    FeatureTriangulation::FeatureTriangulation()
        : nKeyframes_(0), firstFrame_(true), countFrames_(0), isKeyframe_(true)
    {
        //initialize camera matrix, CONST values
        cam_mat_ = Mat::zeros(3, 3, CV_64FC1);
        cam_mat_.at<double>(0, 0) = 631.8094467471138;
        cam_mat_.at<double>(1, 1) = 631.9958703516292;
        cam_mat_.at<double>(0, 2) = 415.26735498345005;
        cam_mat_.at<double>(1, 2) = 300.5214432137359;
        cam_mat_.at<double>(2, 2) = 1.0;
    }

    //destructor
    FeatureTriangulation::~FeatureTriangulation()
    {
    }

    /*  MANUALLY

    void FeatureTriangulation::getKmatrix(Mat intrinsic_data) {
        cam_mat_ = intrinsic_data;
    }

     */

    bool FeatureTriangulation::keyframeDetected() {
        return isKeyframe_;
    }



    void FeatureTriangulation::handleTransformData(geometry_msgs::TransformStamped world_cam) {
        //get transformation matrix
        //INITIAL GUESSES
        incoming_.worldToCam = world_cam;
        incoming_.transl_eigen << incoming_.worldToCam.transform.translation.x,
                                  incoming_.worldToCam.transform.translation.y,
                                  incoming_.worldToCam.transform.translation.z;


         Eigen::Quaterniond q_worldcam(incoming_.worldToCam.transform.rotation.w,
                                       incoming_.worldToCam.transform.rotation.x,
                                       incoming_.worldToCam.transform.rotation.y,
                                       incoming_.worldToCam.transform.rotation.z);

         q_worldcam.normalize();
         Eigen::Matrix3d R_worldcam = q_worldcam.toRotationMatrix();
         eigen2cv(R_worldcam,incoming_.rot_mat);

    }



    void FeatureTriangulation::extractFeatures(Mat frame) {

        frame_ = frame;
        countFrames_ += 1;

        /*
        //ORB detector
        Ptr<ORB> orb_detector = ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);

        orb_detector -> detect(frame, incoming_.keypoints);


        //OPTIONAL drawing
        //Mat img_keypoints;
        //drawKeypoints(frame, incoming_.keypoints, img_keypoints, Scalar(255,0,255), 0);    //pink
         */

        //BRISK detector
        Ptr<BRISK> brisk_detector = BRISK::create(70, 0, 1.0f);

        //possible to include mask where to look for keypoints
        brisk_detector -> detectAndCompute(frame, noArray(), incoming_.keypoints, incoming_.descriptor);
        std::cout << "number of keypoints incoming frame: " << incoming_.keypoints.size() << std::endl;


        if(countFrames_ > 1)
            matchFeatures();

        bool key_criteria = decideToStoreImage();

        if (key_criteria == true) {                 //criteria for keyframe must be met
            //sets firstFrame_ to false after first call
            firstFrame_ = false;
            isKeyframe_ = true;

            //SET INCOMING RVEC AND TVEC TO OPTIMIZED VALUES
            //Rodrigues(rvec_, incoming_.rot_mat);
            incoming_.transl_mat = Mat::zeros(3, 1, CV_64FC1);
            incoming_.transl_mat = tvec_;

            //RESET PREVIOUS AND CURRENT (neglectable for first call)
            if (countFrames_ > 1) {
                previouskf_ = currentkf_;
            }

            //DECLARE INCOMING FRAME AS CURRENT KEYFRAME
            currentkf_ = incoming_;

            //compute projection matrix

            //compute RotMat and ProjMat of current Keyframe
            //NOW: only get proj mat for optimized
            getRotAndProjMat();
        }
    }



    bool FeatureTriangulation::decideToStoreImage() {

        std::cout << "number of frame queries: " << countFrames_ << std::endl;
        std::cout << "number of keyframes: " << nKeyframes_ << std::endl;

        if (firstFrame_ == true) {
            nKeyframes_ += 1;
            return true;
        }
        else if (good_matches_.size() < 150) {
            nKeyframes_ += 1;
            return true;
        }
        else {
            isKeyframe_ = false;
            return false;
        }
    }



    void FeatureTriangulation::matchFeatures() {

        //container for matches
        std::vector<std::vector<DMatch> > knn_matches;     //2D vector
        Ptr<DescriptorMatcher> bfmatcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);

        //Finds the k best matches; first argument is query, second train
        bfmatcher -> knnMatch(currentkf_.descriptor, incoming_.descriptor, knn_matches, 2);
        good_matches_.clear();

        //Lowe's ratio test to filter matches
        const float ratio_thresh = 0.6f;

        for (size_t i = 0; i < knn_matches.size(); i++) {
            //checking with distance to 2nd best match
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches_.push_back(knn_matches[i][0]);
            }
        }
        std::cout << "good matches, size: " << good_matches_.size() << std::endl;
        //called in frame after first triangulation
        if (nKeyframes_ > 1)
           detectTriangulatedPts();
    }




    std::vector<Point3f> FeatureTriangulation::triangulation() {

        std::vector<KeyPoint> pts_previous;
        std::vector<KeyPoint> pts_current;
        std::vector<Point2f> pts_tri_previous;
        std::vector<Point2f> pts_tri_current;
        pointcloud_.clear();


        for (int i = 0; i < good_matches_.size(); ++i) {
            pts_previous.push_back(previouskf_.keypoints[good_matches_[i].queryIdx]);
            pts_current.push_back(currentkf_.keypoints[good_matches_[i].trainIdx]);
        }

        Mat P1 = previouskf_.proj_mat;
        Mat P2 = currentkf_.proj_mat;

        /*
        //NO "PROOF" but I think version with correction has less glitches/extreme outliers------------------------------------------------------------------
        Mat fundamental_mat = findFundamentalMat(pts_previous, pts_current, FM_RANSAC, 3, 0.99);

        correctMatches(fundamental_mat, pts_previous, pts_current, pts_tri_previous_, pts_tri_current_);

        Ptr<BRISK> brisk_detector = BRISK::create(70, 0, 1.0f);
        brisk_detector -> compute(frame_, pts_tri_current_, currentkf_.descriptors_triPts);

        //---------------------------------------------------------------------------------------------------------------------------------------------------

         */

        Ptr<BRISK> brisk_detector = BRISK::create(70, 0, 1.0f);
        brisk_detector -> compute(frame_, pts_current, currentkf_.descriptors_triPts);

        for (int i = 0; i < good_matches_.size(); ++i) {
            pts_tri_previous.push_back(pts_previous[i].pt);
            pts_tri_current.push_back(pts_current[i].pt);
        }

        //stores triangulated points (homogeneous coord.), 4xgood_matches_.size()
        Mat coord_homog;

        //OPENCV FUNCTION to triangulate points, ADDS POINT VECTORS (HOM) COLUMNWISE
        triangulatePoints(P1, P2, pts_tri_previous, pts_tri_current, coord_homog);

            //append coordinates to vector for pointcloud generation
            for (int n = 0; n < good_matches_.size(); ++n) {

                float x = coord_homog.at<float>(0, n);
                float y = coord_homog.at<float>(1, n);
                float z = coord_homog.at<float>(2, n);
                float w = coord_homog.at<float>(3, n);

                //VECTOR named pointcloud, nth element holds nth point
                pointcloud_.push_back(Point3f(x/w, y/w, z/w));
            }
            detectTriangulatedPts();
            return pointcloud_;
    }



    void FeatureTriangulation::detectTriangulatedPts() {
        //detects already triangulated points in an incoming_ image
        //matched descriptors of incoming with currentkf

        std::cout << "camera matrix.: " << std::endl;
        std::cout << cam_mat_ << std::endl;

        //match keypoints of incoming_ to keypoints used in triangulation from current frame -> NEED DESCRIPTORS
        //query: pts_tri_currentkf_; train: incoming_.keyp
        //use descriptors of currentkf that correspond to pts_tri_currentkf_


        //MATCHING BETWEEN DESCRIPTORS OF TRIANGULATED KEYPOINTS OF CURRENTKF AND DESCRIPTORS OF INCOMING FRAME
        std::vector<std::vector<DMatch> > knn_matches;     //2D vector
        Ptr<DescriptorMatcher> bfmatcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
        bfmatcher -> knnMatch(currentkf_.descriptors_triPts, incoming_.descriptor, knn_matches, 2);
        const float ratio_thresh = 0.6f;
        std::vector<DMatch> good_matches;

        for (size_t i = 0; i < knn_matches.size(); i++) {
            //checking with distance to 2nd best match
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        std::vector<Point2f> coord_matched;
        for (int i = 0; i < good_matches_.size(); ++i) {
            coord_matched.push_back(incoming_.keypoints[good_matches[i].queryIdx].pt);
        }

        //PROJECTED POINTS NO DISTORTION COEFFICIENTS
        //Defining rotvec, tvec for incoming_ frame
        Mat dist_coeff0 = Mat::zeros(5, 1, CV_64FC1);

        rvec_ = Mat::zeros(3, 1, CV_64FC1);
        tvec_ = Mat::zeros(3, 1, CV_64FC1);

        //get rotvec
        //Rodrigues(incoming_.rot_mat, rvec_);
        std::cout << "rotvec initial: " << rvec_ << std::endl;

        //get translvec
        eigen2cv(incoming_.transl_eigen, tvec_);
        std::cout << "transvec initial: " << tvec_ << std::endl;


        solvePnPRansac(pointcloud_, coord_matched, cam_mat_, dist_coeff0, rvec_, tvec_,
                1, 100, 8.0, 0.99, noArray(), SOLVEPNP_ITERATIVE);

        std::cout << "rotvec AFTER PnP: " << rvec_ << std::endl;
        std::cout << "transvec AFTER PnP : " << tvec_ << std::endl;

        //if it is a new keyframe -> make improved rvec and tvec to rot_mat and transl_vec of currentkf
        //make rvec and tvec to global variables.
/*
 *      std::vector<Point2f> pts_projected;
        Rodrigues(currentkf_.rot_mat, rvec);
        std::cout << "rotmat: " << currentkf_.rot_mat << std::endl;
        std::cout << "rotvec: " << rvec << std::endl;
        eigen2cv(currentkf_.transl_eigen, tvec);
        //projectPoints(pointcloud_, rvec, tvec, cam_mat_, dist_coeff0, pts_projected);

*/

        /*
        ROS_INFO("PROJECTION VALUES");
        std::cout << "backprojected points NO distortion coeff: " << std::endl;
        std::cout << pts_projected << std::endl;
        std::cout << "keypoints used in triang.: " << std::endl;
        std::cout << pts_2_ << std::endl;
         */

    }



    void FeatureTriangulation::getRotAndProjMat() {

        /*
        //fo test with incoming
        Eigen::Quaterniond q_worldcam(incoming_.worldToCam.transform.rotation.w,
                                      incoming_.worldToCam.transform.rotation.x,
                                      incoming_.worldToCam.transform.rotation.y,
                                      incoming_.worldToCam.transform.rotation.z);

        q_worldcam.normalize();
        Eigen::Matrix3d R_worldcam = q_worldcam.toRotationMatrix();
        eigen2cv(R_worldcam,incoming_.rot_mat);
         */

        Eigen::Matrix3d R_worldcam;
        cv2eigen (incoming_.transl_mat, incoming_.transl_eigen);
        cv2eigen(incoming_.rot_mat, R_worldcam);

        //Concatenate rot_mat and translation vector, RT (3x4)
        Eigen::MatrixXd RT(R_worldcam.rows(), R_worldcam.cols()+incoming_.transl_eigen.cols());
        RT << R_worldcam, incoming_.transl_eigen;

        //intrinsic camera parameters, constant
        Eigen::MatrixXd cam(3, 3);
        cam << 631.8094467471138, 0.0, 415.26735498345005,
                0.0, 631.9958703516292, 300.5214432137359,
                0.0, 0.0, 1.0;

        //compute projection matrix (3x4)
        Eigen::MatrixXd P = cam * RT;
        eigen2cv(P,incoming_.proj_mat);

        /*
        //Calculate Rotation matrix from quaternion info
        Eigen::Quaterniond q_worldcam(currentkf_.worldToCam.transform.rotation.w,
                                      currentkf_.worldToCam.transform.rotation.x,
                                      currentkf_.worldToCam.transform.rotation.y,
                                      currentkf_.worldToCam.transform.rotation.z);

        q_worldcam.normalize();
        Eigen::Matrix3d R_worldcam = q_worldcam.toRotationMatrix();
        eigen2cv(R_worldcam,currentkf_.rot_mat);

        //Concatenate rot_mat and translation vector, RT (3x4)
        Eigen::MatrixXd RT(R_worldcam.rows(), R_worldcam.cols()+currentkf_.transl_eigen.cols());
        RT << R_worldcam, currentkf_.transl_eigen;

        //intrinsic camera parameters, constant
        Eigen::MatrixXd cam(3, 3);
        cam << 631.8094467471138, 0.0, 415.26735498345005,
                0.0, 631.9958703516292, 300.5214432137359,
                0.0, 0.0, 1.0;

        //compute projection matrix (3x4)
        Eigen::MatrixXd P = cam * RT;
        eigen2cv(P,currentkf_.proj_mat);
         */
    }

} /* namespace */
