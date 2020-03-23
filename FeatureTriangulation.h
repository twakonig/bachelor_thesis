
#ifndef TARGET_TRACKING_FEATURETRIANGULATION_H
#define TARGET_TRACKING_FEATURETRIANGULATION_H

namespace triangulation {


    class FeatureTriangulation {
    public:
        //FOR NOW ALL FUNCTIONS OF RETURN TYPE VOID

        void imageCallback(/*sensor_msgs*/);
        //gets an image, queries the camera pose via tf

        void extractFeatures(/*image_frames*/);
        //extracts features given image and camera pose?

        void decideToStoreImage();
        //imposes constraint on usability of image/camera pose/features.
        //store image to use for feature triangulation once another good image is found

        void matchFeatures(/*image1, image2, poseOffsetBetweenCameraInstances*/);
        //two usable images with sufficient translation (baseline) between them
        //match the features and initialize their 3d positions

        //constructor and destructor
        FeatureTriangulation();

        ~FeatureTriangulation();

    };

}   //namespace triangulation


#endif //TARGET_TRACKING_FEATURETRIANGULATION_H
