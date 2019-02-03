#include <vector>
#include <iostream>
#include <string.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <bulletsim_msgs/TrackedObject.h>
#include <sys/timeb.h>

////////////////////////////
#include <pcl_conversions/pcl_conversions.h>
#include "simulation/config_bullet.h"

#include "clouds/utils_ros.h"
#include "clouds/utils_pcl.h"
#include "utils_tracking.h"
#include "utils/logging.h"
#include "utils/utils_vector.h"
#include "visibility.h"
#include "physics_tracker.h"
#include "feature_extractor.h"
#include "initialization.h"
#include "simulation/simplescene.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "clouds/cloud_ops.h"
#include "simulation/util.h"
#include "clouds/utils_cv.h"
#include "simulation/recording.h"
#include "cam_sync.h"

#include "simulation/config_viewer.h"

#include <Eigen/Dense>

extern "C" void print_arr(float * arr_ptr, int size){
    for(int i = 0; i < size; i++){
        std::cout << arr_ptr[i] << std::endl;
    }
    return;
}

extern "C" void print(int size){
    std::cout << size << std::endl;
}

extern "C" void print_matrix(float * arr_ptr, int height, int width){
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            std::cout << arr_ptr[i * width + j] << ' ';
        }
        std::cout << std::endl;
    }
    return;
}

extern "C" void copy_matrix(float * dest, float * arr_ptr, int height, int width){
    memcpy(dest, arr_ptr, sizeof(float) * height * width);
    // for(int i = 0; i < height; i++){
    //     for(int j = 0; j < width; j++){
    //         dest[i * width + j] = arr_ptr[i * width + j];
    //     }
    // }
}

extern "C" void copy_eigen(float * arr_ptr, int size){
    Eigen::VectorXf eigen_vec = Eigen::VectorXf::Map(arr_ptr, size);
    std::cout << eigen_vec << std::endl;
}

struct TrackerState{
    TrackerState(int size){
        size_ = size;
        arr = new float[size];
    }
    ~TrackerState(){
        delete[] arr;
    }
    int size_;
    float * arr;
};

//extern "C" TrackerState* create_tracker(float * arr_ptr, int size){
//    TrackerState* tracker_state = new TrackerState(size);
//
//    int nCameras = 1;
//
//    TrackedObject::Ptr trackedObj;
//    trackedObj->init();
//
//    MultiVisibility::Ptr visInterface(new MultiVisibility());
//    for (int i=0; i<nCameras; i++) {
//        if (trackedObj->m_type == "rope") // Don't do self-occlusion if the trackedObj is a rope
//            visInterface->addVisibility(DepthImageVisibility::Ptr(new DepthImageVisibility(transformers[i])));
//        else
//            visInterface->addVisibility(AllOcclusionsVisibility::Ptr(new AllOcclusionsVisibility(scene.env->bullet->dynamicsWorld, transformers[i])));
//    }
//
//    TrackedObjectFeatureExtractor::Ptr objectFeatures(new TrackedObjectFeatureExtractor(trackedObj));
//    CloudFeatureExtractor::Ptr cloudFeatures(new CloudFeatureExtractor());
//    PhysicsTracker::Ptr alg(new PhysicsTracker(objectFeatures, cloudFeatures, visInterface));
//    PhysicsTrackerVisualizer::Ptr trackingVisualizer(new PhysicsTrackerVisualizer(&scene, alg));
//
//    bool applyEvidence = true;
//    scene.addVoidKeyCallback('a',boost::bind(toggle, &applyEvidence), "apply evidence");
//    scene.addVoidKeyCallback('=',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), 0.1f), "increase opacity");
//    scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), -0.1f), "decrease opacity");
//    bool exit_loop = false;
//    scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop), "exit");
//
//    boost::shared_ptr<ScreenThreadRecorder> screen_recorder;
//    boost::shared_ptr<ImageTopicRecorder> image_topic_recorder;
//    if (RecordingConfig::record == RECORD_RENDER_ONLY) {
//        screen_recorder.reset(new ScreenThreadRecorder(scene.viewer, RecordingConfig::dir + "/" +  RecordingConfig::video_file + "_tracked.avi"));
//    } else if (RecordingConfig::record == RECORD_RENDER_AND_TOPIC) {
//        screen_recorder.reset(new ScreenThreadRecorder(scene.viewer, RecordingConfig::dir + "/" +  RecordingConfig::video_file + "_tracked.avi"));
//        image_topic_recorder.reset(new ImageTopicRecorder(nh, image_topics[0], RecordingConfig::dir + "/" +  RecordingConfig::video_file + "_topic.avi"));
//    }
//
//    scene.setSyncTime(false);
//    scene.setDrawing(true);
//
//    while (!exit_loop && ros::ok()) {
//        //Update the inputs of the featureExtractors and visibilities (if they have any inputs)
//        cloudFeatures->updateInputs(filteredCloud, rgb_images[0], transformers[0]);
//        for (int i=0; i<nCameras; i++)
//            visInterface->visibilities[i]->updateInput(depth_images[i]);
//
//        alg->updateFeatures();
//        Eigen::MatrixXf estPos_next = alg->CPDupdate();
//
//        pending = false;
//
//        while (ros::ok() && !pending) {
//
//            //Do iteration
//            alg->updateFeatures();
//            objectFeatures->m_obj->CPDapplyEvidence(toBulletVectors(estPos_next));
//            trackingVisualizer->update();
//            scene.step(.03, 2, .015);
//
//            ros::spinOnce();
//        }
//        objPub.publish(toTrackedObjectMessage(trackedObj));
//    }
//
//    return tracker_state;
//}


extern "C" void delete_tracker(TrackerState* tracker_state){
    std::cout << tracker_state << std::endl;
    delete tracker_state;
}
