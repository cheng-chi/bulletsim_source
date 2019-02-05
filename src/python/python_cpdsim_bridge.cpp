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

//void copy_rope(const bulletsim_msgs::ObjectInit& initMsg){
//    vector<btVector3> nodes = toBulletVectors(initMsg.rope.nodes);
//    BOOST_FOREACH(btVector3& node, nodes) node += btVector3(0,0,.01);
//}

extern "C" void copy_eigen(float * arr_ptr, int size){
    Eigen::VectorXf eigen_vec = Eigen::VectorXf::Map(arr_ptr, size);
    std::cout << eigen_vec << std::endl;
}

struct TrackerState{
    TrackerState(vector<btVector3>& nodes, btScalar radius){
        simulation_num_iter_ = 20;

        Eigen::setNbThreads(2);
//        GeneralConfig::scale = 100;
//        BulletConfig::maxSubSteps = 0;
//        BulletConfig::gravity = btVector3(0,0,-0.1);

        CapsuleRope::Ptr sim(new CapsuleRope(scaleVecs(nodes,METERS), radius*METERS));
        TrackedRope::Ptr tracked_rope(new TrackedRope(sim));
        std::cout << "created rope" << std::endl;

        trackedObj_ = tracked_rope;
        trackedObj_->init();
        std::cout << "inited rope" << std::endl;

        objectFeatures_.reset(new TrackedObjectFeatureExtractor(trackedObj_));
        cloudFeatures_.reset(new CloudFeatureExtractor());
        alg_.reset(new PhysicsTracker(objectFeatures_, cloudFeatures_));

    }
    ~TrackerState(){
        alg_ = nullptr;
        cloudFeatures_ = nullptr;
        objectFeatures_ = nullptr;
        trackedObj_ = nullptr;
    }

    void update(ColorCloudPtr filteredCloud, Eigen::VectorXf& vis_vec){
        // filtered cloud in ground frame
        cloudFeatures_->updateInputs(filteredCloud);
        alg_->updateFeatures(vis_vec);
        Eigen::MatrixXf estPos_next = alg_->CPDupdate();

        for(int i = 0; i < simulation_num_iter_; i++){
            alg_->updateFeatures(vis_vec);
            objectFeatures_->m_obj->CPDapplyEvidence(toBulletVectors(estPos_next));
        }
    }

    int simulation_num_iter_;
    TrackedObject::Ptr trackedObj_;
    TrackedObjectFeatureExtractor::Ptr objectFeatures_;
    CloudFeatureExtractor::Ptr cloudFeatures_;
    PhysicsTracker::Ptr alg_;
};

extern "C" TrackerState* create_tracker(float * nodes_ptr, int size, float radius){
    std::vector<btVector3> nodes(size);
    for(int i = 0; i < size; i++){
        nodes[i] = btVector3(nodes_ptr[i*3 + 0], nodes_ptr[i*3 + 1], nodes_ptr[i*3 + 2]);
    }

    for(int i = 0; i < size; i++){
        std::cout << nodes[i].x() << ' ' << nodes[i].y() << ' ' << nodes[i].z() << std::endl;
    }

    auto tracker_state = new TrackerState(nodes, radius);
//    TrackerState* tracker_state = nullptr;
    return tracker_state;
}

extern "C" void delete_tracker(TrackerState* tracker_state){
    std::cout << tracker_state << std::endl;
    delete tracker_state;
}

extern "C" void update_tracker(TrackerState* tracker_state){
    std::cout << tracker_state << std::endl;
}
