#include <vector>
#include <iostream>
#include <string.h>
//#include <pcl/ros/conversions.h>
//#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/io/pcd_io.h>
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/Image.h>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv/cv.h>
//#include <bulletsim_msgs/TrackedObject.h>
//#include <sys/timeb.h>

////////////////////////////
//#include <pcl_conversions/pcl_conversions.h>
#include "simulation/config_bullet.h"

//#include "clouds/utils_ros.h"
#include "clouds/utils_pcl.h"
#include "utils_tracking.h"
//#include "utils/logging.h"
#include "utils/utils_vector.h"
//#include "visibility.h"
#include "physics_tracker.h"
#include "feature_extractor.h"
#include "initialization.h"
#include "simulation/simplescene.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "clouds/cloud_ops.h"
#include "simulation/util.h"
//#include "clouds/utils_cv.h"
//#include "simulation/recording.h"
//#include "cam_sync.h"
//
//#include "simulation/config_viewer.h"

#include <Eigen/Dense>

struct TrackerState{
    TrackerState(vector<btVector3>& nodes, btScalar radius){

        Eigen::setNbThreads(2);
        GeneralConfig::scale = 100;
        BulletConfig::maxSubSteps = 0;
        BulletConfig::gravity = btVector3(0,0,-0.1);

        util::setGlobalEnv(scene_.env);

//        std::cout << nodes.size() << std::endl;
        CapsuleRope::Ptr sim(new CapsuleRope(scaleVecs(nodes,METERS), radius*METERS));

        TrackedRope::Ptr tracked_rope(new TrackedRope(sim));
//        std::cout << "created rope" << std::endl;
//        std::cout << tracked_rope->m_nNodes << std::endl;

        trackedObj_ = tracked_rope;
        trackedObj_->init();
//        std::cout << "inited rope" << std::endl;

        scene_.env->add(trackedObj_->m_sim);

        objectFeatures_.reset(new TrackedObjectFeatureExtractor(trackedObj_));
        cloudFeatures_.reset(new CloudFeatureExtractor());
        alg_.reset(new PhysicsTracker(objectFeatures_, cloudFeatures_));

        scene_.setSyncTime(false);
        scene_.setDrawing(false);

    }
    ~TrackerState(){
        alg_ = nullptr;
        cloudFeatures_ = nullptr;
        objectFeatures_ = nullptr;
        trackedObj_ = nullptr;
    }

    void update(int num_iter, ColorCloudPtr filteredCloud, Eigen::VectorXf& vis_vec, float* out_nodes_ptr, int out_nodes_size){
//        for(int i = 0; i < filteredCloud->points.size(); i++){
//            std::cout << filteredCloud->points[i].x << ' '
//                      << filteredCloud->points[i].y << ' '
//                      << filteredCloud->points[i].z << ' '
//                      << std::endl;
//        }
        // filtered cloud in ground frame
        cloudFeatures_->updateInputs(filteredCloud);
        alg_->updateFeatures(vis_vec);
        Eigen::MatrixXf estPos_next = alg_->CPDupdate();
//        std::cout << "estPos_next" << std::endl;
//        std::cout << estPos_next << std::endl;

        for(int i = 0; i < num_iter; i++){
            alg_->updateFeatures(vis_vec);
            objectFeatures_->m_obj->CPDapplyEvidence(toBulletVectors(estPos_next));
            scene_.step(.03, 2, .015);
        }

        std::vector<btVector3> nodes = objectFeatures_->m_obj->getPoints();
        assert(out_nodes_size == nodes.size());
        for(int i = 0; i < nodes.size(); i++){
            btVector3 vec = nodes[i]/METERS;
            out_nodes_ptr[i * 3 + 0] = vec.x();
            out_nodes_ptr[i * 3 + 1] = vec.y();
            out_nodes_ptr[i * 3 + 2] = vec.z();
        }
    }

    TrackedObject::Ptr trackedObj_;
    TrackedObjectFeatureExtractor::Ptr objectFeatures_;
    CloudFeatureExtractor::Ptr cloudFeatures_;
    PhysicsTracker::Ptr alg_;
    Scene scene_;
};

extern "C" TrackerState* create_tracker(float * nodes_ptr, int size, float radius){
    std::vector<btVector3> nodes(size);
    for(int i = 0; i < size; i++){
        nodes[i] = btVector3(nodes_ptr[i*3 + 0], nodes_ptr[i*3 + 1], nodes_ptr[i*3 + 2]);
    }


    auto tracker_state = new TrackerState(nodes, radius);
    return tracker_state;
}

extern "C" void delete_tracker(TrackerState* tracker_state){
    std::cout << tracker_state << std::endl;
    delete tracker_state;
}

extern "C" void update_tracker(TrackerState* tracker_state, int num_iter,
                               float* cloud_ptr, int cloud_size,
                               float* vis_vec_ptr, int vis_vec_size,
                               float* out_nodes_ptr, int out_nodes_size){
    ColorCloudPtr cloud(new ColorCloud);
    cloud->width = cloud_size;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for(int i = 0; i < cloud_size; i++){
        cloud->points[i].x = cloud_ptr[i * 3 + 0] * METERS;
        cloud->points[i].y = cloud_ptr[i * 3 + 1] * METERS;
        cloud->points[i].z = cloud_ptr[i * 3 + 2] * METERS;
    }

//    for(int i = 0; i < cloud->points.size(); i++){
//        std::cout << cloud->points[i].x << 'a'
//                  << cloud->points[i].y << 'b'
//                  << cloud->points[i].z << 'c'
//                  << std::endl;
//    }


    Eigen::VectorXf vis_vec = Eigen::VectorXf::Map(vis_vec_ptr, vis_vec_size);

    tracker_state->update(num_iter, cloud, vis_vec, out_nodes_ptr, out_nodes_size);
}
