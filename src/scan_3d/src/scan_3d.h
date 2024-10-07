#ifndef __SCAN_3D_H__
#define __SCAN_3D_H__

#include "definitions.h"


class Scan3D {
public:
    Scan3D();
    ~Scan3D();
    
    void scanCallback(const laser_t& scan);
    void poseCallback(const pose_msg_t& pose);
    
    cloud_ptr_t transformScan(const laser_t& scan);
    matrix4 createTransformMatrix(double position_factor);
    void searchGround(const laser_t& scan);
    double searchTrigonometric(const laser_t& scan);
    
    void storeCloud(cloud_ptr_t cloud);
    void publishCloud(cloud_ptr_t cloud);
    
    pose_tf_t subPose(const pose_t& pose);
    void updatePose(const pose_t& pose);
    
private:
    handler_t node_;
    handler_t private_nh_;
    subscriber_t scan_subscriber_;
    subscriber_t pose_subscriber_;
    publisher_t map_publisher_;
    publisher_t map_pcl_publisher_;
    
    tf_listener_t tfListener_;
    projector_t projector_;
    
    pose_tf_t pose_;
    pose_tf_t last_pose_;
    cloud_ptr_t cloud_;
    
    bool first_scan_;
    bool pose_update_;
    ros::Time start_time_;
    boost::mutex map_locker_;
    
    bool visualize_rviz_;
    bool visualize_pcl_;
    bool trigonometric_ground_search_;
    double map_update_distance_thresh_;
    double map_update_angle_thresh_;
    double ground_search_angle_;
};


#endif

