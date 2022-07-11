#include "definitions.h"


boost::mutex map_locker_;
viewer_t viewer_;
cloud_ptr_t cloud_;


void cloud_callback(cloud_t cloud) {
    map_locker_.lock();
    for (auto point : cloud) {
        cloud_->push_back(point);
    }
    viewer_.updatePointCloud(cloud_);
    map_locker_.unlock();
}

void reset_cloud(cloud_t cloud) {
    map_locker_.lock();
    cloud_->clear();
    viewer_.removeAllShapes();
    for (auto point : cloud) {
        cloud_->push_back(point);
    }
    viewer_.updatePointCloud(cloud_);
    map_locker_.unlock();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "view_3d");
    
    std::string topic_map_;
    handler_t node_, private_nh_("~");
    private_nh_.param("map_topic", topic_map_, std::string("map_3d_pcl"));
    subscriber_t cloud_subscriber_ = node_.subscribe<cloud_t>(topic_map_, 10, cloud_callback);

    cloud_ = boost::make_shared<cloud_t>();
    viewer_.setBackgroundColor (0.0, 0.0, 0.0);
    viewer_.addCoordinateSystem (1.0f, "reference");
    viewer_.initCameraParameters();
    viewer_.setCameraPosition(0, 30, 0, 0, 0, 0, 0, 0, 1);
    viewer_.setCameraFieldOfView(0.523599);
    viewer_.setCameraClipDistances(0.00522511, 50);
    viewer_.addPointCloud<point_t>(cloud_); 
    
    while (ros::ok()) {
    	if (map_locker_.try_lock()) {
    	    viewer_.spinOnce();
    	    map_locker_.unlock();
    	}
    	ros::spinOnce();
    }
}
