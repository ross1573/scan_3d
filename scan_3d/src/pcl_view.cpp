#include "definitions.h"


boost::mutex map_locker_;
viewer_t viewer_;
cloud_ptr_t cloud_;


void cloud_callback(cloud_t cloud) {
    if (!map_locker_.try_lock()) return;
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
    
    std::string topic_map_, map_file_;
    handler_t node_, private_nh_("~");
    private_nh_.param("map_topic", topic_map_, std::string("map_3d_pcl"));
    private_nh_.param("file", map_file_, std::string(""));

    cloud_ = boost::make_shared<cloud_t>();
    viewer_.addPointCloud<point_t>(cloud_); 
    viewer_.setBackgroundColor (0.0, 0.0, 0.0);
    viewer_.addCoordinateSystem (1.0f, "reference");
    viewer_.initCameraParameters();
    viewer_.setCameraPosition(0, 30, 0, 0, 0, 0, 0, 0, 1);
    viewer_.setCameraFieldOfView(0.523599);
    viewer_.setCameraClipDistances(0.00522511, 50);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);

    if (!map_file_.empty()) {
        map_file_ += ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_, *cloud_) == -1) {
            std::cout << "Failed to read file" << std::endl;
            return -1;
        }

        viewer_.updatePointCloud(cloud_);
        viewer_.spin();
        return 0;
    }
    
    subscriber_t cloud_subscriber_ = node_.subscribe<cloud_t>(topic_map_, 10, cloud_callback);
    while (ros::ok()) {
    	if (map_locker_.try_lock()) {
    	    viewer_.spinOnce();
    	    map_locker_.unlock();
    	}
    	ros::spinOnce();
    }

    std::cout << "file name : ";
    std::getline(std::cin, map_file_);
    if (map_file_ != "") {
        map_file_ += ".pcd";
        pcl::io::savePCDFileASCII(map_file_, *cloud_);
        std::cout << "file successfully saved" << std::endl;
    }
}
