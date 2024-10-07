#include "definitions.h"


boost::mutex map_locker_;
viewer_t viewer_;
cloud_ptr_t cloud_;
pose_tf_t pose_;
bool pose_update_;


void cloud_callback(cloud_t cloud) {
    if (!map_locker_.try_lock()) return;
    for (auto point : cloud) {
        cloud_->push_back(point);
    }
    viewer_.updatePointCloud(cloud_);
    map_locker_.unlock();
}

void pose_callback(pose_msg_t pose) {
    pose_.position.x = pose.pose.position.x;
    pose_.position.y = pose.pose.position.y;
    pose_.position.z = pose.pose.position.z;
    pose_.rotation.x = std::atan2(pose.pose.orientation.z, pose.pose.orientation.w);
    pose_update_ = true;
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
    
    std::string topic_map_, topic_pose_, map_file_;
    handler_t node_, private_nh_("~");
    private_nh_.param("map_topic", topic_map_, std::string("map_3d_pcl"));
    private_nh_.param("file", map_file_, std::string(""));
    private_nh_.param("pose_topic", topic_pose_, std::string("slam_out_pose"));

    cloud_ = boost::make_shared<cloud_t>();
    viewer_.setBackgroundColor (0.0, 0.0, 0.0);
    viewer_.addCoordinateSystem (1.0f, "reference");
    viewer_.initCameraParameters();
    viewer_.setCameraPosition(0, 5, 0, 0, 0, 0, 0, 0, 1);
    viewer_.setCameraFieldOfView(0.523599);
    viewer_.setCameraClipDistances(0.00522511, 50);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);

    if (!map_file_.empty()) {
        map_file_ += ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_, *cloud_) == -1) {
            std::cout << "Failed to read file" << std::endl;
            return -1;
        }

        std::cout << "PointCloud size : " << cloud_->size() << std::endl;
        viewer_.addPointCloud<point_t>(cloud_);
        viewer_.updatePointCloud(cloud_);
        viewer_.spin();
        return 0;
    }
    
    subscriber_t cloud_subscriber_ = node_.subscribe<cloud_t>(topic_map_, 10, cloud_callback);
    subscriber_t pose_subscriber = node_.subscribe<pose_msg_t>(topic_pose_, 1, pose_callback);
    viewer_.addPointCloud<point_t>(cloud_); 
    while (ros::ok()) {
    	if (map_locker_.try_lock()) {
            if (pose_update_) { 
                viewer_.setCameraPosition(pose_.position.x + 5.0*cos(pose_.rotation.x), 
                                          pose_.position.y + 5.0*sin(pose_.rotation.x), 
                                          1, 
                                          pose_.position.x,
                                          pose_.position.y,
                                          1,
                                          0, 0, 1);
                pose_update_ = false;
            }
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
