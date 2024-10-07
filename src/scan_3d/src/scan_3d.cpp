#include "scan_3d.h"


Scan3D::Scan3D()
: private_nh_("~")
, first_scan_(false)
, pose_update_(false)
, cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
    std::string topic_scan_, topic_pose_, topic_map_;
    
    private_nh_.param("map_update_distance_thresh", map_update_distance_thresh_, 0.01);
    private_nh_.param("map_update_angle_thresh", map_update_angle_thresh_, 0.01);
    private_nh_.param("scan_topic", topic_scan_, std::string("scan_3d"));
    private_nh_.param("pose_topic", topic_pose_, std::string("slam_out_pose"));
    private_nh_.param("map_topic", topic_map_, std::string("map_3d"));
    private_nh_.param("visualize_rivz", visualize_rviz_, false);
    private_nh_.param("visualize_pcl", visualize_pcl_, true);
    private_nh_.param("ground_search_angle", ground_search_angle_, 5.0);
    private_nh_.param("use_trigonometric_ground_search", trigonometric_ground_search_, true);
    
    scan_subscriber_ = node_.subscribe(topic_scan_, 5, &Scan3D::scanCallback, this);
    pose_subscriber_ = node_.subscribe(topic_pose_, 1, &Scan3D::poseCallback, this);
    map_publisher_ = node_.advertise<cloud_msg_t>(topic_map_, 10, false);
    map_pcl_publisher_ = node_.advertise<cloud_t>(topic_map_ + "_pcl", 10, false);
    
    pose_.rotation.x =  0.0;
    pose_.rotation.y = -acos(-1.0)/2.0;
    pose_.rotation.z =  0.0;
}

Scan3D::~Scan3D() {}


void Scan3D::scanCallback(const laser_t& scan) { 
    tf::StampedTransform laser_transform;
    if (tfListener_.waitForTransform("base_link", scan.header.frame_id, scan.header.stamp, ros::Duration(0.5))) {
    	tfListener_.lookupTransform("base_link", scan.header.frame_id, scan.header.stamp, laser_transform);
    }
    else {
      	std::cout << "lookupTransform failed" << std::endl;
     	return;
    }
    
    if (!first_scan_) first_scan_ = true;
    if (!pose_update_) return;
    
    auto transformed = transformScan(scan);
    storeCloud(transformed);
    publishCloud(transformed);
}

void Scan3D::poseCallback(const pose_msg_t& pose_msg) {
    if (map_update_distance_thresh_ > 0.0 || map_update_angle_thresh_ > 0.0) {
        pose_tf_t diff_ = subPose(pose_msg.pose);
        double diff_dis_ = std::pow(diff_.position.x, 2) + std::pow(diff_.position.y, 2);
        double diff_ang_ = std::pow(diff_.rotation.x, 2);
    	if (diff_dis_ < std::pow(map_update_distance_thresh_, 2) && diff_ang_ < std::pow(map_update_angle_thresh_, 2)) return;
    }

    updatePose(pose_msg.pose);
}


cloud_ptr_t Scan3D::transformScan(const laser_t& scan) {
    cloud_msg_t cloud;
    cloud_t cloud_pcl;
    cloud_ptr_t cloud_pcl_tf(new pcl::PointCloud<pcl::PointXYZ>);

    projector_.projectLaser(scan, cloud, -1);							
    cloud.header.frame_id = "base_link";
    pcl::fromROSMsg(cloud, cloud_pcl);

    searchGround(scan);    
    matrix4 tf = createTransformMatrix(1.0);
    pcl::transformPointCloud(cloud_pcl, *cloud_pcl_tf, tf);
    
    return cloud_pcl_tf;
}

matrix4 Scan3D::createTransformMatrix(double position_factor) {
    double x = pose_.rotation.z;    /* yaw */
    double y = pose_.rotation.y;	/* pitch */
    double z = pose_.rotation.x;	/* roll */

    matrix4 tf;
    tf << cos(z)*cos(y), -sin(z), cos(z)*sin(x), pose_.position.x * position_factor,                                                                // Roll, Yaw, Pitch transform matrix
          cos(x)*cos(y)*sin(z) + sin(x)*sin(y), cos(x)*cos(z), cos(x)*sin(z)*sin(y) - sin(x)*cos(y), pose_.position.y * position_factor,
          sin(x)*sin(z)*cos(y) - cos(x)*sin(y), sin(x)*cos(z), sin(x)*sin(z)*sin(y) + cos(x)*cos(y), pose_.position.z,
          0, 0, 0, 1.0;
    return tf;
}

void Scan3D::searchGround(const laser_t& scan) {
    if (trigonometric_ground_search_) {
        pose_.rotation.z = searchTrigonometric(scan);
        if (pose_.rotation.z == 0.0) pose_.rotation.z = last_pose_.rotation.z;
        pose_.position.z = scan.ranges[0] * cos(pose_.rotation.z);
    }
}

double Scan3D::searchTrigonometric(const laser_t& scan) {
    static const double pi = acos(-1);
    const int total_scan = scan.scan_time / scan.time_increment;
    const double angle_min = pi + scan.angle_min;
    const double angle_max = pi - scan.angle_max;
    const double angle_inc = scan.angle_increment;

    double ceta = angle_min;
    double sum = 0.0;
    double distance[2];
    int end_index = total_scan - static_cast<int>((angle_min - angle_max) / angle_inc) - 1;
    int count = 0;

    for (int i = 0; i < total_scan; i++) {
        if (ceta > ground_search_angle_) break;
        if (ceta < angle_max) {
            ceta += scan.angle_increment;
            continue;
        }

        distance[0] = scan.ranges[i];
        distance[1] = scan.ranges[end_index-i];
        if (isnan(distance[0]) || isnan(distance[1])) {
            ceta += scan.angle_increment;
            continue;
        }

        double tan_alpha = atan((distance[0] - distance[1]) / ((distance[0] + distance[1]) * tan(ceta)));
        if (!isnan(tan_alpha)) {
            sum += tan_alpha;
            count += 1;
        }
        ceta += scan.angle_increment;
    }
    
    if (count == 0) return 0.0;
    return sum / static_cast<double>(count) * -1.0;
}


void Scan3D::storeCloud(cloud_ptr_t cloud) {
    if (!map_locker_.try_lock()) return;
    for (auto point : *cloud) {
    	cloud_->push_back(point);
    }
    map_locker_.unlock();
}

void Scan3D::publishCloud(cloud_ptr_t cloud) {
    if (visualize_rviz_) {
    	cloud_msg_t cloud;
    	pcl::toROSMsg(*cloud_, cloud);
    	cloud.header.frame_id = "base_link";
    	map_publisher_.publish(cloud);
    }
    if (visualize_pcl_) {
    	map_pcl_publisher_.publish(*cloud);
    }
    pose_update_ = false;
}


pose_tf_t Scan3D::subPose(const pose_t& pose) {
    pose_tf_t new_p;
    new_p.position.x = pose.position.x - pose_.position.x;
    new_p.position.y = pose.position.y - pose_.position.y;
    new_p.position.z = pose.position.z - pose_.position.z;
    new_p.rotation.x = std::atan2(pose.orientation.z, pose.orientation.w) - pose_.rotation.x;
    return new_p;
}

void Scan3D::updatePose(const pose_t& pose) {
    last_pose_.position.x = pose_.position.x;
    last_pose_.position.y = pose_.position.y;
    last_pose_.position.z = pose_.position.z;
    last_pose_.rotation.x = pose_.rotation.x;
    last_pose_.rotation.y = pose_.rotation.y;
    last_pose_.rotation.z = pose_.rotation.z;

    pose_.position.x = pose.position.x;
    pose_.position.y = pose.position.y;
    pose_.position.z = pose.position.z;
    pose_.rotation.x = std::atan2(pose.orientation.z, pose.orientation.w) * 2.0;
    pose_update_ = true;
}
