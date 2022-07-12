#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>


typedef ros::NodeHandle 			        handler_t;
typedef ros::Subscriber 				    subscriber_t;
typedef ros::Publisher 				        publisher_t;
typedef ros::ServiceServer				    service_t;
typedef ros::ServiceClient				    client_t;

typedef geometry_msgs::Pose 			    pose_t;
typedef geometry_msgs::PoseStamped 		    pose_msg_t;
typedef geometry_msgs::Point 			    position_t;
typedef sensor_msgs::LaserScan 			    laser_t;
typedef pcl::PointXYZ					    point_t;
typedef pcl::PointCloud<point_t> 		    cloud_t;
typedef pcl::PointCloud<point_t>::Ptr 	    cloud_ptr_t;
typedef sensor_msgs::PointCloud2 		    cloud_msg_t;
typedef laser_geometry::LaserProjection     projector_t;

typedef Eigen::Matrix4f				        matrix4;

typedef pcl::visualization::PCLVisualizer 	viewer_t;
typedef tf::TransformListener 			    tf_listener_t;


typedef struct Pose_ {
    geometry_msgs::Point position;
    geometry_msgs::Point rotation;
} pose_tf_t;


#endif