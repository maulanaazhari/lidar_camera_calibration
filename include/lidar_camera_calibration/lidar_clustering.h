#ifndef _LIDAR_CLUSTERING_H_
#define _LIDAR_CLUSTERING_H_


#include <iostream>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"


class LidarClustering{
    public:
    LidarClustering(){}
    ~LidarClustering(){}

    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;

    void init(ros::NodeHandle& nh);

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};



#endif