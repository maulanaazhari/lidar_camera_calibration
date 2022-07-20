#ifndef _LIDAR_CAMERA_CALIBRATION_H_
#define _LIDAR_CAMERA_CALIBRATION_H_

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <math.h>

// #include <opencv.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class LidarVisualizer{
    private:
    // PARAMETERS DEFINITION
    std::string camera_topic, cloud_topic, info_topic, camera_frame, cloud_frame, cloud_out_topic, image_out_topic;
    bool info_available = false;
    bool tf_received = false;

    image_geometry::PinholeCameraModel cam_model;
    cv::Matx34d projection_matrix;

    tf::TransformListener tf_listener;
    geometry_msgs::TransformStamped transform;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher cloud_pub, image_pub;
    ros::Subscriber cam_info_subscriber;

    public:
    
    LidarVisualizer() : tf_listener_(tf_buffer_){

    }
    ~LidarVisualizer(){

    }

    void init(ros::NodeHandle& nh);

    void image_cloud_callback(
        const sensor_msgs::ImageConstPtr& image_msg, 
        const sensor_msgs::PointCloud2ConstPtr& cloud_msg
    );

    void camera_info_callback(
        const sensor_msgs::CameraInfoConstPtr& cam_info_msg
    );

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicyImageCloud;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageCloud>> SynchronizerImageCloud;

    SynchronizerImageCloud sync_image_cloud_;

    cv::Mat rgb_image;
    cv::Mat blurred_image;
    
        
};


#endif