#include <lidar_camera_calibration/lidar_visualizer.h>

void LidarVisualizer::init(ros::NodeHandle& nh) {

  nh.param<std::string>("camera_topic", camera_topic, "camera_topic");
  nh.param<std::string>("info_topic", info_topic, "info_topic");
  nh.param<std::string>("cloud_topic", cloud_topic, "cloud_topic");
  nh.param<std::string>("cloud_out_topic", cloud_out_topic, "cloud_out_topic");
  nh.param<std::string>("image_out_topic", image_out_topic, "image_out_topic");
  nh.param<std::string>("camera_frame", camera_frame, "camera_frame");
  nh.param<std::string>("cloud_frame", cloud_frame, "cloud_frame");

  std::cout<<"PARAMETERS : "<< std::endl;
  std::cout<<"camera_topic : "<< camera_topic << std::endl;
  std::cout<<"info_topic : "<< info_topic << std::endl;
  std::cout<<"cloud_topic : "<< cloud_topic << std::endl;
  std::cout<<"cloud_out_topic : "<< cloud_out_topic << std::endl;
  std::cout<<"image_out_topic : "<< image_out_topic << std::endl;
  std::cout<<"camera_frame : "<< camera_frame << std::endl;
  std::cout<<"cloud_frame : "<< cloud_frame << std::endl;

  cam_info_subscriber = nh.subscribe(info_topic, 1, &LidarVisualizer::camera_info_callback, this);
  cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(cloud_out_topic, 1);
  image_pub = nh.advertise<sensor_msgs::Image>(image_out_topic, 1 );

  while (!tf_received){
    ROS_INFO("Waiting for transform between %s to %s...", cloud_frame.c_str(), camera_frame.c_str());

    try{
      transform = tf_buffer_.lookupTransform(camera_frame, cloud_frame, ros::Time::now());
      tf_received = true;
    }
    catch (tf2::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("Tranformation is received!...");

  image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, camera_topic, 10));
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, cloud_topic, 10));
  
  sync_image_cloud_.reset(new message_filters::Synchronizer<SyncPolicyImageCloud>(
    SyncPolicyImageCloud(100), *image_sub_, *cloud_sub_
  ));
  
  sync_image_cloud_->registerCallback(boost::bind(&LidarVisualizer::image_cloud_callback, this, _1, _2));
}

void LidarVisualizer::image_cloud_callback(
  const sensor_msgs::ImageConstPtr& image_msg, 
  const sensor_msgs::PointCloud2ConstPtr& cloud_msg
)
  {
    // Solve all of perception here...
    if (!info_available){
      ROS_INFO("Camera info is not available, waiting...");
    }
    else {
      cv_bridge::CvImagePtr img_ptr;
      try
      {
        img_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        // cv::cvtColor(img_ptr->image, img_ptr->image, CV_BGR2RGB);
        cv::GaussianBlur(img_ptr->image, img_ptr->image, cv::Size(13,13), 0);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      sensor_msgs::PointCloud2 cloud_msg_out;
      tf2::doTransform(*cloud_msg, cloud_msg_out, transform);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(cloud_msg_out, *cloud);

      for (unsigned int i = 0; i < cloud->points.size(); i++){
        cv::Point2d cam2_point= cam_model.project3dToPixel(cv::Point3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));

        if (!(
          cloud->points[i].z > 0
          // cloud->points[i].z < 6 &&
          // abs(cloud->points[i].x) < 6 &&
          // abs(cloud->points[i].y) <6
        )){
          continue;
        }

        if (
          cam2_point.x >= 0 &&
          cam2_point.x <= img_ptr->image.cols &&
          cam2_point.y >= 0 &&
          cam2_point.y <= img_ptr->image.rows
        ){
          // std::cout<< cam2_point.x << " " << cam2_point.y << std::endl;
          pcl::PointXYZRGB new_point;
          new_point.x = cloud->points[i].x;
          new_point.y = cloud->points[i].y;
          new_point.z = cloud->points[i].z;
          new_point.r = img_ptr->image.at<cv::Vec3b>(round(cam2_point.y), round(cam2_point.x))[2];
          new_point.g = img_ptr->image.at<cv::Vec3b>(round(cam2_point.y), round(cam2_point.x))[1];
          new_point.b = img_ptr->image.at<cv::Vec3b>(round(cam2_point.y), round(cam2_point.x))[0];
          cv::circle(img_ptr->image, cv::Point(round(cam2_point.x), round(cam2_point.y)), 2, cv::Scalar(new_point.b, new_point.g, new_point.r), 2);

          rgb_cloud->push_back(new_point);
        }
      }

      if (rgb_cloud->points.size() != 0){
        rgb_cloud->header.frame_id = cloud->header.frame_id;
        cloud_pub.publish(rgb_cloud);

        // cv::cvtColor(img_ptr->image, img_ptr->image, CV_RGB2BGR);
        image_pub.publish(img_ptr->toImageMsg());
      }

      }
  }

void LidarVisualizer::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
  if (!info_available){
    cam_model.fromCameraInfo(cam_info_msg);
    info_available = true;
    std::cout<<"Camera model is ready!!!"<<std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_visualizer");
  ros::NodeHandle nh("~");
  LidarVisualizer lidar_visualizer;
  lidar_visualizer.init(nh);
  
  ros::spin();

  return 0;
}