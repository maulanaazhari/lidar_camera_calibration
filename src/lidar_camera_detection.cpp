#include <lidar_camera_calibration/lidar_camera_detection.h>
#include "pycoral_ros/DetectImage.h"

void LidarCameraDetection::init(ros::NodeHandle& nh) {

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

  cam_info_subscriber = nh.subscribe(info_topic, 1, &LidarCameraDetection::camera_info_callback, this);
  cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(cloud_out_topic, 1);
  image_pub = nh.advertise<sensor_msgs::Image>(image_out_topic, 1 );

  detection_client = nh.serviceClient<pycoral_ros::DetectImage>("/detect_image");

  detection_pub = nh.advertise<vision_msgs::Detection3DArray>("/detection_3d", 1 );
  detection_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/detection_3d_pub", 1 );

  while (!tf_received){
    ROS_INFO("Waiting for transform between %s to %s...", cloud_frame.c_str(), camera_frame.c_str());

    try{
      transform = tf_buffer_.lookupTransform(camera_frame, cloud_frame, ros::Time::now());
      inverse_transform = tf_buffer_.lookupTransform(cloud_frame, camera_frame, ros::Time::now());
      tf_received = true;

    }
    catch (tf2::TransformException ex){
      ROS_INFO("%s",ex.what());
    }
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("Tranformation is received!...");

  image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, camera_topic, 10));
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, cloud_topic, 10));
  
  sync_image_cloud_.reset(new message_filters::Synchronizer<SyncPolicyImageCloud>(
    SyncPolicyImageCloud(100), *image_sub_, *cloud_sub_
  ));
  
  sync_image_cloud_->registerCallback(boost::bind(&LidarCameraDetection::image_cloud_callback, this, _1, _2));
}

void LidarCameraDetection::visualize_detection(
  vision_msgs::Detection3DArray dets
){
  visualization_msgs::MarkerArray ms;
  
  for (unsigned i=0; i<dets.detections.size(); i++){
    visualization_msgs::Marker m;
    m.header.frame_id = cloud_frame;
    m.pose.position = dets.detections[i].results[0].pose.pose.position;
    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;

    m.color.a = 1;
    m.color.r = rand() % 230;
    m.color.g = rand() % 230;
    m.color.b = rand() % 230;

    m.type = 2;
    m.id = i;

    m.text = std::to_string(dets.detections[i].results[0].id);
    ms.markers.push_back(m);
  }

  detection_vis_pub.publish(ms);
}

void LidarCameraDetection::image_cloud_callback(
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
      // tf2::doTransform()

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(cloud_msg_out, *cloud);


      // Filtering the cloud points
      std::vector<Eigen::Vector2d> cam_points;
      std::vector<Eigen::Vector3d> cloud_points;

      for (unsigned int i = 0; i < cloud->points.size(); i++){
        cv::Point3d cloud_point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        cv::Point2d cam_point = cam_model.project3dToPixel(cloud_point);

        if (
          cam_point.x >= 0 &&
          cam_point.x <= img_ptr->image.cols &&
          cam_point.y >= 0 &&
          cam_point.y <= img_ptr->image.rows &&
          cloud->points[i].z > 0
        ){
          cam_points.push_back(Eigen::Vector2d(cam_point.x, cam_point.y));
          cloud_points.push_back(Eigen::Vector3d(cloud_point.x, cloud_point.y, cloud_point.z));
        }
      }

      // Calling object detection service
      pycoral_ros::DetectImage img_srv;
      img_srv.request.image = *image_msg;

      if (detection_client.call(img_srv)){
        vision_msgs::Detection3DArray det3ds;

        det3ds.header.frame_id = cloud_msg->header.frame_id;

        for (unsigned int i = 0; i < img_srv.response.detections.detections.size(); i++){
          vision_msgs::Detection2D det2d = img_srv.response.detections.detections[i];
          
          Eigen::Vector2d detection_center(det2d.bbox.center.x, det2d.bbox.center.y);
          Eigen::Vector3d location_3d(999.9, 999.9, 999.9);

          for (unsigned int j = 0; j < cam_points.size(); j++){
            if((cam_points[j]-detection_center).norm() < 10){
              if(cloud_points[j].norm() < location_3d.norm()){
                location_3d = cloud_points[j];
              }
            }
          }

          // If detection distance from camera is bigger than 50 meters, then invalid
          if (location_3d.norm() > 50){
            continue;
          }

          vision_msgs::Detection3D det3d;
          vision_msgs::ObjectHypothesisWithPose hypotesis;
          Eigen::Vector3d location_3d_world;

          Eigen::Matrix3d matRot = Eigen::Quaterniond(
            inverse_transform.transform.rotation.w, inverse_transform.transform.rotation.x, inverse_transform.transform.rotation.y, inverse_transform.transform.rotation.z
          ).toRotationMatrix();
          Eigen::Vector3d matTrans(inverse_transform.transform.translation.x, inverse_transform.transform.translation.y, inverse_transform.transform.translation.z);

          location_3d_world = matRot*location_3d + matTrans;

          hypotesis.id = det2d.results[0].id;
          hypotesis.score = det2d.results[0].score;

          hypotesis.pose.pose.position.x = location_3d_world[0];
          hypotesis.pose.pose.position.y = location_3d_world[1];
          hypotesis.pose.pose.position.z = location_3d_world[2];
          
          det3d.results.push_back(hypotesis);
          det3d.bbox.center.position.x = location_3d_world[0];
          det3d.bbox.center.position.y = location_3d_world[1];
          det3d.bbox.center.position.z = location_3d_world[2];

          det3ds.detections.push_back(det3d);
        }

        detection_pub.publish(det3ds);
        LidarCameraDetection::visualize_detection(det3ds);

      }
      else{
        ROS_ERROR("Failed to call service detect_image");
        // return 1;
      }

    }
}

void LidarCameraDetection::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
  if (!info_available){
    cam_model.fromCameraInfo(cam_info_msg);
    info_available = true;
    std::cout<<"Camera model is ready!!!"<<std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_camera_detection");
  ros::NodeHandle nh("~");
  LidarCameraDetection lidar_camera_detection;
  lidar_camera_detection.init(nh);
  
  ros::spin();

  return 0;
}