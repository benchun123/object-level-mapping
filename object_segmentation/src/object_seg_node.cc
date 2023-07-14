#include <stdio.h>
#include <string>
#include <iostream>

// ros
#include <ros/ros.h>
// #include <ros/package.h> // use function ros::package::getPath("acdu_numberplate")
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <boost/make_shared.hpp>
#include <geometry_msgs/PolygonStamped.h>

#include <glog/logging.h>

// ours
#include "ObjectDetector.h"
// #include "MapObject.h"

using namespace std;


class ObjectSegmentationNode {
 public:
  ObjectSegmentationNode()
      : node_handle_("~"),
        image_transport_(node_handle_),
        camera_info_ready_(false){


    node_handle_.param<std::string>("depth_image_sub_topic", depth_image_topic_,
                                    object_segmentation::kDepthImageTopic);
    node_handle_.param<std::string>("rgb_image_sub_topic", rgb_image_topic_,
                                    object_segmentation::kRgbImageTopic);
    node_handle_.param<std::string>("depth_camera_info_sub_topic",
                                    depth_camera_info_topic_,
                                    object_segmentation::kDepthCameraInfoTopic);
    node_handle_.param<std::string>("rgb_camera_info_sub_topic",
                                    rgb_camera_info_topic_,
                                    object_segmentation::kRgbCameraInfoTopic);
    node_handle_.param<std::string>("world_frame", world_frame_,
                                    object_segmentation::kTfWorldFrame);
    node_handle_.param<std::string>("local_frame", local_frame_,
                                    object_segmentation::kTfLocalFrame);
    node_handle_.param<std::string>("pub_obj_pcl_topic", pub_obj_pcl_topic_,
                                    object_segmentation::kObjPCLTopic);
    node_handle_.param<std::string>("pub_raw_pcl_topic", pub_raw_pcl_topic_,
                                    object_segmentation::kRawPCLTopic);
    node_handle_.param<bool>("pub_in_local_frame", pub_in_local_frame_,
                                    object_segmentation::kPubLocalFrame);
    node_handle_.param<std::string>("config_file", config_file_,
                                    object_segmentation::kConfigFile);
    node_handle_.param<std::string>("dataset_path", dataset_path_,
                                    object_segmentation::kDatasetPath);

    depth_image_sub_ = new image_transport::SubscriberFilter(
        image_transport_, depth_image_topic_, 1);
    rgb_image_sub_ = new image_transport::SubscriberFilter(image_transport_,
                                                           rgb_image_topic_, 1);
    depth_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(
        node_handle_, depth_camera_info_topic_, 1);
    rgb_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(
        node_handle_, rgb_camera_info_topic_, 1);

    constexpr int kQueueSize = 30;

    image_sync_policy_ = new message_filters::Synchronizer<ImageSyncPolicy>(
        ImageSyncPolicy(kQueueSize), *depth_image_sub_, *rgb_image_sub_);

    image_sync_policy_->registerCallback(
        boost::bind(&ObjectSegmentationNode::imageCallback, this, _1, _2));


    camera_info_sync_policy_ =
        new message_filters::Synchronizer<CameraInfoSyncPolicy>(
            CameraInfoSyncPolicy(kQueueSize), *depth_info_sub_, *rgb_info_sub_);

    camera_info_sync_policy_->registerCallback(
        boost::bind(&ObjectSegmentationNode::cameraInfoCallback, this, _1, _2));

    point_cloud2_segment_pub_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(pub_obj_pcl_topic_,1);
    point_cloud2_raw_pub_ = 
        node_handle_.advertise<sensor_msgs::PointCloud2>(pub_raw_pcl_topic_,1);

    bbox_image_pub_ = image_transport_.advertise("/bbox_image", 1);

    // read configuration
    cv::FileStorage fSettings(config_file_, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        std::cerr << "config file not found" << std::endl;
    }
    else
    {
      // param, just load once
      obj_det_.MultiPlane_SizeMin = int(fSettings["MultiPlane.SizeMin"]);
      obj_det_.MultiPlane_AngleThre = float(fSettings["MultiPlane.AngleThre"]);
      obj_det_.MultiPlane_DistThre = float(fSettings["MultiPlane.DistThre"]);
      obj_det_.ground_min = fSettings["Ground.min"];
      obj_det_.ground_max = fSettings["Ground.max"];
      obj_det_.wall_min = fSettings["Wall.min"];
      obj_det_.wall_max = fSettings["Wall.max"];

      // Option 1: get tf base to cam manually
      Eigen::VectorXd tf_base_cam(7);
      tf_base_cam << fSettings["T_base_cam.x"], fSettings["T_base_cam.y"],
                    fSettings["T_base_cam.z"], fSettings["T_base_cam.qx"],
                    fSettings["T_base_cam.qy"], fSettings["T_base_cam.qz"],
                    fSettings["T_base_cam.qw"];
      // Eigen::Matrix4d Tbc;	// baselink to cam
      Tbc.setIdentity();
      Tbc.block(0,0,3,3) = Eigen::Quaterniond(tf_base_cam(6),tf_base_cam(3),tf_base_cam(4),tf_base_cam(5)).toRotationMatrix();
      Tbc.col(3).head(3) = Eigen::Vector3d(tf_base_cam(0), tf_base_cam(1), tf_base_cam(2));
      cout << "- Tbc: \n" << Tbc << endl;

      // // Option 2: get tf base to cam from calibration
      // // tf laser to camera: pay attention !!! from laser to camera
      // // calibration result: https://github.com/MegviiRobot/CamLaserCalibraTool/blob/master/README.md
      // cv::Mat extrinsicTlc(4,4,CV_32FC1);
      // fSettings["extrinsicTlc"] >> extrinsicTlc;
      // std::cout <<"Load extrinsicTlc: \n"<<extrinsicTlc<<std::endl;
      // Eigen::Matrix4d Tlc;
      // Tlc.setIdentity();
      // if(!extrinsicTlc.empty())
      //     cv::cv2eigen(extrinsicTlc, Tlc);
      // else
      //     std::cerr <<" You Do not have calibra result Tlc. We use a Identity matrix." << std::endl;
      // std::cout <<"Tlc: \n"<<Tlc<<std::endl;
      // Eigen::Matrix4d rotate_z_180; // in agiprobot setting, the camera is opposite
      // rotate_z_180 << -1.0, 0.0, 0.0, 0.0,
      // 				0.0, -1.0, 0.0, 0.0,
      // 				0.0, 0.0, 1.0, 0.0,
      // 				0.0, 0.0, 0.0, 1.0;
      // Tlc = Tlc * rotate_z_180;
      // std::cout <<"Tlc: \n"<<Tlc<<std::endl;
      // Eigen::VectorXd tf_base_laser(7); // tf from base to laser
      // tf_base_laser << fSettings["T_base_laser.x"], fSettings["T_base_laser.y"],
      // 				fSettings["T_base_laser.z"], fSettings["T_base_laser.qx"],
      // 				fSettings["T_base_laser.qy"], fSettings["T_base_laser.qz"],
      // 				fSettings["T_base_laser.qw"];
      // Eigen::Matrix4d Tbl;	// baselink to laser
      // Tbl.setIdentity();
      // Tbl.block(0,0,3,3) = Eigen::Quaterniond(tf_base_laser(6),tf_base_laser(3),tf_base_laser(4),tf_base_laser(5)).toRotationMatrix();
      // Tbl.col(3).head(3) = Eigen::Vector3d(tf_base_laser(0), tf_base_laser(1), tf_base_laser(2));
      // std::cout <<"Tbl: \n"<<Tbl<<std::endl;
      // // Eigen::Matrix4d Tbc;	// baselink to cam
      // Tbc = Tbl * Tlc;

      // transfrom odom to baselink
      // laser odom, laser movement, output is similar to wheel odom but accurate
      // tranform from odom to base_link, odom is fixed, base_link is moving
      std::string odom_file = dataset_path_ + std::string(fSettings["odom_asso_file"]);
      cout << "-odom_file(global): " << odom_file << endl;
      // std::vector<std::string> vRGBIndex; // global
      // std::vector<std::string> vDepthIndex; // global
      // std::vector<Eigen::VectorXd> vOdomIndex; // global
      LoadImagesIndex(odom_file, vRGBIndex, vDepthIndex, vOdomIndex);

      // transfrom map to odom
      Eigen::VectorXd trans_map_odom(7); // tf map to odom, initial pose, fixed
      trans_map_odom << 5.3140, 3.5875, 0, 0, 0, 0, 1 ; // read from tf tree
      // Eigen::Matrix4d Tmo;	// map to odom
      Tmo.setIdentity();
      Tmo.block(0,0,3,3) = Eigen::Quaterniond(trans_map_odom(6),trans_map_odom(3),trans_map_odom(4),trans_map_odom(5)).toRotationMatrix();
      Tmo.col(3).head(3) = Eigen::Vector3d(trans_map_odom(0), trans_map_odom(1), trans_map_odom(2));
      cout << "- Tmo: \n" << Tmo << endl;
    }

  }

 private:
  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  tf::TransformBroadcaster tf_br_pub_;
  tf::TransformListener tf_listener_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      ImageSyncPolicy;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
      CameraInfoSyncPolicy;

  bool camera_info_ready_;

 private:
  std::string rgb_image_topic_;
  std::string rgb_camera_info_topic_;
  std::string depth_image_topic_;
  std::string depth_camera_info_topic_;
  std::string world_frame_;
  std::string local_frame_;
  std::string pub_obj_pcl_topic_;
  std::string pub_raw_pcl_topic_;
  std::string config_file_;
  std::string dataset_path_;
  bool pub_in_local_frame_;

  std::vector<std::string> vRGBIndex; // global
  std::vector<std::string> vDepthIndex; // global
  std::vector<Eigen::VectorXd> vOdomIndex; // global
  Eigen::Matrix4d Tmo;	// map to odom
  Eigen::Matrix4d Tbc;	// baselink to cam

  object_segmentation::ObjectDetector obj_det_;

  image_transport::SubscriberFilter* depth_image_sub_;
  image_transport::SubscriberFilter* rgb_image_sub_;

  message_filters::Subscriber<sensor_msgs::CameraInfo>* depth_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo>* rgb_info_sub_;

  ros::Publisher point_cloud2_segment_pub_;
  ros::Publisher point_cloud2_raw_pub_;
  image_transport::Publisher bbox_image_pub_; 

  message_filters::Synchronizer<ImageSyncPolicy>* image_sync_policy_;
  message_filters::Synchronizer<CameraInfoSyncPolicy>* camera_info_sync_policy_;


  void publish_segments(const pcl::PointCloud<pcl::PointXYZRGB>& segment_pcl,
                        const std_msgs::Header& header) 
    {
      // CHECK_GT(segments.size(), 0u);
      std::cout << "segment_pcl.size "<< segment_pcl.size() << std::endl;

      pcl::PointCloud<pcl::PointXYZRGBL> segment_new;
      segment_new.points.resize(segment_pcl.size());
      for (size_t i = 0; i < segment_pcl.points.size(); i++) {
          segment_new.points[i].x = segment_pcl.points[i].x;
          segment_new.points[i].y = segment_pcl.points[i].y;
          segment_new.points[i].z = segment_pcl.points[i].z;
          segment_new.points[i].label = 1;
      }


      sensor_msgs::PointCloud2 pcl2_msg;
      pcl::toROSMsg(segment_new, pcl2_msg);
      pcl2_msg.header.stamp = header.stamp;
      pcl2_msg.header.frame_id = header.frame_id;
      point_cloud2_segment_pub_.publish(pcl2_msg);
    }

  void imageCallback(const sensor_msgs::Image::ConstPtr& depth_msg,
                     const sensor_msgs::Image::ConstPtr& rgb_msg) {
    double timestamps_tmp = rgb_msg->header.stamp.toSec();
    std::cout << "------------ frame_index " << std::to_string(timestamps_tmp) << "------------" << std::endl;
    //---- step 1 check camera information ---- //
    if (!camera_info_ready_) 
    {
      std::cout << "camera_info_ready_ not !!!" << std::endl;
      return;
    }

    //---- step 2 check rgb and depth image ---- //
    cv_bridge::CvImagePtr cv_rgb_image(new cv_bridge::CvImage);
    cv_rgb_image = cv_bridge::toCvCopy(rgb_msg, "bgr8"); // original encoding is bgra8
    // if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    //     cv::cvtColor(cv_rgb_image->image, cv_rgb_image->image, CV_BGR2RGB);
    // }
    obj_det_.imRGB = cv_rgb_image->image;
    obj_det_.imRGB = rotate(cv_rgb_image->image, 180);
    // std::cout << "get rgb !!!" << rgb_msg->encoding << std::endl;

    // check depth image
    cv::Mat rescaled_depth;
    cv_bridge::CvImagePtr cv_depth_image(new cv_bridge::CvImage);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) 
    {
      cv_depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      rescaled_depth = cv::Mat::zeros(cv_depth_image->image.size(), CV_32FC1);
      cv::rgbd::rescaleDepth(cv_depth_image->image, CV_32FC1, rescaled_depth);
      // std::cout << "get depth TYPE_16UC1 !!!" << std::endl;
    } 
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) 
    {
      cv_depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      rescaled_depth = cv_depth_image->image;
    } 
    else 
    {
      LOG(FATAL) << "Unknown depth image encoding.";
    }
    obj_det_.imDepth = rescaled_depth;
    obj_det_.imDepth = rotate(rescaled_depth, 180);
    // std::cout << "get depth !!!" << std::endl;



    //---- step 3 check odom message ---- //
    // get stamp between baselink and odom
    tf::StampedTransform odom_stamp;
    try{
      tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), odom_stamp);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    // check odom from start position: option 1, read from association file, laser odom
    std::string rgb_timestamp = std::to_string(rgb_msg->header.stamp.toSec());
    // ros::Time test_stamp;
    Eigen::VectorXd odom(7);
    auto it = std::find(vRGBIndex.begin(), vRGBIndex.end(), rgb_timestamp);
    if (it == vRGBIndex.end())
    {
      std::cout << "current rgb has no corrsponding laser odom" << std::endl;
      return;
    }
    else
    {
      auto index = std::distance(vRGBIndex.begin(), it);
      // int index = it - vRGBIndex.begin();
      odom = vOdomIndex[index];
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(odom(0), odom(1), odom(2)) );
      tf::Quaternion q(odom(3),odom(4),odom(5), odom(6));
      // q.setRPY(0, 0, msg->theta);
      transform.setRotation(q);
      tf_br_pub_.sendTransform(tf::StampedTransform(transform, odom_stamp.stamp_, "odom", "new_base_link"));
    }
    Eigen::Matrix4d Tob;	// laser movement, origin: base link
    Tob.setIdentity();
    Tob.block(0,0,3,3) = Eigen::Quaterniond(odom(6),odom(3),odom(4),odom(5)).toRotationMatrix();
    Tob.col(3).head(3) = Eigen::Vector3d(odom(0), odom(1), odom(2));

    // // check odom from start position: option 2, read from tf tree wheel odom
    // tf::StampedTransform odom_transfrom;
    // Eigen::Matrix4d Trans_base_odom;
    // Trans_base_odom.setIdentity();
    // try{
    //   // tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), odom_transfrom);
    //   tf_listener_.lookupTransform("map", "base_link", ros::Time(0), odom_transfrom);
    //   Eigen::Vector3d trans = Eigen::Vector3d(odom_transfrom.getOrigin().x(), odom_transfrom.getOrigin().y(), odom_transfrom.getOrigin().z());
    //   Eigen::Quaterniond quat = Eigen::Quaterniond(odom_transfrom.getRotation().w(), odom_transfrom.getRotation().x(),
    //                             odom_transfrom.getRotation().y(), odom_transfrom.getRotation().z());
    //   Trans_base_odom.block(0,0,3,3) = quat.toRotationMatrix();
    //   Trans_base_odom.col(3).head(3) = trans;
    //   obj_det_.Tbo = Trans_base_odom;
    //   obj_det_.Twc = obj_det_.Tbo * obj_det_.Tcb;
    //   std::cout << "Trans_base_odom" << Trans_base_odom << std::endl;
    // }
    // catch (tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    //     ros::Duration(1.0).sleep();
    // }


    //---- step 4 check world coodinate and Twc ---- //
    // // check transfrom, publish in local frame
    cv::FileStorage fSettings(config_file_, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        std::cerr << "config file not found" << std::endl;
    }
    std::string label_path = dataset_path_+"/"+fSettings["label_folder"];
    double header_name = rgb_msg->header.stamp.toSec();
    std::string bbox_file = label_path + "/" + std::to_string(header_name) + ".txt";

    if(pub_in_local_frame_)
    {
      if(local_frame_ == "new_base_link")
        obj_det_.Twc = Tbc; // base_link to camera
      else
        obj_det_.Twc = Tob * Tbc; // odom to camera

      bool detect_object = obj_det_.ReadBBox2DLabelCloud(bbox_file);
      if(detect_object)
      {
        for ( int m=0; m<obj_det_.objectLabelCloudPoints.size(); m++)
        {
          sensor_msgs::PointCloud2 pcl2_msg;
          pcl::toROSMsg(obj_det_.objectLabelCloudPoints[m], pcl2_msg);
          pcl2_msg.header.stamp = odom_stamp.stamp_;
          pcl2_msg.header.frame_id = local_frame_;
          point_cloud2_segment_pub_.publish(pcl2_msg);
        }
      }

      // // pub_raw_pcl
      // std::cout << "obj_det_.Twc \n" << obj_det_.Twc << std::endl;
      // obj_det_.ComputeLabelCloud(bbox_file);
      // sensor_msgs::PointCloud2 pcl2_msg;
      // pcl::toROSMsg(obj_det_.labelCloudPoints, pcl2_msg);
      // pcl2_msg.header = rgb_msg->header;
      // pcl2_msg.header.stamp = odom_stamp.stamp_;
      // pcl2_msg.header.frame_id = local_frame_;
      // point_cloud2_raw_pub_.publish(pcl2_msg);

    }


    bool pub_bbox_img = true;
    if(pub_bbox_img)
    {
      Eigen::MatrixXd truth_data(2,6);
      if (!read_all_number_txt(bbox_file, truth_data))
        std::cout << "read file failed !!!!!!!!!!! " << bbox_file << std::endl;
      for (size_t i = 0; i < truth_data.rows(); i++)
      {
        if(truth_data(i,0)==2) //station, remove
          {
            truth_data(i,3) = 0;
            truth_data(i,4) = 0;
          }
      }
      cv::Mat plot_2d_img = obj_det_.imRGB.clone();
      Eigen::MatrixXd bboxes = truth_data.block(0,1,truth_data.rows(),4);
      plot_2d_bbox_with_xywh(plot_2d_img, bboxes);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", plot_2d_img).toImageMsg();
      bbox_image_pub_.publish(msg);
    }

  }


  void cameraInfoCallback(
      const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg,
      const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg) {
    if (camera_info_ready_) {
    // std::cout << "fx" << obj_det_.fx << std::endl;
    // std::cout << "fy" << obj_det_.fy << std::endl;
    // std::cout << "cx" << obj_det_.cx << std::endl;
    // std::cout << "cy" << obj_det_.cy << std::endl;

      return;
    }

    sensor_msgs::CameraInfo depth_info;
    depth_info = *depth_camera_info_msg;
    Eigen::Vector2d depth_image_size(depth_info.width, depth_info.height);

    cv::Mat K_depth = cv::Mat::eye(3, 3, CV_32FC1);
    K_depth.at<float>(0, 0) = depth_info.K[0];
    K_depth.at<float>(0, 2) = depth_info.K[2];
    K_depth.at<float>(1, 1) = depth_info.K[4];
    K_depth.at<float>(1, 2) = depth_info.K[5];
    K_depth.at<float>(2, 2) = depth_info.K[8];

    obj_det_.Image_size_depth = depth_image_size;
    obj_det_.Kalib_depth = K_depth;


    sensor_msgs::CameraInfo rgb_info;
    rgb_info = *rgb_camera_info_msg;
    Eigen::Vector2d rgb_image_size(rgb_info.width, rgb_info.height);

    cv::Mat K_rgb = cv::Mat::eye(3, 3, CV_32FC1);
    K_rgb.at<float>(0, 0) = rgb_info.K[0];
    K_rgb.at<float>(0, 2) = rgb_info.K[2];
    K_rgb.at<float>(1, 1) = rgb_info.K[4];
    K_rgb.at<float>(1, 2) = rgb_info.K[5];
    K_rgb.at<float>(2, 2) = rgb_info.K[8];

    obj_det_.Image_size_rgb = rgb_image_size;
    obj_det_.Kalib_rgb = K_rgb;
    obj_det_.fx = rgb_info.K[0];
    obj_det_.fy = rgb_info.K[4];
    obj_det_.cx = rgb_info.K[2];
    obj_det_.cy = rgb_info.K[5];
    std::cout << "fx" << obj_det_.fx << std::endl;
    std::cout << "fy" << obj_det_.fy << std::endl;
    std::cout << "cx" << obj_det_.cx << std::endl;
    std::cout << "cy" << obj_det_.cy << std::endl;

    camera_info_ready_ = true;
  }

  cv::Mat rotate(cv::Mat src, double angle)   //rotate function returning mat object with parametres imagefile and angle    
  {
      cv::Mat dst;      //Mat object for output image file
      cv::Point2f pt(src.cols/2., src.rows/2.);          //point from where to rotate    
      cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);      //Mat object for storing after rotation
      cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));  ///applie an affine transforation to image.
      return dst;         //returning Mat object for output image file
  }


};





int main( int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Starting object segmentation ... ";
    ros::init(argc, argv, "object_segmentation_node");
    ROS_INFO("object_segmentation_node");
    ObjectSegmentationNode object_segmentation_node;
    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}

