#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include "opencv2/opencv.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h> // include pcl::OrganizedMultiPlaneSegmentation
#include <pcl/segmentation/extract_clusters.h>  // include pcl::EuclideanClusterExtraction
#include <pcl/features/integral_image_normal.h>  // include pcl::IntegralImageNormalEstimation
#include <pcl/visualization/cloud_viewer.h>//include pcl::visualization::CloudViewer
#include <boost/make_shared.hpp>

#include <pcl/filters/statistical_outlier_removal.h> // include StatisticalOutlierRemoval
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h> // compute convex hull

#include <matrix_utils.h>
// #include <MapObject.h>

using namespace std;
typedef pcl::PointXYZRGBL PointT;
typedef pcl::PointCloud <PointT> PointCloud;

namespace object_segmentation {
const static std::string kRgbImageTopic = "/camera/rgb/image_raw";
const static std::string kRgbCameraInfoTopic = "/camera/rgb/camera_info";
const static std::string kDepthImageTopic =  "/camera/depth_registered/image_raw";
const static std::string kDepthCameraInfoTopic = "/camera/depth_registered/camera_info";

const static std::string kTfWorldFrame = "map";
const static std::string kTfLocalFrame = "base_link";

const static std::string kObjPCLTopic = "obj_seg";
const static std::string kRawPCLTopic = "raw_pcl";
const static std::string kConfigFile = "sss";
const static std::string kDatasetPath = "sss";
const static bool kPubLocalFrame = false;

class ObjectDetector
{
private:
    /* data */
public:
	float fx, fy, cx, cy;  // Camera intrinsic parameters.
	cv::Mat imRGB, imDepth;
	int imWidth, imHeight;	// image size
	Eigen::Matrix3d Kalib;
    Eigen::Vector2d Image_size_rgb;
    Eigen::Vector2d Image_size_depth;
	Eigen::Matrix4d Tbo; // odom to baselink
	Eigen::Matrix4d Tcb; // baselink to camera
	Eigen::Matrix4d Twc; // camera to odom
    
    cv::Mat Kalib_rgb;
    cv::Mat Kalib_depth;

    // global object detector param 
    int MultiPlane_SizeMin; // param for MultiPlaneSegmentation
    float MultiPlane_AngleThre;
    float MultiPlane_DistThre;

    float ground_min;
    float ground_max;
    float wall_min;
    float wall_max;

	pcl::PointCloud<pcl::PointXYZRGB> rawCloudPoints; // convert from depth image
	pcl::PointCloud<pcl::PointXYZRGB> bboxCloudPoints; // convert from depth image
	std::vector< pcl::PointCloud<pcl::PointXYZRGB>> objectCloudPoints; 
	pcl::PointCloud<pcl::PointXYZRGBL> labelCloudPoints; // convert from depth image
	std::vector< pcl::PointCloud<pcl::PointXYZRGBL>> objectLabelCloudPoints; 

public:
    ObjectDetector(/* args */);
    ~ObjectDetector();
    bool ComputePointCloud();
    bool ReadBBox2DCloud(std::string& filename);
    
	bool ClearData();
    bool ReadBBox2DLabelCloud(std::string& filename);
    bool ComputeLabelCloud(std::string& filename);
    int checkBbox(const int& x, const int& y, const Eigen::MatrixXd& bboxes);

    bool ExtractObjectFromOrganizedPlanes(pcl::PointCloud<pcl::PointXYZRGBL>& bbox_pcl,
													pcl::PointCloud<pcl::PointXYZRGBL>& obj_pcl);

};

ObjectDetector::ObjectDetector(/* args */)
{}

ObjectDetector::~ObjectDetector()
{
    imRGB.release();
    imDepth.release();
}



}  // namespace object_segmentation




#endif