#include "ObjectDetector.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <boost/make_shared.hpp>


#include <cmath>

using namespace std;


namespace object_segmentation {

bool ObjectDetector::ComputePointCloud()
{
	std::cout << "\n----- ComputePointCloud -----" << std::endl;
	std::cout << "imDepth.rows "<< imDepth.rows << std::endl;
	std::cout << "imDepth.cols "<< imDepth.cols << std::endl;
	// translate to point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
	int cloudDis = 3; // use 1 to exact more planes
	for ( int m=0; m<imDepth.rows; m+=cloudDis )
	{
		for ( int n=0; n<imDepth.cols; n+=cloudDis )
		{
			float d = imDepth.ptr<float>(m)[n];
			pcl::PointXYZRGB p;
			p.z = d;
			p.x = ( n - cx) * p.z / fx;
			p.y = ( m - cy) * p.z / fy;
			// p.r = 177;
			// p.g = 177;
			// p.b = 177;
            p.b = imRGB.ptr<uchar>(m)[n*3];
            p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
            p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
			inputCloud->points.push_back(p);
		}
	}
	inputCloud->height = ceil(imDepth.rows/float(cloudDis));
	inputCloud->width = ceil(imDepth.cols/float(cloudDis));
	std::cout << "inputCloud.size "<< inputCloud->size() << std::endl;
	std::cout << "Twc "<< Twc << std::endl;
    

    // rawCloudPoints = *inputCloud;
    // // check if we need to transform point cloud to world coordinates
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_world(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*inputCloud, *cloud_in_world, Twc);
    rawCloudPoints = *cloud_in_world;

    return true;
}

bool ObjectDetector::ReadBBox2DCloud(std::string& filename)
{
	std::cout << "start reading point cloud in 2d bounding box "  << std::endl;
	Eigen::MatrixXd truth_data(2,6);
	if (!read_all_number_txt(filename, truth_data))
		return -1;
	if(truth_data.rows()==0) // no object
		return false;
	
	std::cout << "truth_data " << truth_data << std::endl;
	// filter bbox?
	// filterBBox2D(truth_data);

	pcl::PointCloud<pcl::PointXYZRGB> bbox_raw_points; 
	for (size_t i = 0; i < truth_data.rows(); i++)
    {
		if(truth_data(i,0)!= 1)
			continue;
		if(truth_data(i,5) < 0.5)
			continue;
        Eigen::Vector4d obj_bbox = truth_data.row(i).segment<4>(1);
        int x_min = int(obj_bbox(0));
        int y_min = int(obj_bbox(1));
        int x_max = min(int(obj_bbox(0)+obj_bbox(2)), imDepth.cols);
        int y_max = min(int(obj_bbox(1)+obj_bbox(3)), imDepth.rows);
		pcl::PointCloud<pcl::PointXYZRGB> obj_points_cam;
		int cloudDis = 1; // use 1 to exact more planes
		for ( int m=y_min; m<y_max; m+=cloudDis )
		{
			for ( int n=x_min; n<x_max; n+=cloudDis )
			{
				float d = imDepth.ptr<float>(m)[n];
				// if(d==0)
				// {
				// 	// std::cout << "bad point" << std::endl;
				// 	continue;
				// }
				pcl::PointXYZRGB p;
				p.z = d;
				p.x = ( n - cx) * p.z / fx;
				p.y = ( m - cy) * p.z / fy;
				// p.r = 177;
				// p.g = 177;
				// p.b = 177;
				p.b = imRGB.ptr<uchar>(m)[n*3];
				p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
				p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
				obj_points_cam.points.push_back(p);
			}
		}
        // objectCloudPoints.push_back(obj_points_cam);
        bbox_raw_points += obj_points_cam;
    }

    // // check if we need to transform point cloud to world coordinates
    pcl::transformPointCloud(bbox_raw_points, bboxCloudPoints, Twc);
    return true;
}

bool ObjectDetector::ComputeLabelCloud(std::string& filename)
{
	std::cout << "start reading point cloud in 2d bounding box "  << std::endl;
	Eigen::MatrixXd truth_data(2,6);
	if (!read_all_number_txt(filename, truth_data))
		return -1;
	if(truth_data.rows()==0) // no object
		return false;

	std::cout << "truth_data " << truth_data << std::endl;

	// // filter bbox?
	// filterBBox2D(truth_data);

	pcl::PointCloud<pcl::PointXYZRGBL> inputCloud;
	int cloudDis = 3; // use 1 to exact more planes
	for ( int m=0; m<imDepth.rows; m+=cloudDis )
	{
		for ( int n=0; n<imDepth.cols; n+=cloudDis )
		{
			int obj_label = checkBbox(n, m, truth_data);
			if(obj_label!=0)
				continue;
			float d = imDepth.ptr<float>(m)[n];
			pcl::PointXYZRGBL p;
			p.z = d;
			p.x = ( n - cx) * p.z / fx;
			p.y = ( m - cy) * p.z / fy;
            // p.b = imRGB.ptr<uchar>(m)[n*3];
            // p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
            // p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
			p.b = 177;
			p.g = 0;
			p.r = 0;
			p.label = 0;
			inputCloud.points.push_back(p);
		}
	}
		
    for (size_t i = 0; i < truth_data.rows(); i++)
    {
		if(truth_data(i,0)!= 1)
			continue;
		if(truth_data(i,5) < 0.5)
			continue;
        Eigen::Vector4d obj_bbox = truth_data.row(i).segment<4>(1);
        int x_min = int(obj_bbox(0));
        int y_min = int(obj_bbox(1));
        int x_max = min(int(obj_bbox(0)+obj_bbox(2)), imDepth.cols);
        int y_max = min(int(obj_bbox(1)+obj_bbox(3)), imDepth.rows);
		pcl::PointCloud<pcl::PointXYZRGBL> obj_points_cam;
		int cloudDis = 3; // use 1 to exact more planes
		for ( int m=y_min; m<y_max; m+=cloudDis )
		{
			for ( int n=x_min; n<x_max; n+=cloudDis )
			{
				float d = imDepth.ptr<float>(m)[n];
				pcl::PointXYZRGBL p;
				p.z = d;
				p.x = ( n - cx) * p.z / fx;
				p.y = ( m - cy) * p.z / fy;
				p.b = imRGB.ptr<uchar>(m)[n*3];
				p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
				p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
				p.label = truth_data(i,0);
				obj_points_cam.points.push_back(p);
			}
		}
		obj_points_cam.height = ceil((y_max-y_min)/float(cloudDis));
		obj_points_cam.width = ceil((x_max-x_min)/float(cloudDis));

		// // estimate plane before transfrom
		// // filter outlier by planes (from exprience, it works very good)
		// // reason, from camera capture principle, points of outlier are further than objects
		// // 由于相机采集数据原理，平台相比于物体散射严重，点云质量不行，无法提取平面，相反通过提取平面反而能提出所有物体点云
		// pcl::PointCloud<pcl::PointXYZRGBL> obj_points_new;
		// bool obj_exact = ExtractObjectFromOrganizedPlanes(obj_points_cam, obj_points_new);
		// if(obj_exact == false) // no object point are detected
		// 	continue;
		// std::cout << "obj points raw: " << obj_points_cam.points.size() << " after filter: " << obj_points_new.points.size() << std::endl;

		for (size_t kk = 0; kk < obj_points_cam.points.size(); kk++)
		{
			obj_points_cam.points[kk].label = 5;
			obj_points_cam.points[kk].b = 0;
			obj_points_cam.points[kk].g = 0;
			obj_points_cam.points[kk].r = 177;
		}
		
		inputCloud = inputCloud + obj_points_cam;
		
    }

    // // check if we need to transform point cloud to world coordinates
	pcl::PointCloud<pcl::PointXYZRGBL>  raw_points_world;
	pcl::transformPointCloud(inputCloud, raw_points_world, Twc);
	labelCloudPoints = raw_points_world;
	return true;
}

int ObjectDetector::checkBbox(const int& x, const int& y, const Eigen::MatrixXd& bboxes)
{
	int label = 0;
	for (size_t i = 0; i < bboxes.rows(); i++)
	{
		if(bboxes(i,0)!= 1)
			continue;
		if(bboxes(i,5) < 0.5)
			continue;
		if(x>bboxes(i,1) && x<bboxes(i,1)+bboxes(i,3) &&
			y>bboxes(i,2) && y<bboxes(i,2)+bboxes(i,4)	)
		{
			label = bboxes(i,0);
		}
	}
	return label;
}

bool ObjectDetector::ReadBBox2DLabelCloud(std::string& filename)
{
	std::cout << "start reading point cloud in 2d bounding box "  << std::endl;
	Eigen::MatrixXd truth_data(2,6);
	if (!read_all_number_txt(filename, truth_data))
		return -1;
	if(truth_data.rows()==0) // no object
		return false;

	std::cout << "truth_data " << truth_data << std::endl;
	// filter bbox?
	filterBBox2D(truth_data);
	std::cout << "truth_data " << truth_data << std::endl;

	std::vector< pcl::PointCloud<pcl::PointXYZRGBL>> tempPCL; 
	pcl::PointCloud<pcl::PointXYZRGBL> inputCloud;
	int cloudDis = 10; // use 1 to exact more planes
	for ( int m=0; m<imDepth.rows; m+=cloudDis )
	{
		for ( int n=0; n<imDepth.cols; n+=cloudDis )
		{
			int obj_label = checkBbox(n, m, truth_data);
			if(obj_label!=0)
				continue;
			float d = imDepth.ptr<float>(m)[n];
			pcl::PointXYZRGBL p;
			p.z = d;
			p.x = ( n - cx) * p.z / fx;
			p.y = ( m - cy) * p.z / fy;
            // p.b = imRGB.ptr<uchar>(m)[n*3];
            // p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
            // p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
			p.b = 177;
			p.g = 0;
			p.r = 0;
			p.label = 20;
			inputCloud.points.push_back(p);
		}
	}
    pcl::transformPointCloud(inputCloud, inputCloud, Twc);
	tempPCL.push_back(inputCloud);

    for (size_t i = 0; i < truth_data.rows(); i++)
    {
		if(truth_data(i,0)!= 1)
			continue;
		if(truth_data(i,5) < 0.5)
			continue;
        Eigen::Vector4d obj_bbox = truth_data.row(i).segment<4>(1);
        int x_min = int(obj_bbox(0));
        int y_min = int(obj_bbox(1));
        int x_max = min(int(obj_bbox(0)+obj_bbox(2)), imDepth.cols);
        int y_max = min(int(obj_bbox(1)+obj_bbox(3)), imDepth.rows);
		pcl::PointCloud<pcl::PointXYZRGBL> obj_points_cam;
		int cloudDis = 3; // use 1 to exact more planes
		for ( int m=y_min; m<y_max; m+=cloudDis )
		{
			for ( int n=x_min; n<x_max; n+=cloudDis )
			{
				float d = imDepth.ptr<float>(m)[n];
				pcl::PointXYZRGBL p;
				p.z = d;
				p.x = ( n - cx) * p.z / fx;
				p.y = ( m - cy) * p.z / fy;
				// p.b = imRGB.ptr<uchar>(m)[n*3];
				// p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
				// p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
				p.b = 0;
				p.g = 0;
				p.r = 177;
				p.label = truth_data(i,0);
				obj_points_cam.points.push_back(p);
			}
		}
		obj_points_cam.height = ceil((y_max-y_min)/float(cloudDis));
		obj_points_cam.width = ceil((x_max-x_min)/float(cloudDis));

		// estimate plane before transfrom
		// filter outlier by planes (from exprience, it works very good)
		// reason, from camera capture principle, points of outlier are further than objects
		// 由于相机采集数据原理，平台相比于物体散射严重，点云质量不行，无法提取平面，相反通过提取平面反而能提出所有物体点云
		pcl::PointCloud<pcl::PointXYZRGBL> obj_points_new;
		bool obj_exact = ExtractObjectFromOrganizedPlanes(obj_points_cam, obj_points_new);
		if(obj_exact == false) // no object point are detected
			continue;
		std::cout << "obj points raw: " << obj_points_cam.points.size() << " after filter: " << obj_points_new.points.size() << std::endl;


		// // check if we need to transform point cloud to world coordinates
		pcl::PointCloud<pcl::PointXYZRGBL>  raw_points_world;
		pcl::transformPointCloud(obj_points_new, raw_points_world, Twc);
		tempPCL.push_back(raw_points_world);
		
    }
	objectLabelCloudPoints = tempPCL;
	std::cout << "objectLabelCloudPoints " << objectLabelCloudPoints.size() << std::endl;

	return true;
}

bool ObjectDetector::ExtractObjectFromOrganizedPlanes(pcl::PointCloud<pcl::PointXYZRGBL>& bbox_pcl,
													pcl::PointCloud<pcl::PointXYZRGBL>& obj_pcl)
{

	int min_plane = 100;//MultiPlane_SizeMin;//100;
	float AngTh = 2.0;//MultiPlane_AngleThre;//2.0;
	float DisTh = 0.02;//MultiPlane_DistThre;//0.02;

	// // firstly, compute normal
	pcl::PointCloud<PointT>::Ptr  input_cloud = bbox_pcl.makeShared();
	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.05f); // 0.05
	ne.setNormalSmoothingSize(10.0f); // 10.0
	ne.setInputCloud(input_cloud);
	ne.compute(*cloud_normals);

	// secondly, compute region, label, coefficient, inliners, ...
	pcl::OrganizedMultiPlaneSegmentation< PointT, pcl::Normal, pcl::Label > mps;
	pcl::PointCloud<pcl::Label>::Ptr labels ( new pcl::PointCloud<pcl::Label> );
	vector<pcl::ModelCoefficients> coefficients;
	vector<pcl::PointIndices> inliers;
	vector<pcl::PointIndices> label_indices;
	vector<pcl::PointIndices> boundary;
	std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
	mps.setMinInliers(min_plane);	//int min_plane = 1000;
	mps.setAngularThreshold (0.017453 * AngTh); //float AngleThreshold = 3.0 (0.017453=pi/180)
	mps.setDistanceThreshold (DisTh); // float DistanceThreshold = 0.05
	mps.setInputNormals (cloud_normals);
	mps.setInputCloud (input_cloud);
	mps.segmentAndRefine (regions, coefficients, inliers, labels, label_indices, boundary);
	std::cout << "inliers: " << inliers.size() << std::endl;
	if (inliers.size()==0)
	{
		std::cout << "no object point cloud detected" << std::endl;
		return false;
	}

	// // thirdly, exact and filter point cloud
	// pcl::PointCloud <pcl::PointXYZRGBL> obj_pcl; 
	std::vector<cv::Mat> mvPlaneCoefficients; // plane normal 
	std::vector<pcl::PointIndices > mvPlaneIndices; // plane points 
	std::vector<pcl::PointCloud <pcl::PointXYZRGBL> > mvPlanePoints; // plane points 
    std::vector<pcl::PointCloud <pcl::PointXYZRGBL> > mvBoundaryPoints; // plane boundary, just for visualization
	for (int i = 0; i < inliers.size(); ++i)
	{
		cv::Mat coef = (cv::Mat_<float>(4,1) << coefficients[i].values[0], 
						coefficients[i].values[1], 
						coefficients[i].values[2], 
						coefficients[i].values[3]);
		if(coef.at<float>(3) < 0)
				coef = -coef;
		std::cout << "plane: " << i <<" "<< coef.t() << std::endl;

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(input_cloud);
		extract.setNegative(false);
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
		extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers[i])); // #include <boost/make_shared.hpp>
		extract.filter(*planeCloud);
		// std::cout << "plane: " << i <<" "<< coef.t() << planeCloud->size() << std::endl;

		// save for visualization
		pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices(inliers[i]));
		PointCloud::Ptr boundaryPoints(new PointCloud());
		boundaryPoints->points = regions[i].getContour();
		mvPlanePoints.push_back(*planeCloud);
		mvPlaneCoefficients.push_back(coef);
		mvBoundaryPoints.push_back(*boundaryPoints);
		mvPlaneIndices.push_back(*planeIndices);
		
		// just collect all plane points;
		obj_pcl += *planeCloud;
	}

    return true;
}

bool ObjectDetector::ClearData()
{
	std::cout << "----- frame dataset reset -----" << std::endl;
	rawCloudPoints.clear();
	bboxCloudPoints.clear();
	objectCloudPoints.clear();
	labelCloudPoints.clear();
	objectLabelCloudPoints.clear();
    return true;
}

}