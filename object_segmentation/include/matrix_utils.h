#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace Eigen;

// make sure column size is given. no checks here. row will be adjusted automatically. if more cols given, will be zero.
bool read_all_number_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat)
{
    if (!std::ifstream(txt_file_name))
    {
        std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
        return false;
    }
    std::ifstream filetxt(txt_file_name.c_str());
    int row_counter = 0;
    std::string line;
    if (read_number_mat.rows() == 0)
        read_number_mat.resize(100, 10);

    while (getline(filetxt, line))
    {
        double t;
        if (!line.empty())
        {
            std::stringstream ss(line);
            int colu = 0;
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
            }
            row_counter++;
            if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
        }
    }
    filetxt.close();
    read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows
    return true;
}

bool read_obj_detection_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &all_strings)
{
    if (!std::ifstream(txt_file_name))
    {
        std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
        return false;
    }
    all_strings.clear();
    std::ifstream filetxt(txt_file_name.c_str());
    if (read_number_mat.rows() == 0)
        read_number_mat.resize(100, 10);
    int row_counter = 0;
    std::string line;
    while (getline(filetxt, line))
    {
        double t;
        if (!line.empty())
        {
            std::stringstream ss(line);
            std::string classname;
            ss >> classname;
            all_strings.push_back(classname);
            int colu = 0;
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
            }
            row_counter++;
            if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
        }
    }
    filetxt.close();
    read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows
    return true;
}

Eigen::Matrix3d euler_zyx_to_rot(const double &roll, const double &pitch, const double &yaw)
{
    double cp = cos(pitch);
    double sp = sin(pitch);
    double sr = sin(roll);
    double cr = cos(roll);
    double sy = sin(yaw);
    double cy = cos(yaw);

    Eigen::Matrix<double, 3, 3> R;
    R << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    return R;
}

void quat_to_euler_zyx(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    pitch = asin(2 * (qw * qy - qz * qx));
    yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

Eigen::MatrixXd project_camera_points_to_2d(Eigen::MatrixXd& points3d_3x8, Matrix3d& Kalib)
{
  Eigen::MatrixXd corners_2d(3,8);
  corners_2d = Kalib *  points3d_3x8;
  for (size_t i = 0; i < corners_2d.cols(); i++)
  {
      corners_2d(0,i) = corners_2d(0,i) /corners_2d(2,i);
      corners_2d(1,i) = corners_2d(1,i) /corners_2d(2,i);
      corners_2d(2,i) = corners_2d(2,i) /corners_2d(2,i);
  }
  Eigen::MatrixXd corners_2d_return(2,8);
  corners_2d_return = corners_2d.topRows(2);
  return corners_2d_return;
}

Eigen::MatrixXd compute3D_BoxCorner_in_camera(Eigen::Vector3d& location, Eigen::Vector3d& dimension,  Eigen::Matrix3d& local_rot_mat)
{
    Eigen::MatrixXd corners_body(3, 8);
    corners_body << 1, 1, -1, -1, 1, 1, -1, -1,
                    1, -1, -1, 1, 1, -1, -1, 1,
                    1, 1, 1, 1, -1, -1, -1, -1;
    Eigen::Matrix3d scale_mat = dimension.asDiagonal();
    Eigen::Matrix3d rot;
    rot = local_rot_mat;
    // rot << cos(ry), -sin(ry), 0,
    //     sin(ry), cos(ry), 0,
    //     0, 0, 1;                          // rotation around z (up), world coordinate
    // rot << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry); // rotation around y (up), camera coordinate

    Eigen::MatrixXd corners_without_center = rot * scale_mat * corners_body;
    // std::cout << "dimension\n" << scale_mat * corners_body << std::endl;
    // std::cout << "rot\n" << rot << std::endl;
    // std::cout << "corners_without_center\n" << corners_without_center << std::endl;

    Eigen::MatrixXd corners_3d(3, 8);
    for (size_t i = 0; i < 8; i++)
    {
      corners_3d(0,i) = corners_without_center(0,i) + location(0);
      corners_3d(1,i) = corners_without_center(1,i) + location(1);
      corners_3d(2,i) = corners_without_center(2,i) + location(2);
    }
    return corners_3d;
}

void plot_3d_box_with_loc_dim_camera(cv::Mat &img, Eigen::Matrix3d& Kalib, Eigen::Vector3d& location, Eigen::Vector3d& dimension, Eigen::Matrix3d& local_rot_mat)
{
  MatrixXd corner_3d = compute3D_BoxCorner_in_camera(dimension, location, local_rot_mat);
  // std::cout << "corner_3d: \n" << corner_3d << std::endl;
  MatrixXd corner_img = project_camera_points_to_2d(corner_3d, Kalib);
  // std::cout << "corner_img: \n" << corner_img << std::endl;
  Eigen::MatrixXd edge_pt_ids(2,14); // for 12 body edge (1-2, 2-3, ...) and 2 front line
  edge_pt_ids << 1,2,3,4, 1,2,6,5, 1,4,8,5, 1,2,
                 5,6,7,8, 4,3,7,8, 2,3,7,6, 6,5;
  edge_pt_ids.array()-=1; // transfer 1-8 to 0-7
  for (int pt_id = 0; pt_id < 12; pt_id++) // 12 body edge
  {
    int i = edge_pt_ids(0, pt_id);
    int j = edge_pt_ids(1, pt_id);
    cv::Point pt1 = cv::Point(corner_img(0,i), corner_img(1,i));
    cv::Point pt2 = cv::Point(corner_img(0,j), corner_img(1,j));
    cv::line(img, pt1, pt2, cv::Scalar(0, 225, 0), 1, CV_AA, 0);
  }
  for (int pt_id = 0; pt_id < 2; pt_id++) // 2 front edge
  {
    int i = edge_pt_ids(0, 12+pt_id);
    int j = edge_pt_ids(1, 12+pt_id);
    cv::Point pt1 = cv::Point(corner_img(0,i), corner_img(1,i));
    cv::Point pt2 = cv::Point(corner_img(0,j), corner_img(1,j));
    cv::line(img, pt1, pt2, cv::Scalar(255, 0, 0), 1, CV_AA, 0);
  }

  for (int i = 0; i < 8; i++) // plot 8 corners
  {
    cv::Point pt = cv::Point(corner_img(0,i), corner_img(1,i));
    cv::circle(img, pt, i, CV_RGB(255, 0, 0), 2);
  }

}

void plot_2d_bbox_with_xywh(cv::Mat&output_img, Eigen::MatrixXd& raw_2d_objs)
{
  // cv::Mat output_img = rgb_img.clone();
  for (size_t box_id = 0; box_id < raw_2d_objs.rows(); box_id++)
  {
    cv::Point pt1 = cv::Point(raw_2d_objs(box_id,0),                      raw_2d_objs(box_id,1));
    cv::Point pt2 = cv::Point(raw_2d_objs(box_id,0),                      raw_2d_objs(box_id,1)+raw_2d_objs(box_id,3));
    cv::Point pt3 = cv::Point(raw_2d_objs(box_id,0)+raw_2d_objs(box_id,2), raw_2d_objs(box_id,1));
    cv::Point pt4 = cv::Point(raw_2d_objs(box_id,0)+raw_2d_objs(box_id,2), raw_2d_objs(box_id,1)+raw_2d_objs(box_id,3));
    cv::line(output_img, pt1, pt2, cv::Scalar(0, 225, 0), 3, CV_AA, 0);
    cv::line(output_img, pt1, pt3, cv::Scalar(0, 225, 0), 3, CV_AA, 0);
    cv::line(output_img, pt4, pt2, cv::Scalar(0, 225, 0), 3, CV_AA, 0);
    cv::line(output_img, pt4, pt3, cv::Scalar(0, 225, 0), 3, CV_AA, 0);
  }
}

void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

float bboxOverlapRatio(Eigen::Vector4d &bbox_vec_1, const Eigen::Vector4d &bbox_vec_2, int flag)
{
    // float bboxOverlapRatio(const cv::Rect &rect1, const cv::Rect &rect2, std::string &flag)
    // using opencv with cv::Rect
    // int overlap_area = (rect1 & rect2).area();
    // return (float)overlap_area / ((float)(rect1.area() + rect2.area() - overlap_area));

    // // using eigen: bbox_vec_1: centerx, centery, width, height
    // const Eigen::Vector4d &bbox_vec_1, const Eigen::Vector4d &bbox_vec_2
    float x_left = std::max(bbox_vec_1(0), bbox_vec_2(0));
    float x_right = std::min(bbox_vec_1(0)+bbox_vec_1(2), bbox_vec_2(0)+bbox_vec_2(2));
    float y_top = std::max(bbox_vec_1(1), bbox_vec_2(1));
    float y_bottom = std::min(bbox_vec_1(1)+bbox_vec_1(3), bbox_vec_2(1)+bbox_vec_2(3));
    float intersection_area = (x_right - x_left) * (y_bottom - y_top);
    float bb1_area = bbox_vec_1(2)*bbox_vec_1(3);
    float bb2_area = bbox_vec_2(2)*bbox_vec_2(3);
    float iou_2d = intersection_area / (bb1_area + bb2_area - intersection_area);
    if(flag == 1)
        iou_2d = intersection_area / (bb1_area);
    else if(flag == 2)
        iou_2d = intersection_area / (bb2_area);
    return iou_2d;

}

void filterBBox2D(Eigen::MatrixXd& truth_data)
{
	// filter bbox that has small confidence
	for (size_t i = truth_data.rows()-1; i > 0; i--)
		if(truth_data(i,5)<0.5)
			removeRow(truth_data, i);

	// for (size_t i = 0; i < truth_data.rows(); i++)
	// 	for (size_t j = i+1; j < truth_data.rows(); j++)

	for (int i = truth_data.rows()-1; i>=0 ; i--)
		for (int j = i-1; j>=0; j--)
	{
		// std::cout << "i: " << i << " j: " << j  << std::endl;
		// std::cout << "truth_data:\n " << truth_data << std::endl;
		// compare same class
		if(truth_data(i,0) != truth_data(j,0))
			continue;
		// not exactly iou, but compare with each bbox
        Eigen::Vector4d bbox_1 = truth_data.row(i).segment<4>(1);
        Eigen::Vector4d bbox_2 = truth_data.row(j).segment<4>(1);
		// float iou_0 = bboxOverlapRatio(bbox_1, bbox_2, "0");
		float iou_1 = bboxOverlapRatio(bbox_1, bbox_2, 1);
		float iou_2 = bboxOverlapRatio(bbox_1, bbox_2, 2);
		// std::cout << "i: " << i << " j: " << j  << std::endl;
		// std::cout << "iou_1: " << iou_1 << " iou_2: " << iou_2  << std::endl;
		if(iou_1>0.5 || iou_2 > 0.5)
		{
			// remove object that has smaller confidence than 0.9
			if(truth_data(i,5)>truth_data(j,5) && truth_data(j,5) <0.9)
				removeRow(truth_data, j);
			else if (truth_data(i,5)<truth_data(j,5) && truth_data(i,5) <0.9)
				removeRow(truth_data, i);
		}
	}
}

void LoadImagesIndex(const string &strAssociationFilename, vector<string> &vRGBIndex,
                    vector<string> &vDepthIndex, vector<Eigen::VectorXd>& vOdomIndex)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string id;
            string rgb;
            string depth;
            ss >> id;
            vRGBIndex.push_back(id);
            ss >> rgb;
            ss >> id;
            ss >> depth;
            vDepthIndex.push_back(id);
            Eigen::VectorXd odom(7);
            ss >> id;
            for (size_t i = 0; i < 7; i++)
                ss >> odom(i);
            vOdomIndex.push_back(odom);
        }
    }
}


#endif