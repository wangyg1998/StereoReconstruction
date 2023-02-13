#pragma once
#include <stdio.h>
#include <sys/stat.h>

#include <iostream>
#include <opencv2/opencv.hpp>

namespace sr
{
/// \brief 相机内参标定
void camera_intrinsic(std::vector<std::shared_ptr<cv::Mat>> images, int corners_per_row, int corners_per_col, float square_size, std::string file_name);

/// \brief 双目立体标定，求左相机到右相机的R,T
void calib_extrinisic(std::string left_intrinsic_file,
                      std::string right_intrinsic_file,
                      std::string calib_file,
                      std::vector<std::shared_ptr<cv::Mat>> left_images,
                      std::vector<std::shared_ptr<cv::Mat>> right_images);

/// \brief 双目图像矫正
void image_rectify(std::string calib_file, const cv::Mat* left_image, const cv::Mat* right_image, cv::Mat* rectified_left, cv::Mat* rectified_right);

/// \brief 求相机和投影仪的内参，并进行立体标定(相机到投影仪的R,T)
/// \ref Simple, Accurate, and Robust Projector-Camera Calibration
bool projector_Camera_Calibration(std::vector<std::vector<std::shared_ptr<cv::Mat>>> images_list,
                                  int corners_per_row = 11,
                                  int corners_per_col = 7,
                                  float square_edge_length = 21.f,
                                  int projector_width = 1920,
                                  int projector_height = 1080,
                                  int shadow_threshold = 25,
                                  int homography_window_size = 60,
                                  float decode_param_b = 0.5,
                                  int decode_param_m = 5);

/// \brief 重投影误差
double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                 const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                 const std::vector<cv::Mat>& rvecs,
                                 const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs);

} // namespace sr
