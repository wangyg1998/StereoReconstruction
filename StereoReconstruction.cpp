#include <Windows.h>
#include <conio.h>
#include <io.h>
#include <process.h>
#include <stdio.h>
#include <trimesh.h>
#include <trimesh_algo.h>
#include <windows.h>

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "Calibration/Calibration.h"
#include "Calibration/Encoder.h"
#include "Calibration/structured_light.hpp"
#include "MvCamera/MvCamera.h"
#include "Reconstruction/Reconstruction.h"
#include "Calibration/CalibrationData.hpp"

static void getAllFiles(std::string path, std::vector<std::string>& files, std::string fileType)
{
	// 文件句柄
	intptr_t hFile = 0;
	// 文件信息
	struct _finddata_t fileinfo;

	std::string p;

	if ((hFile = _findfirst(p.assign(path).append("\\*" + fileType).c_str(), &fileinfo)) != -1)
	{
		do
		{
			// 保存文件的全路径
			files.push_back(p.assign(path).append("\\").append(fileinfo.name));

		} while (_findnext(hFile, &fileinfo) == 0); //寻找下一个，成功返回0，否则-1

		_findclose(hFile);
	}
}

int main()
{
	clock_t main_time = clock();

	std::vector<std::string> imgsName(100);
	for (int i = 0; i < imgsName.size(); ++i)
	{
		imgsName[i] = std::to_string(i);
		if (imgsName[i].size() < 2)
		{
			imgsName[i] = "0" + imgsName[i];
		}
	}
	imgsName.erase(imgsName.begin());

//生成用于标定的格雷码图像
#if 0
	std::vector<std::shared_ptr<cv::Mat>> colStripesImgs, rowStripesImgs;
	sr::Encoder_Gray(colStripesImgs, 11, true);
	sr::Encoder_Gray(rowStripesImgs, 11, false);
	colStripesImgs.insert(colStripesImgs.end(), rowStripesImgs.begin(), rowStripesImgs.end());
	for (int i = 0; i < colStripesImgs.size(); ++i)
	{
		cv::Mat img = *colStripesImgs[i].get();

		//cv::Rect rect((img.cols - 1920) / 2, (img.rows - 1080) / 2, 1920, 1080);
		cv::Rect rect(0, 0, 1920, 1080);
		cv::Mat cropped = img(rect);
		cv::imwrite("D://GrayCode_Calibration//" + imgsName[2 + i * 2] + ".bmp", cropped);
		img = *colStripesImgs[i].get();
		//cv::rotate(img, img, cv::ROTATE_180);
		//获取补码
		if (i < colStripesImgs.size() / 2)
		{
			for (int j = 0; j < img.cols; ++j)
			{
				if (img.at<char>(0, j) == 0)
				{
					img.at<char>(0, j) = 255;
				}
				else
				{
					img.at<char>(0, j) = 0;
				}
			}
			for (int j = 1; j < img.rows; ++j)
			{
				img.row(0).copyTo(img.row(j));
			}
		}
		else
		{
			for (int j = 0; j < img.rows; ++j)
			{
				if (img.at<char>(j, 0) == 0)
				{
					img.at<char>(j, 0) = 255;
				}
				else
				{
					img.at<char>(j, 0) = 0;
				}
			}
			for (int j = 1; j < img.cols; ++j)
			{
				img.col(0).copyTo(img.col(j));
			}
		}

		cropped = img(rect);
		cv::imwrite("D://GrayCode_Calibration//" + imgsName[2 + i * 2 + 1] + ".bmp", cropped);
	}

	cv::Mat img(1080, 1920, CV_8UC1);
	img.setTo(255);
	cv::imwrite("D://GrayCode_Calibration//01.bmp", img);
	img.setTo(50);
	cv::imwrite("D://GrayCode_Calibration//02.bmp", img);

	//生成格雷码图像
#elif 0
	std::vector<std::shared_ptr<cv::Mat>> grayCodeImgs_col, grayCodeImgs_row;
	sr::Encoder_GrayCodeImg(grayCodeImgs_col, 1920, 1080, 1, 11, true);
	sr::Encoder_GrayCodeImg(grayCodeImgs_row, 1920, 1080, 1, 11, false);
	grayCodeImgs_col.insert(grayCodeImgs_col.end(), grayCodeImgs_row.begin(), grayCodeImgs_row.end());
	for (int i = 0; i < grayCodeImgs_col.size(); ++i)
	{
		cv::imwrite("D:\\projector_pattern\\" + imgsName[i] + ".bmp", *grayCodeImgs_col[i].get());
	}

	//生成格雷码 + 相移 图像
#elif 0
	std::vector<std::shared_ptr<cv::Mat>> grayCodeImgs;
	sr::Encoder_GrayCodeImg(grayCodeImgs, 1920, 1080, 8, 8, true);
	std::vector<std::shared_ptr<cv::Mat>> PhaseImgs;
	sr::Encoder_PhaseImg(PhaseImgs, 1920, 1080, 8, true);
	grayCodeImgs.insert(grayCodeImgs.end(), PhaseImgs.begin(), PhaseImgs.end());
	for (int i = 0; i < grayCodeImgs.size(); ++i)
	{
		cv::imwrite("D:\\projector_pattern\\" + imgsName[i] + ".bmp", *grayCodeImgs[i].get());
	}

//采集标定数据
#elif 0
	std::vector<std::string> files;
	getAllFiles("D:\\GrayCode_Calibration", files, "bmp");
	std::vector<std::shared_ptr<cv::Mat>> imgs(files.size());
	for (int i = 0; i < files.size(); ++i)
	{
		imgs[i].reset(new cv::Mat);
		*imgs[i].get() = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		std::cout << files[i] << std::endl;
	}

	//查找并启动相机
	int nRet = MV_OK;
	CMvCamera camera;
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	//查找并启动相机
	camera.EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
	camera.Open(stDeviceList.pDeviceInfo[0]);
	camera.SetEnumValue("TriggerMode", 0);
	//camera.SetEnumValue("PixelFormat", PixelType_Gvsp_RGB8_Packed);
	camera.SetEnumValue("PixelFormat", PixelType_Gvsp_Mono8);

	MV_CC_SetExposureAutoMode(camera.getDeviceHandle(), 2);
	MV_CC_SetAutoExposureTimeLower(camera.getDeviceHandle(), 25000);
	MV_CC_SetAutoExposureTimeUpper(camera.getDeviceHandle(), 500000);

	//MV_CC_SetExposureTime(camera.getDeviceHandle(), 50000);
	if (camera.StartGrabbing() != MV_OK)
	{
		std::cout << "StartGrabbing error" << std::endl;
		return 0;
	}
	Sleep(5000); //等待相机启动

	MV_FRAME_OUT frame;
	for (int iter = 0; iter < 3; ++iter)
	{
		system("pause");
		std::string out_win = "output_style_full_screen";
		cv::namedWindow(out_win, cv::WINDOW_NORMAL);
		cv::setWindowProperty(out_win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		for (int i = 0; i < imgs.size(); ++i)
		{
			cv::imshow(out_win, *imgs[i].get());
			cv::waitKey(1000);
			clock_t time = clock();
			if (camera.GetImageBuffer(&frame, 1000) == MV_OK)
			{
				cv::Mat img;
				if (!camera.Convert2Mat(&frame.stFrameInfo, frame.pBufAddr, img))
				{
					std::cout << "Convert2Mat error" << std::endl;
				}
				else
				{
					//cv::resize(img, img, cv::Size(1920, 1080));
					cv::imwrite("D:\\Calibration_imgs\\" + std::to_string(iter) + "\\" + imgsName[i] + ".bmp", img);
				}
			}
			else
			{
				std::cout << "GetImageBuffer error: " << std::to_string(i) << std::endl;
			}
			std::cout << "get image time: " << clock() - time << std::endl;
			camera.FreeImageBuffer(&frame);
		}
	}

#elif 0
	//投影仪和相机标定
	std::vector<std::vector<std::shared_ptr<cv::Mat>>> images_list_;
	images_list_.resize(3);
	for (int k = 0; k < 3; ++k)
	{
		std::string filePath = "D:\\Calibration_imgs\\" + std::to_string(k);
		std::vector<std::string> files;
		getAllFiles(filePath, files, "bmp");
		std::vector<std::shared_ptr<cv::Mat>>& images = images_list_[k];
		images.resize(files.size());
		for (int i = 0; i < files.size(); ++i)
		{
			std::cout << files[i] << std::endl;
			images[i].reset(new cv::Mat);
			*images[i].get() = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		}
	}
	sr::projector_Camera_Calibration(images_list_, 11, 8, 15, 1920, 1080);

//采集重建数据
#elif 0
	std::vector<std::string> files;
	getAllFiles("D:\\projector_pattern", files, "bmp");
	//getAllFiles("D:\\GrayCode_Calibration", files, "bmp");
	std::vector<std::shared_ptr<cv::Mat>> imgs(files.size());
	for (int i = 0; i < files.size(); ++i)
	{
		imgs[i].reset(new cv::Mat);
		*imgs[i].get() = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		std::cout << files[i] << std::endl;
	}

	//查找并启动相机
	int nRet = MV_OK;
	CMvCamera camera;
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	//查找并启动相机
	camera.EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
	camera.Open(stDeviceList.pDeviceInfo[0]);
	camera.SetEnumValue("TriggerMode", 0);
	//camera.SetEnumValue("PixelFormat", PixelType_Gvsp_RGB8_Packed);
	camera.SetEnumValue("PixelFormat", PixelType_Gvsp_Mono8);

	MV_CC_SetExposureAutoMode(camera.getDeviceHandle(), 2);
	MV_CC_SetAutoExposureTimeLower(camera.getDeviceHandle(), 15000);
	MV_CC_SetAutoExposureTimeUpper(camera.getDeviceHandle(), 500000);

	//MV_CC_SetExposureTime(camera.getDeviceHandle(), 50000);
	if (camera.StartGrabbing() != MV_OK)
	{
		std::cout << "StartGrabbing error" << std::endl;
		return 0;
	}
	Sleep(5000); //等待相机启动

	MV_FRAME_OUT frame;
	for (int iter = 0; iter < 1; ++iter)
	{
		system("pause");
		std::string out_win = "output_style_full_screen";
		cv::namedWindow(out_win, cv::WINDOW_NORMAL);
		cv::setWindowProperty(out_win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		for (int i = 0; i < imgs.size(); ++i)
		{
			cv::imshow(out_win, *imgs[i].get());
			cv::waitKey(1000);
			clock_t time = clock();
			if (camera.GetImageBuffer(&frame, 1000) == MV_OK)
			{
				cv::Mat img;
				if (!camera.Convert2Mat(&frame.stFrameInfo, frame.pBufAddr, img))
				{
					std::cout << "Convert2Mat error" << std::endl;
				}
				else
				{
					cv::imwrite("D:\\left_camera_img\\" + std::to_string(iter) + "\\" + imgsName[i] + ".bmp", img);
				}
			}
			else
			{
				std::cout << "GetImageBuffer error: " << std::to_string(i) << std::endl;
			}
			std::cout << "get image time: " << clock() - time << std::endl;
			camera.FreeImageBuffer(&frame);
		}
	}

	//格雷码 + 射线求交 重建
#elif 1
	std::vector<std::string> files;
	getAllFiles("D:\\left_camera_img\\1", files, ".bmp");
	std::vector<std::shared_ptr<cv::Mat>> grayCodeImgs;
	grayCodeImgs.resize(files.size());
	for (int i = 0; i < files.size(); ++i)
	{
		grayCodeImgs[i].reset(new cv::Mat);
		*grayCodeImgs[i].get() = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		std::cout << files[i] << std::endl;
	}
	cv::Mat pattern_image;
	cv::Mat min_max_image;
	if (!sl::decode_pattern(grayCodeImgs, pattern_image, min_max_image, cv::Size(1920, 1080), sl::GrayPatternDecode))
	{
		std::cout << "decode error" << std::endl;
	}
	std::cout << pattern_image.size() << ", " << min_max_image.size() << std::endl;
	sr::reconstruct_model_patch_center(pattern_image, min_max_image, cv::Size(1920, 1080), 0, 10000);

	//格雷码 + 最小二乘重建
#elif 0
	std::vector<std::string> files;
	getAllFiles("D:\\left_camera_img\\2", files, ".bmp");
	//getAllFiles("D:\\projector_pattern", files, ".bmp");
	std::vector<std::shared_ptr<cv::Mat>> grayCodeImgs_col, grayCodeImgs_row;
	grayCodeImgs_col.resize(files.size());
	for (int i = 0; i < files.size(); ++i)
	{
		grayCodeImgs_col[i].reset(new cv::Mat);
		*grayCodeImgs_col[i].get() = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		std::cout << files[i] << std::endl;
	}
	grayCodeImgs_row.insert(grayCodeImgs_row.end(), grayCodeImgs_col.begin() + grayCodeImgs_col.size() / 2, grayCodeImgs_col.end());
	grayCodeImgs_col.resize(grayCodeImgs_col.size() / 2);
	std::cout << grayCodeImgs_row.size() << ", " << grayCodeImgs_col.size() << std::endl;

	//解码
	cv::Mat code_col, code_row;
	sr::decodeGrayCode(grayCodeImgs_col, code_col);
	sr::decodeGrayCode(grayCodeImgs_row, code_row);
	for (int i = 0; i < code_col.cols; ++i)
	{
		std::cout << code_col.at<float>(code_col.rows / 2, i) << ", ";
	}
	std::cout << std::endl << std::endl;
	for (int i = 0; i < code_row.rows; ++i)
	{
		std::cout << code_row.at<float>(i, code_row.cols / 2) << ", ";
	}
	std::cout << std::endl << std::endl;

	//查找对应点
	std::vector<cv::Point2f> leftPoints, rightPoints;
	{
		std::map<unsigned, cv::Point2f> proj_points;
		std::map<unsigned, std::vector<cv::Point2f>> cam_points;
		for (int i = 0; i < code_row.rows; ++i)
		{
			for (int j = 0; j < code_row.cols; ++j)
			{
				int row = code_row.at<float>(i, j);
				int col = code_col.at<float>(i, j);
				if (row > 0 && row < 1000 && col > 0 && col < 1900)
				{
					unsigned index = col * 1920 * row;
					proj_points.insert(std::make_pair(index, cv::Point2f(col, row)));
					cam_points[index].push_back(cv::Point2f(j, i));
				}
			}
		}
		for (auto iter = proj_points.begin(); iter != proj_points.end(); ++iter)
		{
			unsigned index = iter->first;
			rightPoints.push_back(iter->second);
			cv::Point2f left(0.f, 0.f);
			const std::vector<cv::Point2f>& cam_point_list = cam_points[index];
			float count = static_cast<float>(cam_point_list.size());
			for (auto iter2 : cam_point_list)
			{
				left.x += iter2.x;
				left.y += iter2.y;
			}
			left.x /= count;
			left.y /= count;
			leftPoints.push_back(left);
		}
	}

	//最小二乘求解
	if (true)
	{
		std::shared_ptr<trimesh::TriMesh> debug(new trimesh::TriMesh);
		CalibrationData calib;
		calib.load_calibration("D:\\calib_file_0.yml");
		//https://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf
		cv::Mat R_inv = calib.R.inv();
		cv::Mat T1 = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
		cv::Mat T2 = (cv::Mat_<float>(3, 4) << R_inv.at<double>(0, 0),
		              R_inv.at<double>(0, 1),
		              R_inv.at<double>(0, 2),
		              calib.T.at<double>(0, 0) * -1.0,
		              R_inv.at<double>(1, 0),
		              R_inv.at<double>(1, 1),
		              R_inv.at<double>(1, 2),
		              calib.T.at<double>(1, 0) * -1.0,
		              R_inv.at<double>(2, 0),
		              R_inv.at<double>(2, 1),
		              R_inv.at<double>(2, 2),
		              calib.T.at<double>(2, 0) * -1.0);
		for (int i = 0; i < leftPoints.size(); ++i)
		{
			cv::Mat inp1(1, 1, CV_64FC2), inp2(1, 1, CV_64FC2);
			inp1.at<cv::Vec2d>(0, 0) = cv::Vec2d(leftPoints[i].x, leftPoints[i].y);
			inp2.at<cv::Vec2d>(0, 0) = cv::Vec2d(rightPoints[i].x, rightPoints[i].y);
			cv::Mat outp1, outp2;
			cv::undistortPoints(inp1, outp1, calib.cam_K, calib.cam_kc);
			cv::undistortPoints(inp2, outp2, calib.proj_K, calib.proj_kc);
			assert(outp1.type() == CV_64FC2 && outp1.rows == 1 && outp1.cols == 1);
			assert(outp2.type() == CV_64FC2 && outp2.rows == 1 && outp2.cols == 1);
			const cv::Vec2d& outvec1 = outp1.at<cv::Vec2d>(0, 0);
			const cv::Vec2d& outvec2 = outp2.at<cv::Vec2d>(0, 0);
			cv::Point3d p1(outvec1[0], outvec1[1], 1.0);
			cv::Point3d p2(outvec2[0], outvec2[1], 1.0);

			cv::Mat A = (cv::Mat_<float>(4, 4));
			A.row(0) = p1.y * T1.row(2) - T1.row(1);
			A.row(1) = T1.row(0) - p1.x * T1.row(2);
			A.row(2) = p2.y * T2.row(2) - T2.row(1);
			A.row(3) = T2.row(0) - p2.x * T2.row(2);
			Eigen::Matrix<float, 4, 4> EA;
			cv::cv2eigen(A, EA);
			Eigen::Matrix<float, 4, 4> C = EA.transpose() * EA;

			/*cv::Mat A = (cv::Mat_<float>(3, 4));
			A.row(0) = p1.y * T1.row(2) - T1.row(1);
			A.row(1) = T1.row(0) - p1.x * T1.row(2);
			A.row(2) = p2.y * T2.row(2) - T2.row(1);
			Eigen::Matrix<float, 3, 4> EA;
			cv::cv2eigen(A, EA);
			Eigen::Matrix<float, 4, 4> C = EA.transpose() * EA;*/

			Eigen::EigenSolver<Eigen::MatrixXf> es(C);
			Eigen::VectorXcf eigenValues = es.eigenvalues();
			Eigen::MatrixXcf eigenVectors = es.eigenvectors();
			std::pair<float, int> minValue(FLT_MAX, -1);
			for (int j = 0; j < eigenValues.size(); ++j)
			{
				if (eigenValues[j].real() < minValue.first)
				{
					minValue.first = eigenValues[j].real();
					minValue.second = j;
				}
			}
			trimesh::point p(eigenVectors(0, minValue.second).real(), eigenVectors(1, minValue.second).real(), eigenVectors(2, minValue.second).real());
			float scale = eigenVectors(3, minValue.second).real();

			/*std::cout << eigenValues.size() << ", " << eigenVectors.size() << std::endl;
			std::cout << eigenValues << std::endl << eigenVectors << std::endl;
			std::cout << minValue.second << ", "<< p << ", " << scale << std::endl<<std::endl;*/
			p /= scale;
			debug->vertices.push_back(p);
		}
		debug->write("D:/debug.ply");
	}

#endif

	std::cout << "main time: " << clock() - main_time << std::endl;
	system("pause");
}
