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
#include "Calibration/CalibrationData.hpp"
#include "Calibration/Encoder.h"
#include "Calibration/structured_light.hpp"
#include "MvCamera/MvCamera.h"
#include "Reconstruction/Reconstruction.h"

std::string pathPrefix = "D:\\Algorithms\\StereoReconstruction\\Data\\";

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

//单目(相机 + 投影仪)标定的格雷码图像
#if 0
	std::vector<std::shared_ptr<cv::Mat>> colStripesImgs, rowStripesImgs;
	sr::Encoder_Gray(colStripesImgs, 11, true);
	sr::Encoder_Gray(rowStripesImgs, 11, false);
	colStripesImgs.insert(colStripesImgs.end(), rowStripesImgs.begin(), rowStripesImgs.end());
	for (int i = 0; i < colStripesImgs.size(); ++i)
	{
		cv::Mat img = *colStripesImgs[i].get();
		cv::Rect rect(0, 0, 1920, 1080);
		cv::Mat cropped = img(rect);
		cv::imwrite(pathPrefix +  "Calibration_Pattern\\" + imgsName[2 + i * 2] + ".bmp", cropped);
		img = *colStripesImgs[i].get();
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
		cv::imwrite(pathPrefix + "Calibration_Pattern\\" + imgsName[2 + i * 2 + 1] + ".bmp", cropped);
	}

	cv::Mat img(1080, 1920, CV_8UC1);
	img.setTo(255);
	cv::imwrite(pathPrefix + "Calibration_Pattern\\01.bmp", img);
	img.setTo(0);
	cv::imwrite(pathPrefix + "Calibration_Pattern\\02.bmp", img);

	//重建(格雷码 + 相移)图像
#elif 0
	std::vector<std::shared_ptr<cv::Mat>> grayCodeImgs;
	sr::Encoder_GrayCodeImg(grayCodeImgs, 1920, 1080, 8, 8, true);
	std::vector<std::shared_ptr<cv::Mat>> PhaseImgs;
	sr::Encoder_PhaseImg(PhaseImgs, 1920, 1080, 8, true);
	grayCodeImgs.insert(grayCodeImgs.end(), PhaseImgs.begin(), PhaseImgs.end());
	for (int i = 0; i < grayCodeImgs.size(); ++i)
	{
		cv::imwrite(pathPrefix + "GrayCode_PhaseShift\\" + imgsName[i] + ".bmp", *grayCodeImgs[i].get());
	}

//单目(相机 + 投影仪)标定数据采集
#elif 0
	std::vector<std::string> files;
	getAllFiles(pathPrefix + "Calibration_Pattern\\", files, "bmp");
	std::vector<std::shared_ptr<cv::Mat>> imgs(files.size());
	for (int i = 0; i < files.size(); ++i)
	{
		imgs[i].reset(new cv::Mat);
		*imgs[i].get() = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
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

	MV_CC_SetExposureAutoMode(camera.getDeviceHandle(), 0);
	MV_CC_SetExposureTime(camera.getDeviceHandle(), 4000);

	//MV_CC_SetExposureAutoMode(camera.getDeviceHandle(), 2);
	//MV_CC_SetAutoExposureTimeLower(camera.getDeviceHandle(), 25000);
	//MV_CC_SetAutoExposureTimeUpper(camera.getDeviceHandle(), 500000);
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
					cv::imwrite(pathPrefix + "Single_Calibration_imgs\\" + std::to_string(iter) + "\\" + imgsName[i] + ".bmp", img);
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
	//单目(投影仪+相机)标定
	std::vector<std::vector<std::shared_ptr<cv::Mat>>> images_list_;
	images_list_.resize(3);
	for (int k = 0; k < 3; ++k)
	{
		std::string filePath = pathPrefix + "Single_Calibration_imgs\\" + std::to_string(k);
		std::vector<std::string> files;
		getAllFiles(filePath, files, "bmp");
		std::vector<std::shared_ptr<cv::Mat>>& images = images_list_[k];
		images.resize(files.size());
		for (int i = 0; i < files.size(); ++i)
		{
			std::cout << files[i] << std::endl;
			images[i].reset(new cv::Mat);
			*images[i].get() = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
		}
	}
	sr::projector_Camera_Calibration(images_list_, pathPrefix + "calib.yml", 17, 17, 10, 1920, 1080);

	//单目重建数据采集
#elif 0
	std::vector<std::string> files;
	getAllFiles(pathPrefix + "Calibration_Pattern", files, "bmp");
	std::vector<std::shared_ptr<cv::Mat>> imgs(files.size());
	for (int i = 0; i < files.size(); ++i)
	{
		imgs[i].reset(new cv::Mat);
		*imgs[i].get() = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
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

	MV_CC_SetExposureAutoMode(camera.getDeviceHandle(), 0);
	MV_CC_SetExposureTime(camera.getDeviceHandle(), 50000);
	/*MV_CC_SetExposureAutoMode(camera.getDeviceHandle(), 2);
	MV_CC_SetAutoExposureTimeLower(camera.getDeviceHandle(), 25000);
	MV_CC_SetAutoExposureTimeUpper(camera.getDeviceHandle(), 500000);*/

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
					cv::imwrite(pathPrefix + "SingleCameraImage\\" + std::to_string(iter) + "\\" + imgsName[i] + ".bmp", img);
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

	//单目重建(纯格雷码 + 射线求交)
#elif 0
	std::vector<std::string> files;
	getAllFiles(pathPrefix + "SingleCameraImage\\0", files, ".bmp");
	std::vector<std::shared_ptr<cv::Mat>> grayCodeImgs;
	grayCodeImgs.resize(files.size());
	for (int i = 0; i < files.size(); ++i)
	{
		grayCodeImgs[i].reset(new cv::Mat);
		*grayCodeImgs[i].get() = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
		std::cout << files[i] << std::endl;
	}
	cv::Mat pattern_image;
	cv::Mat min_max_image;
	if (!sl::decode_pattern(grayCodeImgs, pattern_image, min_max_image, cv::Size(1920, 1080), sl::GrayPatternDecode))
	{
		std::cout << "decode error" << std::endl;
	}
	sr::reconstruct_model_patch_center(pattern_image, min_max_image, cv::Size(1920, 1080), 0, 10000);

	//双目标定数据采集
#elif 0
	//查找并启动相机
	int nRet = MV_OK;
	std::vector<CMvCamera> cameras(2);
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	CMvCamera::EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
	std::cout << "stDeviceList.nDeviceNum: " << stDeviceList.nDeviceNum << std::endl;
	if (stDeviceList.nDeviceNum != 2)
	{
		std::cout << "stDeviceList.nDeviceNum != 2" << std::endl;
	}
	else
	{
		for (int i = 0; i < stDeviceList.nDeviceNum; ++i)
		{
			std::string deviceNmae = (char*)stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chUserDefinedName;
			if (deviceNmae == "LeftCamera")
			{
				cameras[0].Open(stDeviceList.pDeviceInfo[i]);
				std::cout << "open leftCmaera" << std::endl;
			}
			else if (deviceNmae == "RightCamera")
			{
				cameras[1].Open(stDeviceList.pDeviceInfo[i]);
				std::cout << "open rightCmaera" << std::endl;
			}
		}
	}
	for (int i = 0; i < cameras.size(); ++i)
	{
		CMvCamera& camera = cameras[i];
		camera.SetEnumValue("TriggerMode", 0);
		camera.SetEnumValue("PixelFormat", PixelType_Gvsp_Mono8);
		MV_CC_SetExposureAutoMode(camera.getDeviceHandle(), 0);
		if (i == 0)
		{
			//左相机
			MV_CC_SetExposureTime(camera.getDeviceHandle(), 100000);
		}
		else
		{
			MV_CC_SetExposureTime(camera.getDeviceHandle(), 200000);
		}
		if (camera.StartGrabbing() != MV_OK)
		{
			std::cout << "StartGrabbing error, camera " + std::to_string(i) << std::endl;
			system("pause");
			return 0;
		}
	}
	Sleep(5000); //等待相机启动

	for (int iter = 0; iter < 30; ++iter)
	{
		system("pause");
		for (int i = 0; i < cameras.size(); ++i)
		{
			MV_FRAME_OUT frame;
			CMvCamera& camera = cameras[i];
			if (camera.GetImageBuffer(&frame, 1000) == MV_OK)
			{
				cv::Mat img;
				if (!camera.Convert2Mat(&frame.stFrameInfo, frame.pBufAddr, img))
				{
					std::cout << "Convert2Mat error" << std::endl;
				}
				else
				{
					if (i == 0)
					{
						cv::imwrite(pathPrefix + "Dual_Calib_Imgs\\left\\" + imgsName[iter] + ".bmp", img);
					}
					else
					{
						cv::imwrite(pathPrefix + "Dual_Calib_Imgs\\right\\" + imgsName[iter] + ".bmp", img);
					}
				}
			}
			else
			{
				std::cout << "GetImageBuffer error: " << std::to_string(i) << std::endl;
			}
			camera.FreeImageBuffer(&frame);
			Sleep(300);
		}
	}

	for (int i = 0; i < cameras.size(); ++i)
	{
		cameras[i].Close();
	}

//双目重建数据采集
#elif 0
	std::vector<std::string> files;
	getAllFiles(pathPrefix + "GrayCode_PhaseShift", files, "bmp");
	std::vector<std::shared_ptr<cv::Mat>> imgs(files.size());
	for (int i = 0; i < files.size(); ++i)
	{
		imgs[i].reset(new cv::Mat);
		*imgs[i].get() = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
		std::cout << files[i] << std::endl;
	}

	//查找并启动相机
	int nRet = MV_OK;
	std::vector<CMvCamera> cameras(2);
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	//查找并启动相机
	CMvCamera::EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
	std::cout << "stDeviceList.nDeviceNum: " << stDeviceList.nDeviceNum << std::endl;
	if (stDeviceList.nDeviceNum < 2)
	{
		std::cout << "stDeviceList.nDeviceNum < 2" << std::endl;
	}
	else
	{
		for (int i = 0; i < stDeviceList.nDeviceNum; ++i)
		{
			std::string deviceNmae = (char*)stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chUserDefinedName;
			if (deviceNmae == "LeftCamera")
			{
				cameras[0].Open(stDeviceList.pDeviceInfo[i]);
				std::cout << "open leftCmaera" << std::endl;
			}
			else if (deviceNmae == "RightCamera")
			{
				cameras[1].Open(stDeviceList.pDeviceInfo[i]);
				std::cout << "open rightCmaera" << std::endl;
			}
		}
	}
	for (int i = 0; i < cameras.size(); ++i)
	{
		CMvCamera& camera = cameras[i];
		camera.SetEnumValue("TriggerMode", 0);
		camera.SetEnumValue("PixelFormat", PixelType_Gvsp_Mono8);
		MV_CC_SetExposureAutoMode(camera.getDeviceHandle(), 0);
		if (i == 0)
		{
			MV_CC_SetExposureTime(camera.getDeviceHandle(), 5000);
		}
		else
		{
			MV_CC_SetExposureTime(camera.getDeviceHandle(), 15000);
		}
		if (camera.StartGrabbing() != MV_OK)
		{
			std::cout << "StartGrabbing error, camera " + std::to_string(i) << std::endl;
			system("pause");
			return 0;
		}
	}
	Sleep(5000); //等待相机启动

	std::string out_win = "output_style_full_screen";
	cv::namedWindow(out_win, cv::WINDOW_NORMAL);
	cv::setWindowProperty(out_win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	for (int i = 0; i < imgs.size(); ++i)
	{
		cv::imshow(out_win, *imgs[i].get());
		cv::waitKey(5000);
		for (int j = 0; j < cameras.size(); ++j)
		{
			MV_FRAME_OUT frame;
			CMvCamera& camera = cameras[j];
			if (camera.GetImageBuffer(&frame, 1000) == MV_OK)
			{
				cv::Mat img;
				if (!camera.Convert2Mat(&frame.stFrameInfo, frame.pBufAddr, img))
				{
					std::cout << "Convert2Mat error" << std::endl;
				}
				else
				{
					if (j == 0)
					{
						cv::imwrite(pathPrefix + "Dual_Recon_Imgs\\left\\" + imgsName[i] + ".bmp", img);
					}
					else
					{
						cv::imwrite(pathPrefix + "Dual_Recon_Imgs\\right\\" + imgsName[i] + ".bmp", img);
					}
				}
			}
			else
			{
				std::cout << "GetImageBuffer error: " << std::to_string(i) << std::endl;
			}
			camera.FreeImageBuffer(&frame);
		}
	}

	for (int i = 0; i < cameras.size(); ++i)
	{
		cameras[i].Close();
	}

	//双目标定
#elif 1
	std::vector<std::string> leftFile, rightFile;
	getAllFiles(pathPrefix + "Dual_Calib_Imgs\\left", leftFile, "bmp");
	getAllFiles(pathPrefix + "Dual_Calib_Imgs\\right", rightFile, "bmp");
	std::vector<std::shared_ptr<cv::Mat>> leftImages(leftFile.size()), rightImages(rightFile.size());
	for (int i = 0; i < leftFile.size(); ++i)
	{
		leftImages[i].reset(new cv::Mat);
		*leftImages[i].get() = cv::imread(leftFile[i], cv::IMREAD_GRAYSCALE);
		std::cout << leftFile[i] << std::endl;
	}
	for (int i = 0; i < rightFile.size(); ++i)
	{
		rightImages[i].reset(new cv::Mat);
		*rightImages[i].get() = cv::imread(rightFile[i], cv::IMREAD_GRAYSCALE);
		std::cout << rightFile[i] << std::endl;
	}

	//注意查看角点的顺序是否正确，否则会导致矫正后图像出现非常离谱的扭曲
	sr::calib_stereo(leftImages, rightImages, 17, 17, 10, pathPrefix + "calib.yml");

	//双目重建
#elif 0
	std::vector<std::string> leftFile, rightFile;
	getAllFiles(pathPrefix + "Dual_Recon_Imgs\\left", leftFile, ".bmp");
	getAllFiles(pathPrefix + "Dual_Recon_Imgs\\right", rightFile, ".bmp");
	std::vector<std::shared_ptr<cv::Mat>> leftImage(leftFile.size()), rightImage(rightFile.size());
	for (int i = 0; i < leftFile.size(); ++i)
	{
		leftImage[i].reset(new cv::Mat);
		*leftImage[i].get() = cv::imread(leftFile[i], cv::IMREAD_GRAYSCALE);
		std::cout << leftFile[i] << std::endl;
	}
	for (int i = 0; i < rightFile.size(); ++i)
	{
		rightImage[i].reset(new cv::Mat);
		*rightImage[i].get() = cv::imread(rightFile[i], cv::IMREAD_GRAYSCALE);
		std::cout << rightFile[i] << std::endl;
	}

	//图像矫正
	for (int i = 0; i < leftImage.size(); ++i)
	{
		sr::undistort_rectify(pathPrefix + "calib.yml", leftImage[i].get(), rightImage[i].get(), leftImage[i].get(), rightImage[i].get());
	}

	//解码
	std::vector<std::shared_ptr<cv::Mat>> leftPhaseImage, rightPhaseImage;
	for (int i = 0; i < 4; ++i)
	{
		leftPhaseImage.insert(leftPhaseImage.begin(), leftImage.back());
		leftImage.pop_back();
		rightPhaseImage.insert(rightPhaseImage.begin(), rightImage.back());
		rightImage.pop_back();
	}
	cv::Mat leftPhase, rightPhase;
	leftPhase = sr::CalWrappedPhase(leftPhaseImage);
	rightPhase = sr::CalWrappedPhase(rightPhaseImage);
	cv::Mat leftUnwrappedPhase, rightUnwrappedPhase;
	sr::UnwrappedPhaseGraycodeMethod(leftPhase, leftUnwrappedPhase, leftImage);
	sr::UnwrappedPhaseGraycodeMethod(rightPhase, rightUnwrappedPhase, rightImage);

	//查找对应点
	std::vector<cv::Point2f> leftPoints, rightPoints;
	for (int y = 0; y < leftUnwrappedPhase.rows; ++y)
	{
		for (int x = 0; x < leftUnwrappedPhase.cols; ++x)
		{
			float left = leftUnwrappedPhase.at<float>(y, x);
			if (left < M_PIf * 2.f)
			{
				continue;
			}
			std::pair<float, int> bestMatch(FLT_MAX, -1);
			for (int k = 0; k < rightUnwrappedPhase.cols; ++k)
			{
				float right = rightUnwrappedPhase.at<float>(y, k);
				if (std::abs(left - right) < bestMatch.first)
				{
					bestMatch.first = std::abs(left - right);
					bestMatch.second = k;
				}
			}
			if (bestMatch.first < 0.3)
			{
				leftPoints.push_back(cv::Point2f(x, y));
				rightPoints.push_back(cv::Point2f(bestMatch.second, y));
			}
		}
	}
	std::cout << "leftPoints.size(): " << leftPoints.size() << std::endl;

	//重建
	//
	//平行式光轴方案，重建出的Z值不连续的原因：
	//目标物体在视野范围的占比过小，且相机距离过近，且对应点查找不是亚像素匹配，导致一单位像素对应的Z轴长度过长。
	//所以相机选型时要注意！要根据视野范围、工作距离、精度要求，选取合适焦距的镜头、合适像素的相机！
	if (true)
	{
		//最小二乘求解目标点
		//https://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf
		std::shared_ptr<trimesh::TriMesh> debug(new trimesh::TriMesh);
		debug->vertices.resize(leftPoints.size());
		std::vector<bool> rmv(leftPoints.size(), true);
		cv::FileStorage fs1(pathPrefix + "calib.yml", cv::FileStorage::READ);
		cv::Mat P1, P2;
		fs1["P1"] >> P1;
		fs1["P2"] >> P2;
		Eigen::Matrix<float, 3, 4> T1, T2;
		cv::cv2eigen(P1, T1);
		cv::cv2eigen(P2, T2);
		std::cout << T1 << std::endl;
		std::cout << T2 << std::endl;
#pragma omp parallel for
		for (int i = 0; i < leftPoints.size(); ++i)
		{
			const cv::Point2f &p1 = leftPoints[i];
			const cv::Point2f &p2 = rightPoints[i];
			Eigen::Matrix<float, 4, 4> A;
			A.row(0) = p1.y * T1.row(2) - T1.row(1);
			A.row(1) = T1.row(0) - p1.x * T1.row(2);
			A.row(2) = p2.y * T2.row(2) - T2.row(1);
			A.row(3) = T2.row(0) - p2.x * T2.row(2);

			Eigen::EigenSolver<Eigen::MatrixXf> es(A.transpose() * A);
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
			p /= scale;

			debug->vertices[i] = p;
			if (trimesh::dist(p, trimesh::point(0.f)) < 1000)
			{
				rmv[i] = false;
			}
		}
		trimesh::remove_vertices(debug.get(), rmv);
		debug->write(pathPrefix + "debug.ply");
	}
	else
	{
		std::shared_ptr<trimesh::TriMesh> debug(new trimesh::TriMesh);
		cv::FileStorage fs1(pathPrefix + "calib.yml", cv::FileStorage::READ);
		cv::Mat P1, P2;
		fs1["P1"] >> P1;
		fs1["P2"] >> P2;
		cv::Mat pnts(4, leftPoints.size(), CV_64F);
		cv::triangulatePoints(P1, P2, leftPoints, rightPoints, pnts);
		float *pnts3D_row1 = pnts.ptr<float>(0);
		float *pnts3D_row2 = pnts.ptr<float>(1);
		float *pnts3D_row3 = pnts.ptr<float>(2);
		float *pnts3D_row4 = pnts.ptr<float>(3);
		for (int i = 0; i < pnts.cols; i++)
		{
			float pnts3D_data4 = *(pnts3D_row4 + i);

			float pnts3D_data1 = *(pnts3D_row1 + i) / pnts3D_data4;
			float pnts3D_data2 = *(pnts3D_row2 + i) / pnts3D_data4;
			float pnts3D_data3 = *(pnts3D_row3 + i) / pnts3D_data4;
			trimesh::point p(pnts3D_data1, pnts3D_data2, pnts3D_data3);
			if (trimesh::length(p) < 1000)
			{
				debug->vertices.push_back(p);
			}
		}
		debug->write(pathPrefix + "debug.ply");
	}

#endif

	std::cout << "main time: " << clock() - main_time << std::endl;
	system("pause");
}
