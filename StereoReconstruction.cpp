#include <io.h>
#include <trimesh.h>
#include <trimesh_algo.h>
#include <windows.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "Calibration/Calibration.h"
#include "Calibration/Encoder.h"
#include "Reconstruction/Reconstruction.h"

static void getAllFiles(std::string path, std::vector<std::string> &files, std::string fileType)
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
#if 1
	//生成用于标定的格雷码图像
	std::vector<std::shared_ptr<cv::Mat>> colStripesImgs, rowStripesImgs;
	sr::Encoder_Gray(colStripesImgs, 11, true);
	sr::Encoder_Gray(rowStripesImgs, 11, false);
	colStripesImgs.insert(colStripesImgs.end(), rowStripesImgs.begin(), rowStripesImgs.end());
	for (int i = 0; i < colStripesImgs.size(); ++i)
	{
		cv::Mat img = *colStripesImgs[i].get();
		cv::Rect rect((img.cols - 1920) / 2, (img.rows - 1080) / 2, 1920, 1080);
		cv::Mat cropped = img(rect);
		cv::imwrite("D://GrayCode_Calibration//" + std::to_string(1 + 2 + i * 2) + ".bmp", cropped);
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
		cv::imwrite("D://GrayCode_Calibration//" + std::to_string(1 + 2 + i * 2 + 1) + ".bmp", cropped);
	}

	cv::Mat img(1080, 1920, CV_8UC1);
	img.setTo(255);
	cv::imwrite("D://GrayCode_Calibration//1.bmp", img);
	img.setTo(50);
	cv::imwrite("D://GrayCode_Calibration//2.bmp", img);

	
#elif 0
	//投影仪和相机标定
	std::cout << "hello world" << std::endl;
	std::vector<std::vector<std::shared_ptr<cv::Mat>>> images_list_;
	images_list_.resize(3);
	for (int k = 0; k < 3; ++k)
	{
		std::string filePath = "C:\\Users\\admin\\Downloads\\calib-data-small\\calib-small\\" + std::to_string(k);
		std::vector<std::string> files;
		getAllFiles(filePath, files, "jpg");
		std::vector<std::shared_ptr<cv::Mat>>& images = images_list_[k];
		images.resize(files.size());
		for (int i = 0; i < files.size(); ++i)
		{
			images[i].reset(new cv::Mat);
			*images[i].get() = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		}
	}
	sr::projector_Camera_Calibration(images_list_);

#elif 1
	cv::Mat img = cv::imread("D:/001.png", CV_LOAD_IMAGE_COLOR);
	std::string out_win = "output_style_full_screen";
	cv::namedWindow(out_win, cv::WINDOW_NORMAL);
	cv::setWindowProperty(out_win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	cv::imshow(out_win, img);
	cv::waitKey(1000);

#endif

	std::cout << "main time: " << clock() - main_time << std::endl;
	system("pause");
}
