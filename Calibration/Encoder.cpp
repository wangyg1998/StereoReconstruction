#include "Encoder.h"

#include <fstream>
#include <opencv2/highgui.hpp>
#include <strstream>
namespace sr
{

bool getGrayCode(std::vector<std::vector<bool>>& grayCode, int order)
{
	grayCode.clear();
	if (order < 4 || order > 16)
	{
		return false;
	}

	//获取value的第n个二进制位
	auto getBit = [](int value, int n) -> bool
	{
		bool res = (value >> (n)) & 0x0001;
		return res;
	};

	int stripeNumber = std::pow(2, order);

	//格雷码编码
	grayCode.resize(stripeNumber, std::vector<bool>(order));
	for (int i = 0; i < grayCode.size(); ++i)
	{
		std::vector<bool>& code = grayCode[i];
		for (int j = 0; j < order; ++j)
		{
			code[j] = getBit(i, j);
		}
		std::vector<bool> temp = code;
		for (int j = code.size() - 2; j >= 0; --j)
		{
			code[j] = temp[j] xor temp[j + 1];
		}
		/*for (int j = code.size() - 1; j >= 0; --j)
		{
			std::cout << code[j];
		}
		std::cout << std::endl;*/
	}
	return true;
}

bool Encoder_GrayCodeImg(std::vector<std::shared_ptr<cv::Mat>>& grayCodeImgs,
                         int projectorWidth,
                         int projectorHeight,
                         int pixelPerPeriod,
                         int order,
                         bool colBased)
{
	grayCodeImgs.clear();
	std::vector<std::vector<bool>> grayCode;
	if (!getGrayCode(grayCode, order))
	{
		return false;
	}

	//生成图像
	grayCodeImgs.resize(order);
	for (int k = 0; k < order; ++k)
	{
		grayCodeImgs[k].reset(new cv::Mat(projectorHeight, projectorWidth, CV_8UC1));
		cv::Mat& img = *grayCodeImgs[k].get();
		img.setTo(0);
		if (colBased)
		{
			for (int i = 0; i < grayCode.size() && i * pixelPerPeriod < img.cols; ++i)
			{
				if (grayCode[i][order - k - 1])
				{
					for (int j = 0; j < pixelPerPeriod && (i * pixelPerPeriod + j) < img.cols; ++j)
					{
						img.at<char>(0, i * pixelPerPeriod + j) = 255;
					}
				}
			}
			for (int i = 1; i < img.rows; ++i)
			{
				img.row(0).copyTo(img.row(i));
			}
		}
		else
		{
			for (int i = 0; i < grayCode.size() && i * pixelPerPeriod < img.rows; ++i)
			{
				if (grayCode[i][order - k - 1])
				{
					for (int j = 0; j < pixelPerPeriod && (i * pixelPerPeriod + j) < img.rows; ++j)
					{
						img.at<char>(i * pixelPerPeriod + j, 0) = 255;
					}
				}
			}
			for (int i = 1; i < img.cols; ++i)
			{
				img.col(0).copyTo(img.col(i));
			}
		}
		/*cv::imshow("gray code mat", img);
		cv::waitKey(0);*/
	}

	return true;
}

bool Encoder_PhaseImg(std::vector<std::shared_ptr<cv::Mat>>& PhaseImgs, int projectorWidth, int projectorHeight, int pixelPerPeriod, bool colBased)
{
	PhaseImgs.clear();
	PhaseImgs.resize(4);
	for (int i = 0; i < PhaseImgs.size(); ++i)
	{
		PhaseImgs[i].reset(new cv::Mat(projectorHeight, projectorWidth, CV_32FC1));
	}
	if (colBased)
	{
		int numPeriod = projectorWidth / pixelPerPeriod + 1;
		for (int i = 0; i < numPeriod; i++)
		{
			for (int j = 0; j < pixelPerPeriod && (i * pixelPerPeriod + j) < projectorWidth; j++)
			{
				float x = static_cast<float>(j) / static_cast<float>(pixelPerPeriod) * 2.f * M_PI;
				PhaseImgs[0]->at<float>(0, i * pixelPerPeriod + j) = (sin(x) + 1.f) * 127.f;
				PhaseImgs[1]->at<float>(0, i * pixelPerPeriod + j) = (sin(x + M_PI / 2.f) + 1.f) * 127.f;
				PhaseImgs[2]->at<float>(0, i * pixelPerPeriod + j) = (sin(x + M_PI) + 1.f) * 127.f;
				PhaseImgs[3]->at<float>(0, i * pixelPerPeriod + j) = (sin(x + 3.f * M_PI / 2.f) + 1.f) * 127.f;
			}
		}
		for (int i = 0; i < PhaseImgs.size(); ++i)
		{
			cv::Mat& img = *PhaseImgs[i].get();
			for (int i = 1; i < img.rows; ++i)
			{
				img.row(0).copyTo(img.row(i));
			}
		}
	}
	else
	{
		int numPeriod = projectorHeight / pixelPerPeriod + 1;
		for (int i = 0; i < numPeriod; i++)
		{
			for (int j = 0; j < pixelPerPeriod && (i * pixelPerPeriod + j) < projectorHeight; j++)
			{
				float x = static_cast<float>(j) / static_cast<float>(pixelPerPeriod) * 2.f * M_PI;
				PhaseImgs[0]->at<float>(i * pixelPerPeriod + j, 0) = (sin(x) + 1.f) * 127.f;
				PhaseImgs[1]->at<float>(i * pixelPerPeriod + j, 0) = (sin(x + M_PI / 2.f) + 1.f) * 127.f;
				PhaseImgs[2]->at<float>(i * pixelPerPeriod + j, 0) = (sin(x + M_PI) + 1.f) * 127.f;
				PhaseImgs[3]->at<float>(i * pixelPerPeriod + j, 0) = (sin(x + 3.f * M_PI / 2.f) + 1.f) * 127.f;
			}
		}
		for (int i = 0; i < PhaseImgs.size(); ++i)
		{
			cv::Mat& img = *PhaseImgs[i].get();
			for (int i = 1; i < img.cols; ++i)
			{
				img.col(0).copyTo(img.col(i));
			}
		}
	}

	return true;
}

bool Encoder_Gray(std::vector<std::shared_ptr<cv::Mat>>& grayCodeMats, int order, bool colBased)
{
	grayCodeMats.clear();
	if(order < 4 || order > 16)
	{
		return false;
	}

	//获取value的第n个二进制位
	auto getBit = [](int value, int n)->bool
	{
		bool res = (value >> (n)) & 0x0001;
		return res;
	};

	int stripeNumber = std::pow(2, order);

	//格雷码编码
	std::vector<std::vector<bool>> grayCode(stripeNumber, std::vector<bool>(order));
	for (int i = 0; i < grayCode.size(); ++i)
	{
		std::vector<bool>& code = grayCode[i];
		for(int j=0;j<order;++j)
		{
			code[j] = getBit(i, j);
		}
		std::vector<bool> temp = code;
		for(int j = code.size() - 2;j>=0;--j)
		{
			code[j] = temp[j] xor temp[j + 1];
		}
		/*for (int j = code.size() - 1; j >= 0; --j)
		{
			std::cout << code[j];
		}
		std::cout << std::endl;*/
	}

	//生成图像
	grayCodeMats.resize(order);
	for(int k=0;k<order;++k)
	{
		grayCodeMats[k].reset(new cv::Mat(stripeNumber, stripeNumber, CV_8UC1));
		cv::Mat& img = *grayCodeMats[k].get();
		img.setTo(0);
		if(colBased)
		{
			for (int i = 0; i < stripeNumber; ++i)
			{
				if (grayCode[i][order - k - 1])
				{
					img.at<char>(0, i) = 255;
				}
			}
			for (int i = 1; i < stripeNumber; ++i)
			{
				img.row(0).copyTo(img.row(i));
			}
		}
		else
		{
			for (int i = 0; i < stripeNumber; ++i)
			{
				if (grayCode[i][order - k - 1])
				{
					img.at<char>(i, 0) = 255;
				}
			}
			for (int i = 1; i < stripeNumber; ++i)
			{
				img.col(0).copyTo(img.col(i));
			}
		}
		/*cv::imshow("gray code mat", img);
		cv::waitKey(0);*/
	}

	return true;
}


} // namespace sr