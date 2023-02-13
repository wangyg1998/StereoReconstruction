#include "Reconstruction.h"

namespace sr
{

// 包裹相位计算（相对相位）
// 输入：极线校正以后的调制图像
cv::Mat CalWrappedPhase(std::vector<std::shared_ptr<cv::Mat>> images)
{
	//cv::Mat img1 = cv::imread(imagelist[6], CV_LOAD_IMAGE_GRAYSCALE);

	cv::Mat wrapped_phase(images.front()->size(), CV_32FC1, cv::Scalar(0.0));
	for (int i = 0; i < images.front()->size().height; i++)
	{
		uchar *img1_data = images[0]->ptr<uchar>(i);
		uchar *img2_data = images[0]->ptr<uchar>(i);
		uchar *img3_data = images[0]->ptr<uchar>(i);
		uchar *img4_data = images[0]->ptr<uchar>(i);
		float *wrapped_phase_data = wrapped_phase.ptr<float>(i);

		for (int j = 0; j < images.front()->size().width; j++)
		{
			*wrapped_phase_data++ = atan2(((float)(*img4_data++) - (float)(*img2_data++)), ((float)(*img1_data++) - (float)(*img3_data++)));
		}
	}

	return wrapped_phase;
}


/***********************************************************************
基于格雷码的展开相位计算，得到表面采样点周期次数k，绝对相位（展开相位）
输入：
	src: 包裹相位（相对相位）
	dst：展开相位（绝对相位）
	Rect_images: 极线校正（立体校正）后的图像
***********************************************************************/
void UnwrappedPhaseGraycodeMethod(cv::Mat &wrapped_phase, cv::Mat &unwrapped_phase, std::vector<std::shared_ptr<cv::Mat>> images)
{

	// 读取6张格雷码调制图像
	//Mat img1 = imread(imagelist[0], CV_LOAD_IMAGE_GRAYSCALE);

	// 相位序列Mat
	cv::Mat phase_series(cv::Size(images.front()->size()), CV_8UC1, cv::Scalar(0.0));

	// 二值化阈值
	uchar thresh = 130;
	for(int i=0;i<images.size();++i)
	{
		cv::threshold(*images[i].get(), *images[i].get(), thresh,255, CV_THRESH_BINARY);
	}
	for(int i=0;i<images.size() - 1;++i)
	{
		cv::bitwise_xor(*images[i].get(), *images[i + 1].get(), *images[i + 1].get());
	}

	int width = images.front()->cols;
	int height = images.front()->rows;
	uchar pre_series, cur_series;
	float pre_unphase, cur_unphase;
	for (int y = 0; y < height; y++)
	{
		uchar *img1_ptr = images[0]->ptr<uchar>(y);
		uchar *img2_ptr = images[1]->ptr<uchar>(y);
		uchar *img3_ptr = images[2]->ptr<uchar>(y);
		uchar *img4_ptr = images[3]->ptr<uchar>(y);
		uchar *img5_ptr = images[4]->ptr<uchar>(y);
		uchar *img6_ptr = images[5]->ptr<uchar>(y);

		for (int x = 0; x < width; x++)
		{
			phase_series.at<uchar>(y, x) = ((int)(*img1_ptr++)) * 32 + ((int)(*img2_ptr++)) * 16 + ((int)(*img3_ptr++)) * 8 + ((int)(*img4_ptr++)) * 4 +
			                               ((int)(*img5_ptr++)) * 2 + ((int)(*img6_ptr++)) * 1;
		}
	}

	medianBlur(phase_series, phase_series, 9); //中值滤波

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			//绝对相位 = 2*PI*k + θ(x,y)（相对相位/相位主体）
			unwrapped_phase.at<float>(y, x) = phase_series.at<uchar>(y, x) * 2 * CV_PI + wrapped_phase.at<float>(y, x);
		}
	}
}


void find_featurepionts_single_match(cv::Mat &leftphase, cv::Mat &rightphase, std::vector<cv::Point2f> &leftkeypoint, std::vector<cv::Point2f> &rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	int x, y, k;
	float left;
	cv::Point2f fleft, fright;
	float PHASE_THRESHOLD = 0.01;

	for (y = 0; y < nr; y += 1)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);

		for (x = 0; x < nc; x++)
		{
			left = *left_phase_data++;

			if (left > 2 * CV_PI)
			{
				right_phase_data = rightphase.ptr<float>(y);
				k = 0;

				while ((abs(left - *right_phase_data++) > PHASE_THRESHOLD) && (k < nc))
				{
					k++;
				}
				if (k < nc)
				{
					fleft.x = x;
					fleft.y = y;
					fright.x = k;
					fright.y = y;
					leftkeypoint.push_back(fleft);
					rightkeypoint.push_back(fright);
				}
			}
		}
	}
}


} // namespace sr
