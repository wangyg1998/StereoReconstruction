#pragma once
#include <opencv2/opencv.hpp>

//ͶӰ�Ƿֱ���
static int PROJECTOR_RESCOL = 1920;
static int PROJECTOR_RESROW = 1080;

namespace sr
{

	/// \param order[in] ������״�
	/// \param grayCodeMats[out] ������,ͼ��SizeΪ(2^order, 2^order)
	/// \param colBased[in] true:������ false:������
bool Encoder_Gray(std::vector<std::shared_ptr<cv::Mat>> &grayCodeMats, int order, bool colBased = true);

bool getGrayCode(std::vector<std::vector<bool>> &grayCode, int order);

bool Encoder_GrayCodeImg(std::vector<std::shared_ptr<cv::Mat>> &grayCodeImgs,
                         int projectorWidth,
                         int projectorHeight,
                         int pixelPerPeriod,
                         int order,
                         bool colBased = true);

bool Encoder_PhaseImg(std::vector<std::shared_ptr<cv::Mat>> &PhaseImgImgs, int projectorWidth, int projectorHeight, int pixelPerPeriod, bool colBased = true);




} // namespace sr
