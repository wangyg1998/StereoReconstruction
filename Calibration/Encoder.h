#pragma once
#include <opencv2/opencv.hpp>

//投影仪分辨率
static int PROJECTOR_RESCOL = 1920;
static int PROJECTOR_RESROW = 1080;

namespace sr
{

	/// \param order[in] 格雷码阶次
	/// \param grayCodeMats[out] 编码结果,图像Size为(2^order, 2^order)
	/// \param colBased[in] true:列条纹 false:行条纹
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
