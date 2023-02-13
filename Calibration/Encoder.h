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


// PhaseShifting生成
class Encoder_Phase
{
private:
	int m_numMat; // Mat数目
	int m_pixPeriod; // 每周期的pix数目
	cv::Mat *m_PSMat; // phaseshifting对应的图像

	int m_resRow; // 图像的行分辨率
	int m_resCol; // 图像的列分辨率
	bool m_colBased; // 是否按照列来绘制

	std::string m_filePath; // 存储路径名
	std::string m_matName; // 图像名
	std::string m_matEnd; // 图像后缀名

	bool DrawMat(); // 根据PS内容，绘制图像
	bool WriteData(); // 输出到文件

public:
	Encoder_Phase();
	~Encoder_Phase();
	bool Encode(int pixPeriod, bool colBased);
	bool SetMatFileName(std::string filePath, std::string matName, std::string matEnd);
	void Visualization();
};


} // namespace sr
