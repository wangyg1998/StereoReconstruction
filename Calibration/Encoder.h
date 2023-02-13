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


// PhaseShifting����
class Encoder_Phase
{
private:
	int m_numMat; // Mat��Ŀ
	int m_pixPeriod; // ÿ���ڵ�pix��Ŀ
	cv::Mat *m_PSMat; // phaseshifting��Ӧ��ͼ��

	int m_resRow; // ͼ����зֱ���
	int m_resCol; // ͼ����зֱ���
	bool m_colBased; // �Ƿ�����������

	std::string m_filePath; // �洢·����
	std::string m_matName; // ͼ����
	std::string m_matEnd; // ͼ���׺��

	bool DrawMat(); // ����PS���ݣ�����ͼ��
	bool WriteData(); // ������ļ�

public:
	Encoder_Phase();
	~Encoder_Phase();
	bool Encode(int pixPeriod, bool colBased);
	bool SetMatFileName(std::string filePath, std::string matName, std::string matEnd);
	void Visualization();
};


} // namespace sr
