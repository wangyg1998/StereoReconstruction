#include "Encoder.h"

#include <fstream>
#include <opencv2/highgui.hpp>
#include <strstream>
namespace sr
{

bool Encoder_Gray(std::vector<std::shared_ptr<cv::Mat>>& grayCodeMats, int order, bool colBased)
{
	grayCodeMats.clear();
	if(order < 4 || order > 16)
	{
		return false;
	}

	//��ȡvalue�ĵ�n��������λ
	auto getBit = [](int value, int n)->bool
	{
		bool res = (value >> (n)) & 0x0001;
		return res;
	};

	int stripeNumber = std::pow(2, order);

	//���������
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

	//����ͼ��
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


// ���캯����
Encoder_Phase::Encoder_Phase()
{
	this->m_pixPeriod = 0;
	this->m_numMat = 0;
	this->m_PSMat = NULL;

	this->m_resRow = PROJECTOR_RESROW;
	this->m_resCol = PROJECTOR_RESCOL;
	this->m_colBased = true;

	this->m_filePath = "/";
	this->m_matName = "Phase";
	this->m_matEnd = ".bmp";
}

// ����������ɾ���������ؿռ䡣
Encoder_Phase::~Encoder_Phase()
{
	// ɾ��m_PSMat
	if (this->m_PSMat != NULL)
	{
		delete[] (this->m_PSMat);
		this->m_PSMat = NULL;
	}
}

// ����PS���ݣ�����ͼ��
bool Encoder_Phase::DrawMat()
{
	using namespace cv;

	// ����ͼ��
	this->m_numMat = 4;
	this->m_PSMat = new Mat[this->m_numMat];
	for (int i = 0; i < this->m_numMat; i++)
	{
		// this->m_PSMat[i].create(this->m_resRow, this->m_resLine, CV_8UC1);
		this->m_PSMat[i].create(this->m_resRow, this->m_resCol, CV_32FC1);
	}

	// ����ͼ��
	if (this->m_colBased)
	{
		// ���������������������ڻ���
		int numPeriod = this->m_resCol / this->m_pixPeriod;
		for (int T = 0; T < numPeriod; T++)
		{
			for (int pix = 0; pix < this->m_pixPeriod; pix++)
			{
				// ��pixӳ�䵽 [0, 2PI)
				double x = (double)pix / (double)(this->m_pixPeriod) * 2 * CV_PI;
				// ���Ϊsin(x), sin(x+pi/3), sin(x-pi/3);
				// ���Ҳ���ÿ����λ��λpi/2
				for (int r = 0; r < this->m_resRow; r++)
				{
					this->m_PSMat[0].at<float>(r, pix + T * this->m_pixPeriod) = (sin(x) + 1) * 127.0;
					this->m_PSMat[1].at<float>(r, pix + T * this->m_pixPeriod) = (sin(x + CV_PI / 2) + 1) * 127.0;
					this->m_PSMat[2].at<float>(r, pix + T * this->m_pixPeriod) = (sin(x + CV_PI) + 1) * 127.0;
					this->m_PSMat[3].at<float>(r, pix + T * this->m_pixPeriod) = (sin(x + 3 * CV_PI / 2) + 1) * 127.0;
				}
			}
		}
	}
	else
	{
		// ���������������������ڻ���
		int numPeriod = this->m_resRow / this->m_pixPeriod;
		for (int T = 0; T < numPeriod; T++)
		{
			for (int pix = 0; pix < this->m_pixPeriod; pix++)
			{
				// ��pixӳ�䵽��0, 2PI��
				double x = (double)pix / (double)(this->m_pixPeriod) * 2 * CV_PI;
				// ���Ϊsin(x), sin(x+pi/3), sin(x-pi/3);
				for (int l = 0; l < this->m_resCol; l++)
				{
					this->m_PSMat[0].at<float>(pix + T * this->m_pixPeriod, l) = (sin(x) + 1) * 127.0;
					this->m_PSMat[1].at<float>(pix + T * this->m_pixPeriod, l) = (sin(x + CV_PI / 2) + 1) * 127.0;
					this->m_PSMat[2].at<float>(pix + T * this->m_pixPeriod, l) = (sin(x + CV_PI) + 1) * 127.0;
					this->m_PSMat[3].at<float>(pix + T * this->m_pixPeriod, l) = (sin(x + 3 * CV_PI / 2) + 1) * 127.0;
				}
			}
		}
	}

	return true;
}

// ������ļ�
bool Encoder_Phase::WriteData()
{
	using namespace cv;

	for (int i = 0; i < this->m_numMat; i++)
	{
		std::string tempNum;
		std::strstream ss;
		ss << m_numMat - i - 1;
		ss >> tempNum;

		std::string tempPath = this->m_filePath;
		for (int i = 0; i < tempPath.length(); i++)
		{
			if (tempPath[i] == '/')
				tempPath[i] = '\\';
		}
		system((std::string("mkdir ") + tempPath).c_str());

		cv::imwrite(this->m_filePath + this->m_matName + tempNum + this->m_matEnd, this->m_PSMat[i]);
	}

	return true;
}

// ��ʼ����PhaseShifting����Ҫ����pixel���ڣ��Լ����Ʒ���
bool Encoder_Phase::Encode(int pixPeriod, bool colBased)
{
	// �жϲ����Ƿ�Ϸ��������Ρ�
	if (pixPeriod <= 0)
		return false;
	this->m_pixPeriod = pixPeriod;
	this->m_colBased = colBased; //true�������� false��������

	// ����
	if (!this->DrawMat())
		return false;

	// ����
	if (!this->WriteData())
		return false;

	return true;
}

// �趨�洢��λ��ͼ�����ļ���
bool Encoder_Phase::SetMatFileName(std::string filePath, std::string matName, std::string matEnd)
{
	this->m_filePath = filePath;
	this->m_matName = matName;
	this->m_matEnd = matEnd;
	return true;
}

// ���ӻ�
void Encoder_Phase::Visualization()
{
	using namespace cv;

#ifdef VISUAL
	namedWindow(this->m_matName);
	for (int i = 0; i < this->m_numMat; i++)
	{
		std::cout << "Now present: " << i << std::endl;
		imshow(this->m_matName, this->m_PSMat[i]);
		cv::waitKey(400);
	}
	destroyWindow(this->m_matName);
#endif
}

} // namespace sr