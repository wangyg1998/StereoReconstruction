#ifndef __CALIBRATIONDATA_HPP__
#define __CALIBRATIONDATA_HPP__

#include <iostream>
#include <opencv2/core/core.hpp>

class CalibrationData
{
public:
	static const int CALIBRATION_FILE_VERSION = 1;

	CalibrationData();
	~CalibrationData();

	void clear(void);

	bool is_valid(void) const;

	bool load_calibration(std::string const& filename);
	bool save_calibration(std::string const& filename);

	bool load_calibration_yml(std::string const& filename);
	bool save_calibration_yml(std::string const& filename);

	void display(std::ostream& stream = std::cout) const;

	//data
	cv::Mat cam_K;
	cv::Mat cam_kc;
	cv::Mat proj_K;
	cv::Mat proj_kc;
	cv::Mat R;
	cv::Mat T;

	double cam_error;
	double proj_error;
	double stereo_error;

	std::string filename;
};

#endif //__CALIBRATIONDATA_HPP__