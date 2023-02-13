#include "CalibrationData.hpp"

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

CalibrationData::CalibrationData()
    : cam_K()
    , cam_kc()
    , proj_K()
    , proj_kc()
    , R()
    , T()
    , cam_error(0.0)
    , proj_error(0.0)
    , stereo_error(0.0)
    , filename()
{
}

CalibrationData::~CalibrationData()
{
}

void CalibrationData::clear(void)
{
	cam_K = cv::Mat();
	cam_kc = cv::Mat();
	proj_K = cv::Mat();
	proj_kc = cv::Mat();
	R = cv::Mat();
	T = cv::Mat();
	filename = std::string();
}

bool CalibrationData::is_valid(void) const
{
	return (cam_K.data && cam_kc.data && proj_K.data && proj_kc.data && R.data && T.data);
}

bool CalibrationData::load_calibration(std::string const& filename)
{
	if (filename.substr(filename.size() - 3, 3) == "yml")
	{
		return load_calibration_yml(filename);
	}

	return false;
}

bool CalibrationData::save_calibration(std::string const& filename)
{
	if (filename.substr(filename.size() - 3, 3) == "yml")
	{
		return save_calibration_yml(filename);
	}

	return false;
}

bool CalibrationData::load_calibration_yml(std::string const& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		return false;
	}

	fs["cam_K"] >> cam_K;
	fs["cam_kc"] >> cam_kc;
	fs["proj_K"] >> proj_K;
	fs["proj_kc"] >> proj_kc;
	fs["R"] >> R;
	fs["T"] >> T;

	fs["cam_error"] >> cam_error;
	fs["proj_error"] >> proj_error;
	fs["stereo_error"] >> stereo_error;

	fs.release();

	this->filename = filename;

	return true;
}

bool CalibrationData::save_calibration_yml(std::string const& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		return false;
	}

	fs << "cam_K" << cam_K << "cam_kc" << cam_kc << "proj_K" << proj_K << "proj_kc" << proj_kc << "R" << R << "T" << T << "cam_error" << cam_error
	   << "proj_error" << proj_error << "stereo_error" << stereo_error;
	fs.release();

	this->filename = filename;

	return true;
}

void CalibrationData::display(std::ostream& stream) const
{
	stream << "Camera Calib: " << std::endl
	       << " - reprojection error: " << cam_error << std::endl
	       << " - K:\n"
	       << cam_K << std::endl
	       << " - kc: " << cam_kc << std::endl;
	stream << std::endl;
	stream << "Projector Calib: " << std::endl
	       << " - reprojection error: " << proj_error << std::endl
	       << " - K:\n"
	       << proj_K << std::endl
	       << " - kc: " << proj_kc << std::endl;
	stream << std::endl;
	stream << "Stereo Calib: " << std::endl
	       << " - reprojection error: " << stereo_error << std::endl
	       << " - R:\n"
	       << R << std::endl
	       << " - T:\n"
	       << T << std::endl;
}