#include "Calibration.h"

#include "CalibrationData.hpp"
#include "structured_light.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
namespace sr
{

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                 const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                 const std::vector<cv::Mat>& rvecs,
                                 const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs)
{
	std::vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	std::vector<float> perViewErrors;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i)
	{
		cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}

void camera_intrinsic(std::vector<std::shared_ptr<cv::Mat>> images, int corners_per_row, int corners_per_col, float square_size, std::string file_name)
{
	cv::Size board_size = cv::Size(corners_per_row, corners_per_col);
	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> image_points;
	cv::Size img_size = images.front()->size();
	for (int k = 0; k < images.size(); k++)
	{
		cv::Mat &img = *images[k].get(), gray;
		cv::cvtColor(img, gray, CV_BGR2GRAY);
		bool found = false;
		std::vector<cv::Point2f> corners;
		found = cv::findChessboardCorners(img, board_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found)
		{
			cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray, board_size, corners, found);
			//cv::imshow("corners", gray);
			//cv::waitKey(0);
		}

		std::vector<cv::Point3f> obj;
		for (int i = 0; i < corners_per_row; i++)
		{
			for (int j = 0; j < corners_per_col; j++)
			{
				obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));
			}
		}

		if (found)
		{
			std::cout << k << ". Found corners!" << std::endl;
			image_points.push_back(corners);
			object_points.push_back(obj);
		}
	}

	cv::Mat K;
	cv::Mat D;
	std::vector<cv::Mat> rvecs, tvecs;
	int flag = 0;
	flag |= CV_CALIB_FIX_K4;
	flag |= CV_CALIB_FIX_K5;
	calibrateCamera(object_points, image_points, img_size, K, D, rvecs, tvecs, flag);

	std::cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << std::endl;

	cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
	fs << "K" << K;
	fs << "D" << D;
	fs << "corners_per_row" << corners_per_row;
	fs << "corners_per_col" << corners_per_col;
	fs << "square_size" << square_size;
	printf("Done Calibration\n");
}

void calib_extrinisic(std::string left_intrinsic_file,
                      std::string right_intrinsic_file,
                      std::string calib_file,
                      std::vector<std::shared_ptr<cv::Mat>> left_images,
                      std::vector<std::shared_ptr<cv::Mat>> right_images)
{
	//加载内参
	cv::FileStorage fsl(left_intrinsic_file, cv::FileStorage::READ);
	cv::FileStorage fsr(right_intrinsic_file, cv::FileStorage::READ);
	cv::Size board_size = cv::Size(fsl["board_width"], fsl["board_height"]);
	float square_size = fsl["square_size"];
	cv::Mat K1, K2;
	cv::Mat D1, D2;
	fsl["K"] >> K1;
	fsr["K"] >> K2;
	fsl["D"] >> D1;
	fsr["D"] >> D2;

	//检测角点
	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
	std::vector<std::vector<cv::Point2f>> left_img_points, right_img_points;
	for (int i = 0; i < left_images.size(); ++i)
	{
		cv::Mat gray1, gray2;
		cvtColor(*left_images[i].get(), gray1, CV_BGR2GRAY);
		cvtColor(*right_images[i].get(), gray2, CV_BGR2GRAY);

		bool found1 = false, found2 = false;
		std::vector<cv::Point2f> corners1, corners2;
		found1 = cv::findChessboardCorners(*left_images[i].get(), board_size, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		found2 = cv::findChessboardCorners(*right_images[i].get(), board_size, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if (!found1 || !found2)
		{
			std::cout << "Chessboard find error!" << std::endl;
			continue;
		}
		if (found1)
		{
			cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			//cv::drawChessboardCorners(gray1, board_size, corners1, found1);
		}
		if (found2)
		{
			cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			//cv::drawChessboardCorners(gray2, board_size, corners2, found2);
		}

		std::vector<cv::Point3f> obj;
		for (int i = 0; i < board_size.height; i++)
			for (int j = 0; j < board_size.width; j++)
				obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));

		if (found1 && found2)
		{
			std::cout << i << ". Found corners!" << std::endl;
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);
			object_points.push_back(obj);
		}
	}
	for (int i = 0; i < imagePoints1.size(); i++)
	{
		std::vector<cv::Point2f> v1, v2;
		for (int j = 0; j < imagePoints1[i].size(); j++)
		{
			v1.push_back(cv::Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
			v2.push_back(cv::Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
		}
		left_img_points.push_back(v1);
		right_img_points.push_back(v2);
	}

	//求解外参
	cv::Mat R, E, F;
	cv::Vec3d T;
	cv::stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2, left_images.front()->size(), R, T, E, F);
	cv::FileStorage fs1(calib_file, cv::FileStorage::WRITE);
	fs1 << "K1" << K1;
	fs1 << "K2" << K2;
	fs1 << "D1" << D1;
	fs1 << "D2" << D2;
	fs1 << "R" << R;
	fs1 << "T" << T;
	fs1 << "E" << E;
	fs1 << "F" << F;

	//求解矫正参数
	cv::Mat R1, R2, P1, P2, Q;
	cv::stereoRectify(K1, D1, K2, D2, left_images.front()->size(), R, T, R1, R2, P1, P2, Q);
	fs1 << "R1" << R1;
	fs1 << "R2" << R2;
	fs1 << "P1" << P1;
	fs1 << "P2" << P2;
	fs1 << "Q" << Q;
}

void image_rectify(std::string calib_file, const cv::Mat* left_image, const cv::Mat* right_image, cv::Mat* rectified_left, cv::Mat* rectified_right)
{
	//加载标定文件
	cv::Mat R1, R2, P1, P2, Q;
	cv::Mat K1, K2, R;
	cv::Vec3d T;
	cv::Mat D1, D2;
	cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
	fs1["K1"] >> K1;
	fs1["K2"] >> K2;
	fs1["D1"] >> D1;
	fs1["D2"] >> D2;
	fs1["R"] >> R;
	fs1["T"] >> T;

	fs1["R1"] >> R1;
	fs1["R2"] >> R2;
	fs1["P1"] >> P1;
	fs1["P2"] >> P2;
	fs1["Q"] >> Q;

	//矫正
	cv::Mat lmapx, lmapy, rmapx, rmapy;
	cv::initUndistortRectifyMap(K1, D1, R1, P1, left_image->size(), CV_32F, lmapx, lmapy);
	cv::initUndistortRectifyMap(K2, D2, R2, P2, right_image->size(), CV_32F, rmapx, rmapy);
	cv::remap(*left_image, *rectified_left, lmapx, lmapy, cv::INTER_LINEAR);
	cv::remap(*right_image, *rectified_right, rmapx, rmapy, cv::INTER_LINEAR);
}

static void get_chessboard_world_coords(std::vector<cv::Point3f>& world_corners, cv::Size corner_count, cv::Size corner_size)
{
	//generate world object coordinates
	for (int h = 0; h < corner_count.height; h++)
	{
		for (int w = 0; w < corner_count.width; w++)
		{
			world_corners.push_back(cv::Point3f(corner_size.width * w, corner_size.height * h, 0.f));
		}
	}
}

static bool extract_chessboard_corners(const cv::Mat& gray_image,
                                       cv::Size border_size,
                                       cv::Size2f square_size,
                                       std::vector<cv::Point2f>& cam_corners,
                                       std::vector<cv::Point3f>& world_corners)
{
	//this will be filled by the detected corners
	cam_corners.clear();
	world_corners.clear();

	cv::Mat small_img;
	int image_scale = cvRound(gray_image.size().width / 1024.0);
	if (image_scale > 1)
	{
		cv::resize(gray_image, small_img, cv::Size(gray_image.cols / image_scale, gray_image.rows / image_scale));
	}
	else
	{
		gray_image.copyTo(small_img);
	}

	if (cv::findChessboardCorners(
	        small_img, border_size, cam_corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE /*+ cv::CALIB_CB_FILTER_QUADS*/))
	{
		get_chessboard_world_coords(world_corners, border_size, square_size);
	}
	else
	{
		return false;
	}
	for (std::vector<cv::Point2f>::iterator iter = cam_corners.begin(); iter != cam_corners.end(); iter++)
	{
		*iter = image_scale * (*iter);
	}
	if (cam_corners.size())
	{
		cv::cornerSubPix(gray_image, cam_corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	}
	return true;
}

static bool decode_gray_set(std::vector<std::shared_ptr<cv::Mat>> images,
                            float decode_param_b,
                            int decode_param_m,
                            cv::Mat& pattern_image,
                            cv::Mat& min_max_image)
{
	pattern_image = cv::Mat();
	min_max_image = cv::Mat();

	const float b = decode_param_b;
	const unsigned m = decode_param_m;

	//estimate direct component
	int total_images = images.size();
	int total_patterns = total_images / 2 - 1;
	const int direct_light_count = 4;
	const int direct_light_offset = 4;
	if (total_patterns < direct_light_count + direct_light_offset)
	{ //too few images
		return false;
	}
	std::vector<std::shared_ptr<cv::Mat>> image_for_light;
	for (unsigned i = 0; i < direct_light_count; i++)
	{
		int index = total_images - total_patterns - direct_light_count - direct_light_offset + i + 1;
		image_for_light.push_back(images[index]);
		image_for_light.push_back(images[index + total_patterns]);
	}

	//QList<unsigned> direct_component_images(QList<unsigned>() << 15 << 16 << 17 << 18 << 35 << 36 << 37 << 38);
	cv::Mat direct_light = sl::estimate_direct_light(images, b);

	cv::Size projector_size(1920, 1080);
	bool rv = sl::decode_pattern(images, pattern_image, min_max_image, projector_size, sl::RobustDecode | sl::GrayPatternDecode, direct_light, m);
	return rv;
}

bool projector_Camera_Calibration(std::vector<std::vector<std::shared_ptr<cv::Mat>>> images_list,
                                  int corners_per_row,
                                  int corners_per_col,
                                  float square_edge_length,
                                  int projector_width,
                                  int projector_height,
                                  int shadow_threshold,
                                  int homography_window_size,
                                  float decode_param_b,
                                  int decode_param_m)
{
	CalibrationData calib;
	std::vector<cv::Mat> pattern_list;
	std::vector<cv::Mat> min_max_list;
	std::vector<std::vector<cv::Point3f>> corners_world;
	std::vector<std::vector<cv::Point2f>> corners_camera;
	std::vector<std::vector<cv::Point2f>> corners_projector;

	calib.clear();
	int count = images_list.size();
	const int threshold = shadow_threshold;
	cv::Size image_size = images_list.front().front()->size();
	cv::Size border_size(corners_per_row, corners_per_col);
	cv::Size2f square_size(square_edge_length, square_edge_length);

	//detect corners ////////////////////////////////////
	corners_camera.clear();
	corners_world.clear();
	corners_camera.resize(count);
	corners_world.resize(count);
	for (int i = 0; i < count; ++i)
	{
		if (!extract_chessboard_corners(*(images_list[i].front().get()), border_size, square_size, corners_camera[i], corners_world[i]))
		{
			return false;
		}
	}

	//collect projector correspondences
	corners_projector.clear();
	pattern_list.clear();
	min_max_list.clear();
	corners_projector.resize(count);
	pattern_list.resize(count);
	min_max_list.resize(count);

	//Decoding and computing homographies...
	for (int i = 0; i < count; i++)
	{
		std::vector<cv::Point2f> const& cam_corners = corners_camera[i];
		std::vector<cv::Point2f>& proj_corners = corners_projector[i];
		proj_corners.clear(); //erase previous points
		cv::Mat& pattern_image = pattern_list[i];
		cv::Mat& min_max_image = min_max_list[i];

		//Decoding
		if (!decode_gray_set(images_list[i], decode_param_b, decode_param_m, pattern_image, min_max_image))
		{ //error
			std::cout << "ERROR: Decode image set " << i << " failed. " << std::endl;
			return false;
		}
		if (image_size != pattern_image.size())
		{
			std::cout << "ERROR: pattern image of different size: set " << i << std::endl;
			return false;
		}

		//Computing homographies
		for (std::vector<cv::Point2f>::const_iterator iter = cam_corners.cbegin(); iter != cam_corners.cend(); iter++)
		{
			const cv::Point2f& p = *iter;
			cv::Point2f q;

			//find an homography around p
			unsigned WINDOW_SIZE = homography_window_size / 2;
			std::vector<cv::Point2f> img_points, proj_points;
			if (p.x > WINDOW_SIZE && p.y > WINDOW_SIZE && p.x + WINDOW_SIZE < pattern_image.cols && p.y + WINDOW_SIZE < pattern_image.rows)
			{
				for (unsigned h = p.y - WINDOW_SIZE; h < p.y + WINDOW_SIZE; h++)
				{
					register const cv::Vec2f* row = pattern_image.ptr<cv::Vec2f>(h);
					register const cv::Vec2b* min_max_row = min_max_image.ptr<cv::Vec2b>(h);
					//cv::Vec2f * out_row = out_pattern_image.ptr<cv::Vec2f>(h);
					for (unsigned w = p.x - WINDOW_SIZE; w < p.x + WINDOW_SIZE; w++)
					{
						const cv::Vec2f& pattern = row[w];
						const cv::Vec2b& min_max = min_max_row[w];
						//cv::Vec2f & out_pattern = out_row[w];
						if (sl::INVALID(pattern))
						{
							continue;
						}
						if ((min_max[1] - min_max[0]) < static_cast<int>(threshold))
						{ //apply threshold and skip
							continue;
						}

						img_points.push_back(cv::Point2f(w, h));
						proj_points.push_back(cv::Point2f(pattern));

						//out_pattern = pattern;
					}
				}
				cv::Mat H = cv::findHomography(img_points, proj_points, CV_RANSAC);
				//std::cout << " H:\n" << H << std::endl;
				cv::Point3d Q = cv::Point3d(cv::Mat(H * cv::Mat(cv::Point3d(p.x, p.y, 1.0))));
				q = cv::Point2f(Q.x / Q.z, Q.y / Q.z);
			}
			else
			{
				return false;
			}

			//save
			proj_corners.push_back(q);
		}
	}

	std::vector<std::vector<cv::Point3f>> world_corners_active;
	std::vector<std::vector<cv::Point2f>> camera_corners_active;
	std::vector<std::vector<cv::Point2f>> projector_corners_active;
	world_corners_active.reserve(count);
	camera_corners_active.reserve(count);
	projector_corners_active.reserve(count);
	for (unsigned i = 0; i < count; i++)
	{
		std::vector<cv::Point3f> const& world_corners = corners_world.at(i);
		std::vector<cv::Point2f> const& cam_corners = corners_camera.at(i);
		std::vector<cv::Point2f> const& proj_corners = corners_projector.at(i);
		if (world_corners.size() && cam_corners.size() && proj_corners.size())
		{ //active set
			world_corners_active.push_back(world_corners);
			camera_corners_active.push_back(cam_corners);
			projector_corners_active.push_back(proj_corners);
		}
	}
	if (world_corners_active.size() < 3)
	{
		std::cout << "ERROR: use at least 3 sets" << std::endl;
		return false;
	}

	int cal_flags = 0
	                //+ cv::CALIB_FIX_K1
	                //+ cv::CALIB_FIX_K2
	                //+ cv::CALIB_ZERO_TANGENT_DIST
	                + cv::CALIB_FIX_K3;

	//calibrate the camera ////////////////////////////////////
	std::vector<cv::Mat> cam_rvecs, cam_tvecs;
	int cam_flags = cal_flags;
	calib.cam_error = cv::calibrateCamera(world_corners_active,
	                                      camera_corners_active,
	                                      image_size,
	                                      calib.cam_K,
	                                      calib.cam_kc,
	                                      cam_rvecs,
	                                      cam_tvecs,
	                                      cam_flags,
	                                      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

	//calibrate the projector ////////////////////////////////////
	cv::Size projector_size(projector_width, projector_height);
	std::vector<cv::Mat> proj_rvecs, proj_tvecs;
	int proj_flags = cal_flags;
	calib.proj_error = cv::calibrateCamera(world_corners_active,
	                                       projector_corners_active,
	                                       projector_size,
	                                       calib.proj_K,
	                                       calib.proj_kc,
	                                       proj_rvecs,
	                                       proj_tvecs,
	                                       proj_flags,
	                                       cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));
	/*
    //TMP: estimate an initial stereo R and T
    double errStereo = 0.0;
    std::vector<cv::Point3f> wpts;
    std::vector<cv::Point2f> cipts, pipts;
    for (size_t i=0; i<world_corners_active.size(); ++i)
    {
      auto const& world_curr = world_corners_active.at(i);
      auto const& cam_curr = camera_corners_active.at(i);
      auto const& proj_curr = projector_corners_active.at(i);
      cv::Matx33d Rc; cv::Rodrigues(cam_rvecs.at(i), Rc); cv::Point3d Tc = cam_tvecs.at(i);
      cv::Matx33d Rp; cv::Rodrigues(proj_rvecs.at(i), Rp); cv::Point3d Tp = proj_tvecs.at(i);
      for (size_t j=0; j<world_curr.size(); ++j)
      {
        auto const& wpt = world_curr.at(j);

        cv::Matx31d wptc = Rc*cv::Point3d(wpt) + Tc;
        wpts.push_back( cv::Point3f(wptc(0,0),wptc(1,0),wptc(2,0)) );

        cipts.push_back(cam_curr.at(j));
        pipts.push_back(proj_curr.at(j));
      }
    }

    //cam ransac
    cv::Vec3d cam_rvec, cam_tvec;
    cv::solvePnPRansac(wpts, cipts, calib.cam_K, calib.cam_kc, cam_rvec, cam_tvec);
    std::cerr << "Cam (ransac) R,T: R " << cam_rvec << " T " << cam_tvec << std::endl;

    { //reproj error
      double errCam = 0.0;
      std::vector<cv::Point2f> imgPts;
      cv::projectPoints(wpts, cam_rvec, cam_tvec, calib.cam_K, calib.cam_kc, imgPts);
      for (size_t i=0; i<imgPts.size(); ++i)
      {
        auto const& p1 = imgPts.at(i);
        auto const& p2 = cipts.at(i);
        double err = cv::norm(p1-p2);
        errCam += err;
      }
      errCam /= imgPts.size();
      std::cerr << " errCam (ransac):  " << errCam << std::endl;
    }

    //cam LM
    bool rv1 = cv::solvePnP(wpts, cipts, calib.cam_K, calib.cam_kc, cam_rvec, cam_tvec, true);
    std::cerr << "Cam (LM) R,T: R " << cam_rvec << " T " << cam_tvec << std::endl;

    { //reproj error
      double errCam = 0.0;
      std::vector<cv::Point2f> imgPts;
      cv::projectPoints(wpts, cam_rvec, cam_tvec, calib.cam_K, calib.cam_kc, imgPts);
      for (size_t i=0; i<imgPts.size(); ++i)
      {
        auto const& p1 = imgPts.at(i);
        auto const& p2 = cipts.at(i);
        double err = cv::norm(p1-p2);
        errCam += err;
        errStereo += err;
      }
      errCam /= imgPts.size();
      std::cerr << " errCam (LM):  " << errCam << std::endl;
    }

    //proj ransac
    cv::Vec3d proj_rvec, proj_tvec;
    cv::solvePnPRansac(wpts, pipts, calib.proj_K, calib.proj_kc, proj_rvec, proj_tvec);
    std::cerr << "Proj (ransac) R,T: R " << proj_rvec << " T " << proj_tvec << std::endl;

    { //reproj error
      double errPrj = 0.0;
      std::vector<cv::Point2f> projPts;
      cv::projectPoints(wpts, proj_rvec, proj_tvec, calib.proj_K, calib.proj_kc, projPts);
      for (size_t i=0; i<projPts.size(); ++i)
      {
        auto const& p1 = projPts.at(i);
        auto const& p2 = pipts.at(i);
        double err = cv::norm(p1-p2);
        errPrj += err;
      }
      errPrj /= projPts.size();
      std::cerr << " errPrj (ransac):  " << errPrj << std::endl;
    }

    //proj LM
    bool rv2 = cv::solvePnP(wpts, pipts, calib.proj_K, calib.proj_kc, proj_rvec, proj_tvec, true);
    std::cerr << "Proj (LM) R,T: R " << proj_rvec << " T " << proj_tvec << std::endl;

    { //reproj error
      double errPrj = 0.0;
      std::vector<cv::Point2f> projPts;
      cv::projectPoints(wpts, proj_rvec, proj_tvec, calib.proj_K, calib.proj_kc, projPts);
      for (size_t i=0; i<projPts.size(); ++i)
      {
        auto const& p1 = projPts.at(i);
        auto const& p2 = pipts.at(i);
        double err = cv::norm(p1-p2);
        errPrj += err;
        errStereo += err;
      }
      errPrj /= projPts.size();
      std::cerr << " errPrj (LM):  " << errPrj << std::endl;
    }

    errStereo /= 2*wpts.size();;
    std::cerr << " errStereo:  " << errStereo << std::endl;
*/

	cv::Mat E, F;
	calib.stereo_error = cv::stereoCalibrate(world_corners_active,
	                                         camera_corners_active,
	                                         projector_corners_active,
	                                         calib.cam_K,
	                                         calib.cam_kc,
	                                         calib.proj_K,
	                                         calib.proj_kc,
	                                         image_size,
	                                         calib.R,
	                                         calib.T,
	                                         E,
	                                         F,
	                                         cv::CALIB_FIX_INTRINSIC /*cv::CALIB_USE_INTRINSIC_GUESS*/ + cal_flags,
	                                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 150, DBL_EPSILON));
	//print to console
	calib.display();
	//save to file
	calib.save_calibration("D:/calib_file.yml");

	return true;
}

} // namespace sr