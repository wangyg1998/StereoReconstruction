#include "Reconstruction.h"

#include <trimesh.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "../Calibration/CalibrationData.hpp"
namespace sr
{

// 包裹相位计算（相对相位）
// 输入：极线校正以后的调制图像
cv::Mat CalWrappedPhase(std::vector<std::shared_ptr<cv::Mat>> images)
{
	cv::Mat wrapped_phase(images.front()->size(), CV_32FC1, cv::Scalar(0.0));

	for (int i = 0; i < images.front()->rows; i++)
	{
		uchar *img1_data = images[0]->ptr<uchar>(i);
		uchar *img2_data = images[1]->ptr<uchar>(i);
		uchar *img3_data = images[2]->ptr<uchar>(i);
		uchar *img4_data = images[3]->ptr<uchar>(i);
		float *wrapped_phase_data = wrapped_phase.ptr<float>(i);

		for (int j = 0; j < images.front()->cols; j++)
		{
			*wrapped_phase_data++ = atan2(((float)(*img4_data++) - (float)(*img2_data++)), ((float)(*img1_data++) - (float)(*img3_data++)));
		}
	}

	if (false)
	{
		for (int i = 0; i < wrapped_phase.cols; ++i)
		{
			std::cout << wrapped_phase.at<float>(0, i) << ", ";
		}
		std::cout << std::endl;
	}

	return wrapped_phase;
}

void decodeGrayCode(std::vector<std::shared_ptr<cv::Mat>> images, cv::Mat &result)
{
	result = cv::Mat(cv::Size(images.front()->size()), CV_32FC1, cv::Scalar(0.0));
	// 二值化阈值
	uchar thresh = 130;
	for (int i = 0; i < images.size(); ++i)
	{
		cv::threshold(*images[i].get(), *images[i].get(), thresh, 1, CV_THRESH_BINARY);
	}

	//格雷码转二进制码 images[0]是高位
	for (int i = 0; i < images.size() - 1; ++i)
	{
		cv::bitwise_xor(*images[i].get(), *images[i + 1].get(), *images[i + 1].get());
	}

	//二进制码转十进制数
	std::vector<float> scale(images.size());
	for (int i = 0; i < scale.size(); ++i)
	{
		scale[i] = std::powf(2.f, static_cast<float>(i));
	}
	std::reverse(scale.begin(), scale.end());
	for (int y = 0; y < images.front()->rows; y++)
	{
		for (int x = 0; x < images.front()->cols; x++)
		{
			for (int j = 0; j < images.size(); ++j)
			{
				result.at<float>(y, x) += static_cast<float>(images[j]->at<uchar>(y, x)) * scale[j];
			}
		}
	}

	if (true)
	{
		for (int i = 0; i < result.cols; ++i)
		{
			std::cout << result.at<float>(result.rows / 2, i) << ", ";
		}
		std::cout << std::endl << std::endl;
	}
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
	unwrapped_phase = cv::Mat(cv::Size(images.front()->size()), CV_32FC1, cv::Scalar(0.0));
	// 相位序列Mat
	cv::Mat phase_series(cv::Size(images.front()->size()), CV_32FC1, cv::Scalar(0.0));

	// 二值化阈值
	uchar thresh = 130;
	for (int i = 0; i < images.size(); ++i)
	{
		cv::threshold(*images[i].get(), *images[i].get(), thresh, 1, CV_THRESH_BINARY);
	}

	//格雷码转二进制码 images[0]是高位
	for (int i = 0; i < images.size() - 1; ++i)
	{
		cv::bitwise_xor(*images[i].get(), *images[i + 1].get(), *images[i + 1].get());
	}

	//二进制码转十进制数
	std::vector<float> scale(images.size());
	for (int i = 0; i < scale.size(); ++i)
	{
		scale[i] = std::powf(2.f, static_cast<float>(i));
	}
	std::reverse(scale.begin(), scale.end());
	for (int y = 0; y < images.front()->rows; y++)
	{
		for (int x = 0; x < images.front()->cols; x++)
		{
			for (int j = 0; j < images.size(); ++j)
			{
				phase_series.at<float>(y, x) += static_cast<float>(images[j]->at<uchar>(y, x)) * scale[j];
			}
		}
	}

	if (true)
	{
		for (int i = 0; i < phase_series.cols; ++i)
		{
			std::cout << phase_series.at<float>(0, i) << ", ";
		}
		std::cout << std::endl;
	}

	medianBlur(phase_series, phase_series, 5); //中值滤波

	for (int y = 0; y < images.front()->rows; y++)
	{
		for (int x = 0; x < images.front()->cols; x++)
		{
			//绝对相位 = 2*PI*k + θ(x,y)（相对相位/相位主体）
			unwrapped_phase.at<float>(y, x) = phase_series.at<float>(y, x) * 2 * CV_PI + wrapped_phase.at<float>(y, x);
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

cv::Point3d approximate_ray_intersection(const cv::Point3d &v1,
                                         const cv::Point3d &q1,
                                         const cv::Point3d &v2,
                                         const cv::Point3d &q2,
                                         double *distance,
                                         double *out_lambda1,
                                         double *out_lambda2)
{
	cv::Mat v1mat = cv::Mat(v1);
	cv::Mat v2mat = cv::Mat(v2);

	double v1tv1 = cv::Mat(v1mat.t() * v1mat).at<double>(0, 0);
	double v2tv2 = cv::Mat(v2mat.t() * v2mat).at<double>(0, 0);
	double v1tv2 = cv::Mat(v1mat.t() * v2mat).at<double>(0, 0);
	double v2tv1 = cv::Mat(v2mat.t() * v1mat).at<double>(0, 0);

	//cv::Mat V(2, 2, CV_64FC1);
	//V.at<double>(0,0) = v1tv1;  V.at<double>(0,1) = -v1tv2;
	//V.at<double>(1,0) = -v2tv1; V.at<double>(1,1) = v2tv2;
	//std::cout << " V: "<< V << std::endl;

	cv::Mat Vinv(2, 2, CV_64FC1);
	double detV = v1tv1 * v2tv2 - v1tv2 * v2tv1;
	Vinv.at<double>(0, 0) = v2tv2 / detV;
	Vinv.at<double>(0, 1) = v1tv2 / detV;
	Vinv.at<double>(1, 0) = v2tv1 / detV;
	Vinv.at<double>(1, 1) = v1tv1 / detV;
	//std::cout << " V.inv(): "<< V.inv() << std::endl << " Vinv: " << Vinv << std::endl;

	//cv::Mat Q(2, 1, CV_64FC1);
	//Q.at<double>(0,0) = cv::Mat(v1mat.t()*(cv::Mat(q2-q1))).at<double>(0,0);
	//Q.at<double>(1,0) = cv::Mat(v2mat.t()*(cv::Mat(q1-q2))).at<double>(0,0);
	//std::cout << " Q: "<< Q << std::endl;

	cv::Point3d q2_q1 = q2 - q1;
	double Q1 = v1.x * q2_q1.x + v1.y * q2_q1.y + v1.z * q2_q1.z;
	double Q2 = -(v2.x * q2_q1.x + v2.y * q2_q1.y + v2.z * q2_q1.z);

	//cv::Mat L = V.inv()*Q;
	//cv::Mat L = Vinv*Q;
	//std::cout << " L: "<< L << std::endl;

	double lambda1 = (v2tv2 * Q1 + v1tv2 * Q2) / detV;
	double lambda2 = (v2tv1 * Q1 + v1tv1 * Q2) / detV;
	//std::cout << "lambda1: " << lambda1 << " lambda2: " << lambda2 << std::endl;

	//cv::Mat p1 = L.at<double>(0,0)*v1mat + cv::Mat(q1); //ray1
	//cv::Mat p2 = L.at<double>(1,0)*v2mat + cv::Mat(q2); //ray2
	//cv::Point3d p1 = L.at<double>(0,0)*v1 + q1; //ray1
	//cv::Point3d p2 = L.at<double>(1,0)*v2 + q2; //ray2
	cv::Point3d p1 = lambda1 * v1 + q1; //ray1
	cv::Point3d p2 = lambda2 * v2 + q2; //ray2

	//cv::Point3d p = cv::Point3d(cv::Mat((p1+p2)/2.0));
	cv::Point3d p = 0.5 * (p1 + p2);

	if (distance != NULL)
	{
		*distance = cv::norm(p2 - p1);
	}
	if (out_lambda1)
	{
		*out_lambda1 = lambda1;
	}
	if (out_lambda2)
	{
		*out_lambda2 = lambda2;
	}

	return p;
}

void triangulate_stereo(const cv::Mat &K1,
                        const cv::Mat &kc1,
                        const cv::Mat &K2,
                        const cv::Mat &kc2,
                        const cv::Mat &Rt,
                        const cv::Mat &T,
                        const cv::Point2d &p1,
                        const cv::Point2d &p2,
                        cv::Point3d &p3d,
                        double *distance)
{
	//to image camera coordinates
	cv::Mat inp1(1, 1, CV_64FC2), inp2(1, 1, CV_64FC2);
	inp1.at<cv::Vec2d>(0, 0) = cv::Vec2d(p1.x, p1.y);
	inp2.at<cv::Vec2d>(0, 0) = cv::Vec2d(p2.x, p2.y);
	cv::Mat outp1, outp2;
	cv::undistortPoints(inp1, outp1, K1, kc1);
	cv::undistortPoints(inp2, outp2, K2, kc2);
	assert(outp1.type() == CV_64FC2 && outp1.rows == 1 && outp1.cols == 1);
	assert(outp2.type() == CV_64FC2 && outp2.rows == 1 && outp2.cols == 1);
	const cv::Vec2d &outvec1 = outp1.at<cv::Vec2d>(0, 0);
	const cv::Vec2d &outvec2 = outp2.at<cv::Vec2d>(0, 0);
	cv::Point3d u1(outvec1[0], outvec1[1], 1.0);
	cv::Point3d u2(outvec2[0], outvec2[1], 1.0);

	if (false)
	{
		//不去畸变
		cv::Point3d t1(p1.x, p1.y, 1.0);
		cv::Point3d t2(p2.x, p2.y, 1.0);
		u1 = cv::Point3d(cv::Mat(K1.inv() * cv::Mat(t1)));
		u2 = cv::Point3d(cv::Mat(K2.inv() * cv::Mat(t2)));
	}

	//to world coordinates
	cv::Point3d w1 = u1;
	cv::Point3d w2 = cv::Point3d(cv::Mat(Rt * (cv::Mat(u2) - T)));

	//world rays
	cv::Point3d v1 = u1;
	cv::Point3d v2 = cv::Point3d(cv::Mat(Rt * cv::Mat(u2)));

	//compute ray-ray approximate intersection
	p3d = approximate_ray_intersection(v1, w1, v2, w2, distance);
}


void reconstruct_model_patch_center(cv::Mat const &pattern_image, cv::Mat const &min_max_image, cv::Size const &projector_size, int threshold, double max_dist)
{
	CalibrationData calib;
	calib.load_calibration("D:\\calib_file_0.yml");

	if (!pattern_image.data || pattern_image.type() != CV_32FC2)
	{ //pattern not correctly decoded
		std::cerr << "[reconstruct_model] ERROR invalid pattern_image\n";
		return;
	}
	if (!min_max_image.data || min_max_image.type() != CV_8UC2)
	{ //pattern not correctly decoded
		std::cerr << "[reconstruct_model] ERROR invalid min_max_image\n";
		return;
	}

	//parameters
	//const unsigned threshold = config.value("main/shadow_threshold", 70).toUInt();
	//const double   max_dist  = config.value("main/max_dist_threshold", 40).toDouble();
	//const bool     remove_background = config.value("main/remove_background", true).toBool();
	//const double   plane_dist = config.value("main/plane_dist", 100.0).toDouble();
	double plane_dist = 100.0;

	//init point cloud
	int scale_factor_x = 1;
	int scale_factor_y = (projector_size.width > projector_size.height ? 1 : 2); //XXX HACK: preserve regular aspect ratio XXX HACK
	int out_cols = projector_size.width / scale_factor_x;
	int out_rows = projector_size.height / scale_factor_y;

	//candidate points
	std::map<unsigned, cv::Point2f> proj_points;
	std::map<unsigned, std::vector<cv::Point2f>> cam_points;

	//cv::Mat proj_image = cv::Mat::zeros(out_rows, out_cols, CV_8UC3);

	unsigned good = 0;
	unsigned bad = 0;
	unsigned invalid = 0;
	unsigned repeated = 0;
	for (int h = 0; h < pattern_image.rows; h++)
	{
		register const cv::Vec2f *curr_pattern_row = pattern_image.ptr<cv::Vec2f>(h);
		register const cv::Vec2b *min_max_row = min_max_image.ptr<cv::Vec2b>(h);
		for (register int w = 0; w < pattern_image.cols; w++)
		{
			const cv::Vec2f &pattern = curr_pattern_row[w];
			const cv::Vec2b &min_max = min_max_row[w];

			if (pattern[0] < 0.f || pattern[0] >= projector_size.width || pattern[1] < 0.f || pattern[1] >= projector_size.height ||
			    (min_max[1] - min_max[0]) < static_cast<int>(threshold))
			{ //skip
				continue;
			}

			//ok
			cv::Point2f proj_point(pattern[0] / scale_factor_x, pattern[1] / scale_factor_y);
			unsigned index = static_cast<unsigned>(proj_point.y) * out_cols + static_cast<unsigned>(proj_point.x);
			proj_points.insert(std::make_pair(index, proj_point));
			cam_points[index].push_back(cv::Point2f(w, h));

			//proj_image.at<cv::Vec3b>(static_cast<unsigned>(proj_point.y), static_cast<unsigned>(proj_point.x)) = color_image.at<cv::Vec3b>(h, w);
		}
	}

	std::shared_ptr<trimesh::TriMesh> debug(new trimesh::TriMesh);
	cv::Mat Rt = calib.R.t();//单位正交矩阵的逆等于其转置矩阵
	unsigned n = 0;
	for (auto iter = proj_points.begin(); iter != proj_points.end(); ++iter)
	{
		n++;
		unsigned index = iter->first;
		const cv::Point2f &proj_point = iter->second;
		const std::vector<cv::Point2f> &cam_point_list = cam_points[index];
		const unsigned count = static_cast<int>(cam_point_list.size());

		if (!count)
		{ //empty list
			continue;
		}

		//center average
		cv::Point2d sum(0.0, 0.0);
		cv::Point2d sum2(0.0, 0.0);
		for (std::vector<cv::Point2f>::const_iterator iter2 = cam_point_list.begin(); iter2 != cam_point_list.end(); iter2++)
		{
			sum.x += iter2->x;
			sum.y += iter2->y;
			sum2.x += (iter2->x) * (iter2->x);
			sum2.y += (iter2->y) * (iter2->y);
		}
		cv::Point2d cam(sum.x / count, sum.y / count);
		cv::Point2d proj(proj_point.x * scale_factor_x, proj_point.y * scale_factor_y);

		//triangulate
		double distance = max_dist; //quality meassure
		cv::Point3d p; //reconstructed point
		triangulate_stereo(calib.cam_K, calib.cam_kc, calib.proj_K, calib.proj_kc, Rt, calib.T, cam, proj, p, &distance);
		if (distance < max_dist)
		{ //good point

			//evaluate the plane
			double d = plane_dist + 1;
			/*if (remove_background)
            {
                d = cv::Mat(plane.rowRange(0,3).t()*cv::Mat(p) + plane.at<double>(3,0)).at<double>(0,0);
            }*/
			if (d > plane_dist)
			{ //object point, keep
				good++;
				debug->vertices.push_back(trimesh::point(p.x, p.y, p.z));
			}
		}
		else
		{ //skip
			bad++;
			//std::cout << " d = " << distance << std::endl;
		}
	}
	debug->write("D:/debug.ply");

	std::cout << "Reconstructed points [patch center]: " << good << " (" << bad << " skipped, " << invalid << " invalid) " << std::endl
	          << " - repeated points: " << repeated << " (ignored) " << std::endl;
}

} // namespace sr
