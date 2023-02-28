#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>


namespace sr
{
cv::Mat CalWrappedPhase(std::vector<std::shared_ptr<cv::Mat>> images);

void decodeGrayCode(std::vector<std::shared_ptr<cv::Mat>> images, cv::Mat &result);

void UnwrappedPhaseGraycodeMethod(cv::Mat &wrapped_phase, cv::Mat &unwrapped_phase, std::vector<std::shared_ptr<cv::Mat>> images);

void find_featurepionts_single_match(cv::Mat &leftphase, cv::Mat &rightphase, std::vector<cv::Point2f> &leftkeypoint, std::vector<cv::Point2f> &rightkeypoint);

cv::Point3d approximate_ray_intersection(const cv::Point3d &v1,
                                         const cv::Point3d &q1,
                                         const cv::Point3d &v2,
                                         const cv::Point3d &q2,
                                         double *distance = NULL,
                                         double *out_lambda1 = NULL,
                                         double *out_lambda2 = NULL);

void triangulate_stereo(const cv::Mat &K1,
                        const cv::Mat &kc1,
                        const cv::Mat &K2,
                        const cv::Mat &kc2,
                        const cv::Mat &Rt,
                        const cv::Mat &T,
                        const cv::Point2d &p1,
                        const cv::Point2d &p2,
                        cv::Point3d &p3d,
                        double *distance = NULL);

void reconstruct_model_patch_center(cv::Mat const &pattern_image, cv::Mat const &min_max_image, cv::Size const &projector_size, int threshold, double max_dist);

} // namespace sr