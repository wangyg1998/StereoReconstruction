#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>


namespace sr
{
cv::Mat CalWrappedPhase(std::vector<std::shared_ptr<cv::Mat>> images);

void UnwrappedPhaseGraycodeMethod(cv::Mat &wrapped_phase, cv::Mat &unwrapped_phase, std::vector<std::shared_ptr<cv::Mat>> images);

void find_featurepionts_single_match(cv::Mat &leftphase, cv::Mat &rightphase, std::vector<cv::Point2f> &leftkeypoint, std::vector<cv::Point2f> &rightkeypoint);


} // namespace sr