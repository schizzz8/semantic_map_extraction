// Copyright Roberto Capobianco, 2016

#include "semantic_map_extraction/color_extractor.hpp"


ColorExtractor::ColorExtractor() {
  this->color_lb = this->color_ub = NULL;
}

ColorExtractor::ColorExtractor(const cv::Scalar &color_lb,
                               const cv::Scalar &color_ub) {
  this->setColorRange(color_lb, color_ub);
}

void ColorExtractor::setColorRange(const cv::Scalar &color_lb,
                                   const cv::Scalar &color_ub) {
  delete this->color_lb;
  delete this->color_ub;
  this->color_lb = new cv::Scalar(color_lb);
  this->color_ub = new cv::Scalar(color_ub);
}

cv::Mat ColorExtractor::extractColor(const cv::Mat &image) {
  cv::Mat mask, result = image.clone();

  // get the mask
  cv::inRange(image, *this->color_lb, *this->color_ub, mask);

  // fill the holes std::vector<std::vector<cv::Point> > contours;
  // cv::findContours(mask.clone(), contours, CV_RETR_EXTERNAL,
  // CV_CHAIN_APPROX_SIMPLE); mask = cv::Mat::zeros(image.size(),
  // CV_8U); cv::drawContours(mask, contours, -1,
  // cv::Scalar::all(255), CV_FILLED);

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  return mask;
}
