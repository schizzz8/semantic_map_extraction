// Copyright Roberto Capobianco, 2016

#include "semantic_map_extraction/gui.hpp"
#include <iostream>

void GUI::showImage(const cv::Mat &image) {
  cv::namedWindow(this->window_name, CV_WINDOW_NORMAL);
  cv::imshow(this->window_name, image);
  cv::waitKey(1);
}

cv::Mat GUI::loadImage() {
  return cv::imread(this->image_path, CV_LOAD_IMAGE_COLOR);
}

GUI::GUI(const std::string &window_name) {
  this->window_name = window_name;
  this->can_start = false;
}

void GUI::setImage(const std::string &image_path) {
  this->image_path = image_path;
  this->actual_image = this->loadImage();
  this->can_start = true;
}

void GUI::drawRobot(
    const cv::Point2i cell_robot,
    const float theta_robot,
    const int img_size_x,
    const int img_size_y) {
  if (this->can_start != false) {
    cv::Mat image = this->actual_image.clone();

    float ratio_x = image.size().width/img_size_x;
    float ratio_y = image.size().height/img_size_y;

    cv::Point robot_pt(
        cell_robot.x*ratio_x + ratio_x/2,
        cell_robot.y*ratio_y + ratio_y/2);
    cv::Point robot_oriented(
        cell_robot.x*ratio_x + ratio_x/2*(1 + cos(theta_robot)),
        cell_robot.y*ratio_y + ratio_y/2*(1 - sin(theta_robot)));

    cv::circle(image, robot_pt, 4, cv::Scalar(0), -1);
    cv::circle(image, robot_oriented, 3, cv::Scalar(255, 255, 255), -1);
    this->showImage(image);
  }
}
