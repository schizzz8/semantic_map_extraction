// ---------------------------------------------------------------------------
// gui.hpp
// Semantic Map Extraction GUI
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _GUI_HPP_
#define _GUI_HPP_

#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class GUI {
	private:
		std::string window_name;
		std::string image_path;
		bool can_start;
		cv::Mat actual_image;
		
		void showImage(const cv::Mat &image);
		cv::Mat loadImage();
		
	public:
		GUI(const std::string &window_name);
		void setImage(const std::string &image_path);
		void drawRobot(const cv::Point2i cell_x_robot, const float theta_robot, const int img_size_x, const int img_size_y);
};

#endif
