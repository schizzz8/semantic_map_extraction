// ---------------------------------------------------------------------------
// grid_map.hpp
// GridMap processor and handler class
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _GRIDMAP_HPP_
#define _GRIDMAP_HPP_

#include <string>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class GridMap {
	private:
		cv::Mat original_map;
		std::vector<float> horizontal_lines;
		std::vector<float> vertical_lines;
		std::vector<cv::Mat> stack;
		std::vector<std::string> operations;

		std::vector<float> computeLineDistances(std::vector<float> lines);
		void extractGrid();
	public:
		GridMap(const cv::Mat &original_map);
		void produce();
		void showStack();
		bool saveStack(std::string path="./", std::string basename="map");
		cv::Point2i getCellFromImageCoords(cv::Point2i coords);
		cv::Point2i getImageCoordsFromCell(cv::Point2i cell);
		std::vector<float> getHorizontalDistances();
		std::vector<float> getVerticalDistances();
};

#endif
