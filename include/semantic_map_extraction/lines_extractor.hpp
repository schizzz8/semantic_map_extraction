// ---------------------------------------------------------------------------
// lines_extractor.hpp
// Images line extractor and grid producer class
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _LINES_EXTRACTOR_HPP_
#define _LINES_EXTRACTOR_HPP_

#include <queue>
#include <vector>
#include <utility>
#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class LinesExtractor {
	private:
		cv::Mat image;
		cv::Mat original_image;
		cv::Mat dilated_image;
		int max_threshold;
		int min_threshold;
		float min_line_distance;
		int active_pixel_range;
		int active_pixel_threshold;
		int near_active_pixels_threshold;
		cv::Size max_grid_spacing;
		std::vector<cv::Vec2f> extracted_lines;
		std::vector<cv::Vec2f> grid;
		std::vector<std::pair<cv::Point2i, cv::Point2i> > doors;

		bool isSameAngle(const float angle1, const float angle2);
		void extract();
		void computeMaxGridDistance();
		void makeGrid();
		bool checkActivePixel(int row, int col, cv::Point2i &good_pt, cv::Point2i &good_pt_inv);
		std::vector<float> getLinesByAngle(const std::vector<cv::Vec2f> &lines, const float angle);
		void printLinesOnImage(cv::Mat &draw, const std::vector<cv::Vec2f> &lines);
		void printLinesFromPointsOnImage(cv::Mat &draw, const std::vector<std::pair<cv::Point2i, cv::Point2i> > &points);
	public:
		LinesExtractor(const cv::Mat &image, const cv::Mat &original_image, const int max_threshold=100, const int min_threshold=0, const float min_line_distance=5.0f, const int active_pixel_range = 13, const int active_pixel_threshold = 2);
		void processImage();
		std::vector<std::pair<cv::Point2i, cv::Point2i> > computeDoors();
		std::vector<float> getExtractedHorizontalLines();
		std::vector<float> getExtractedVerticalLines();
		std::vector<float> getGridHorizontalLines();
		std::vector<float> getGridVerticalLines();
		cv::Mat getExtractedImage();
		cv::Mat getGridImage();
		cv::Mat getActivePixelImage();
};

#endif
