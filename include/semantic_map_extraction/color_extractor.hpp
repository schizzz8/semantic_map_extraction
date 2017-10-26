// ---------------------------------------------------------------------------
// color_extractor.hpp
// Color based segmentation class
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _COLOR_EXTRACTOR_HPP_
#define _COLOR_EXTRACTOR_HPP_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class ColorExtractor {
	private:
		cv::Scalar* color_lb;
		cv::Scalar* color_ub;
	public:
		ColorExtractor();
		ColorExtractor(const cv::Scalar &color_lb, const cv::Scalar &color_ub);
		void setColorRange(const cv::Scalar &color_lb, const cv::Scalar &color_ub);
		cv::Mat extractColor(const cv::Mat &image);
};

#endif
