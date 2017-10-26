// Copyright Roberto Capobianco, 2016

#include "semantic_map_extraction/lines_extractor.hpp"

bool LinesExtractor::isSameAngle(const float angle1, const float angle2) {
  return (std::fabs(
      std::fmod(angle1,
                static_cast<float>(CV_PI))
      - std::fmod(angle2,
                  static_cast<float>(CV_PI)))
          < .01f);
}

void LinesExtractor::extract() {
  std::vector<cv::Vec2f> lines;
  bool toPrint = true;

  this->extracted_lines.push_back(cv::Vec2f(0, 0));
  this->extracted_lines.push_back(cv::Vec2f(0, CV_PI/2));
  this->extracted_lines.push_back(cv::Vec2f(this->image.cols, 0));
  this->extracted_lines.push_back(cv::Vec2f(this->image.rows, CV_PI/2));

  for (int j = this->max_threshold; j > this->min_threshold; j--) {
    cv::HoughLines(this->image, lines, 2, CV_PI/2, (j > 0) ? j : 1, 0, 0);
    float min_distance = std::max(
        this->max_threshold/static_cast<float>(j),
        this->min_line_distance);

    for (std::vector<cv::Vec2f>::iterator i = lines.begin();
         i != lines.end(); i++) {
      float rho = (*i)[0], theta = (*i)[1];
      toPrint = true;

      for (std::vector<cv::Vec2f>::iterator it = this->extracted_lines.begin();
           it != this->extracted_lines.end(); it++)
        if (std::fabs((*it)[0] - rho) < min_distance
            && this->isSameAngle((*it)[1], theta)) {
          toPrint = false;
          break;
        }

      if (toPrint)
        this->extracted_lines.push_back(*i);
    }
  }
}

void LinesExtractor::computeMaxGridDistance() {
  for (std::vector<cv::Vec2f>::iterator i = this->extracted_lines.begin();
       i != this->extracted_lines.end() - 1; ++i) {
    for (std::vector<cv::Vec2f>::iterator j = i + 1;
         j != this->extracted_lines.end(); ++j) {
      if (this->isSameAngle((*i)[1], (*j)[1])) {
        if (this->isSameAngle((*i)[1], 0.0f))
          this->max_grid_spacing.width = std::min(
              fabs((*i)[0] - (*j)[0]),
              static_cast<double>(this->max_grid_spacing.width));
        else
          this->max_grid_spacing.height = std::min(
              fabs((*i)[0] - (*j)[0]),
              static_cast<double>(this->max_grid_spacing.height));
      }
    }
  }
}

void LinesExtractor::makeGrid() {
  cv::Size image_size = this->image.size();
  std::queue<std::pair<cv::Vec2f, cv::Vec2f> > processing_queue;

  for (std::vector<cv::Vec2f>::iterator i = this->extracted_lines.begin();
       i != this->extracted_lines.end(); i++) {
    float segment_dist = std::max(image_size.width, image_size.height);
    cv::Vec2f first_line, second_line;

    // Detect the smallest segment for each line
    for (std::vector<cv::Vec2f>::iterator j = this->extracted_lines.begin();
         j != this->extracted_lines.end(); j++) {
      first_line = *i;

      if (this->isSameAngle((*i)[1], (*j)[1])) {
        if ((*i)[0] - (*j)[0] > 0.0 && (*i)[0] - (*j)[0] < segment_dist) {
          segment_dist = (*i)[0] - (*j)[0];
          second_line = *j;
        }
      }
    }

    //For each detected segment, divide it (if needed)
    if (segment_dist != std::max(image_size.width, image_size.height))
      processing_queue.push(std::make_pair(first_line, second_line));

    this->grid.push_back(*i);
  }

  while (!processing_queue.empty()) {
    std::pair<cv::Vec2f, cv::Vec2f> in_process(processing_queue.front());
    processing_queue.pop();

    float max_dist = (this->isSameAngle(in_process.first[1], 0.0))
        ? max_grid_spacing.width : max_grid_spacing.height;

    if (std::floor(
            std::fabs(in_process.first[0] - in_process.second[0])/max_dist)
        >= 2.0) {
      cv::Vec2f new_line(
          (in_process.first[0] + in_process.second[0])/2.0f,
          in_process.first[1]);

      this->grid.push_back(new_line);
      processing_queue.push(std::make_pair(new_line, in_process.first));
      processing_queue.push(std::make_pair(in_process.second, new_line));
    }
  }
}

bool LinesExtractor::checkActivePixel(int row, int col,
                                      cv::Point2i &good_pt,
                                      cv::Point2i &good_pt_inv) {
  int range_pixels = 0;
  cv::Point2i pt1(
      std::max(
          0, col - this->active_pixel_range),
      std::max(0, row - this->active_pixel_range));
  cv::Point2i pt2(
      std::min(this->image.cols - 1, col + this->active_pixel_range),
      std::min(this->image.rows - 1, row + this->active_pixel_range));

  cv::Rect roi(pt1, pt2);
  cv::Mat image_roi = this->dilated_image(roi);

  cv::Mat points;
  cv::findNonZero(image_roi, points);

  size_t total_points = points.total();

  if (static_cast<float>(total_points)
      /static_cast<float>(this->active_pixel_range
                          *this->active_pixel_range)
      > 0.5f)
    return false;

  cv::Point2i central_pt(col, row);

  for (size_t i = 0; i < total_points; i++) {
    cv::Point2i pt = points.at<cv::Point2i>(i) + pt1 - central_pt;

    bool found = false;

    for (size_t j = std::min(i + 1, total_points); j < total_points; j++) {
      cv::Point2i pt_inv = points.at<cv::Point2i>(j) + pt1 - central_pt;

      if (cv::norm(pt) < 0.35*this->active_pixel_range)
        return false;

      if (pt == -pt_inv) {
        found = true;

        if (range_pixels == 0
            || std::abs(pt.x - pt_inv.x)
            < std::abs(good_pt.x - good_pt_inv.x)
            || std::abs(pt.y - pt_inv.y)
            < std::abs(good_pt.y - good_pt_inv.y)) {
          good_pt = pt + central_pt;
          good_pt_inv = pt_inv + central_pt;
        }

        break;
      }
    }

    if (found)
        range_pixels++;
  }

  if (range_pixels > this->active_pixel_threshold
      && static_cast<float>(range_pixels)/static_cast<float>(total_points)
      < 0.08
      && range_pixels < 0.35*this->active_pixel_range)
    return true;

  return false;
}

std::vector<std::pair<cv::Point2i, cv::Point2i> >
LinesExtractor::computeDoors() {
  for (int row = 0; row < this->image.rows; row+=2) {
    for (int col = 0; col < this->image.cols; col+=2) {
      cv::Point2i pt, pt_inv;

      if (this->checkActivePixel(row, col, pt, pt_inv)) {
        std::vector<std::pair<cv::Point2i, cv::Point2i> >::iterator i;
        float norm = cv::norm(pt - pt_inv);

        for (i = this->doors.begin();
             i != this->doors.end() &&
                 cv::norm(cv::Point2i((i->first.x + i->second.x)/2,
                                      (i->first.y + i->second.y)/2)
                          - cv::Point2i((pt.x + pt_inv.x)/2,
                                        (pt.y + pt_inv.y)/2))
                 > 2*this->active_pixel_range; ++i) {}

        if (i == this->doors.end()) {
          this->doors.push_back(std::make_pair(pt, pt_inv));
        } else {
          float old_norm = cv::norm(i->first - i->second);
          float new_pt_xDist = std::abs(pt.x - pt_inv.x);
          float new_pt_yDist = std::abs(pt.y - pt_inv.y);
          float old_pt_xDist = std::abs(i->first.x - i->second.x);
          float old_pt_yDist = std::abs(i->first.y - i->second.y);
          bool check_x = new_pt_xDist > 0.6*norm
              && old_pt_xDist > 0.6*old_norm
              && new_pt_xDist < old_pt_xDist;
          bool check_y = new_pt_yDist > 0.6*norm
              && old_pt_yDist > 0.6*old_norm
              && new_pt_yDist < old_pt_yDist;

          if (check_x || check_y)
            this->doors[i - this->doors.begin()] = std::make_pair(pt, pt_inv);
        }
      }
    }
  }

  return this->doors;
}

std::vector<float> LinesExtractor::getLinesByAngle(
    const std::vector<cv::Vec2f> &lines, const float angle) {
  std::vector<float> good_angle;

  for (std::vector<cv::Vec2f>::const_iterator i = lines.begin();
       i != lines.end(); i++)
    if (this->isSameAngle((*i)[1], angle))
      good_angle.push_back((*i)[0]);

  std::sort(good_angle.begin(), good_angle.end());

  return good_angle;
}

void LinesExtractor::printLinesOnImage(cv::Mat &draw,
                                       const std::vector<cv::Vec2f> &lines) {
  cv::cvtColor(draw, draw, CV_GRAY2BGR);

  for (std::vector<cv::Vec2f>::const_iterator i = lines.begin();
       i != lines.end(); i++) {
    double x0 = (*i)[0]*cos((*i)[1]), y0 = (*i)[0]*sin((*i)[1]);
    cv::Point pt1(cvRound(x0), cvRound(y0 + draw.rows*cos((*i)[1])));
    cv::Point pt2(cvRound(x0 + draw.cols*sin((*i)[1])), cvRound(y0));
    cv::line(draw, pt1, pt2, cv::Scalar(0, 0, 255), 0.5, CV_AA);
  }
}

void LinesExtractor::printLinesFromPointsOnImage(
    cv::Mat &draw,
    const std::vector<std::pair<cv::Point2i, cv::Point2i> > &points) {
  cv::cvtColor(draw, draw, CV_GRAY2BGR);

  for (std::vector<std::pair<cv::Point2i, cv::Point2i> >::const_iterator
           i = points.begin(); i != points.end(); i++) {
    cv::line(draw, i->first, i->second, cv::Scalar(0, 0, 255), 0.5, CV_AA);
  }
}

LinesExtractor::LinesExtractor(const cv::Mat &image,
                               const cv::Mat &original_image,
                               const int max_threshold,
                               const int min_threshold,
                               const float min_line_distance,
                               const int active_pixel_range,
                               const int active_pixel_threshold) {
  this->image = image.clone();
  this->original_image = original_image.clone();
  cv::dilate(this->image, this->dilated_image, cv::Mat(), cv::Point(-1, -1), 1);
  cv::erode(this->dilated_image, this->dilated_image,
            cv::Mat(), cv::Point(-1, -1), 1);
  this->max_threshold = max_threshold;
  this->min_threshold = min_threshold;
  this->min_line_distance = min_line_distance;
  this->active_pixel_range = active_pixel_range;
  this->active_pixel_threshold = active_pixel_threshold;
  this->max_grid_spacing = this->image.size();
}

void LinesExtractor::processImage() {
  this->extract();
  this->computeMaxGridDistance();
  this->makeGrid();
}

std::vector<float> LinesExtractor::getExtractedHorizontalLines() {
  return this->getLinesByAngle(this->extracted_lines, CV_PI/2);
}

std::vector<float> LinesExtractor::getExtractedVerticalLines() {
  return this->getLinesByAngle(this->extracted_lines, 0.0f);
}

std::vector<float> LinesExtractor::getGridHorizontalLines() {
  return this->getLinesByAngle(this->grid, CV_PI/2);
}

std::vector<float> LinesExtractor::getGridVerticalLines() {
  return this->getLinesByAngle(this->grid, 0.0f);
}

cv::Mat LinesExtractor::getExtractedImage() {
  cv::Mat extracted_image = this->image.clone();
  cv::threshold(extracted_image, extracted_image,
                100, 255, cv::THRESH_BINARY_INV);
  this->printLinesOnImage(extracted_image, this->extracted_lines);
  return extracted_image;
}

cv::Mat LinesExtractor::getGridImage() {
  cv::Mat grid_image = this->image.clone();
  cv::threshold(grid_image, grid_image, 100, 255, cv::THRESH_BINARY_INV);
  this->printLinesOnImage(grid_image, this->grid);
  return grid_image;
}

cv::Mat LinesExtractor::getActivePixelImage() {
  cv::Mat active_pixels_image = this->dilated_image.clone();
  cv::threshold(active_pixels_image, active_pixels_image,
                100, 255, cv::THRESH_BINARY_INV);
  this->printLinesFromPointsOnImage(active_pixels_image, this->doors);
  return active_pixels_image;
}

