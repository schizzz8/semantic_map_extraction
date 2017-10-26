// Copyright Roberto Capobianco, 2016

#include <iostream>

#include "semantic_map_extraction/grid_map.hpp"
#include "semantic_map_extraction/lines_extractor.hpp"

std::vector<float> GridMap::computeLineDistances(std::vector<float> lines) {
  std::vector<float> distances;

  for (std::vector<float>::iterator i = lines.begin();
       i != lines.end() - 1; ++i)
    distances.push_back(*(i + 1) - *i);

  return distances;
}

void GridMap::extractGrid() {
  LinesExtractor extractor(this->stack[1], this->original_map);
  extractor.processImage();
  this->stack.push_back(extractor.getExtractedImage().clone());
  this->operations.push_back("extracted_lines");
  this->stack.push_back(extractor.getGridImage().clone());
  this->operations.push_back("grid_lines");
  this->horizontal_lines = extractor.getGridHorizontalLines();
  this->vertical_lines = extractor.getGridVerticalLines();
}

GridMap::GridMap(const cv::Mat &original_map) {
  this->original_map = original_map.clone();
  this->stack.push_back(this->original_map);
  this->operations.push_back("original");
}

void GridMap::produce() {
  cv::Mat tmp;

  // cv::Canny(this->original_map, tmp, 1, 30, 3, true);
  cv::Canny(this->original_map, tmp, 0.3, 30, 3, true);
  this->stack.push_back(tmp.clone());
  this->operations.push_back("canny");
  cv::threshold(tmp, tmp, 100, 255, cv::THRESH_BINARY_INV);
  this->stack.push_back(tmp.clone());
  this->operations.push_back("canny_inverted");
  this->extractGrid();
}

void GridMap::showStack() {
  std::vector<std::string>::iterator j = this->operations.begin();

  for (std::vector<cv::Mat>::iterator i = this->stack.begin();
       i != this->stack.end(); i++, j++) {
    cv::namedWindow(*j, CV_WINDOW_NORMAL);
    cv::imshow(*j, *i);
  }
}

bool GridMap::saveStack(std::string path, std::string basename) {
  std::vector<std::string>::iterator j = this->operations.begin();

  for (std::vector<cv::Mat>::iterator i = this->stack.begin();
       i != this->stack.end(); i++, j++) {
    if (!cv::imwrite(path + "/" + basename + "_" + *j + ".png", *i))
      return false;
  }

  return true;
}

cv::Point2i GridMap::getCellFromImageCoords(cv::Point2i coords) {
  cv::Point2i cell(-1, -1);
  cv::Size dims = this->original_map.size();

  if (coords.x < dims.width
      && coords.y < dims.height
      && coords.x >= 0
      && coords.y >= 0) {
    for (std::vector<float>::iterator i = this->horizontal_lines.begin();
         coords.y >= *i; i++, cell.y++) {}
    for (std::vector<float>::iterator i = this->vertical_lines.begin();
         coords.x >= *i; i++, cell.x++) {}
  }

  return cell;
}

cv::Point2i GridMap::getImageCoordsFromCell(cv::Point2i cell) {
  cv::Point2i coords(0, 0);
  std::vector<float> vertical_distances = this->getVerticalDistances();
  std::vector<float> horizontal_distances = this->getHorizontalDistances();

  if (cell.x <= static_cast<int>(horizontal_distances.size())
      && cell.y <= static_cast<int>(vertical_distances.size())
      && cell.x >= 0 && cell.y >= 0) {
    for (std::vector<float>::iterator i = vertical_distances.begin();
         i <= vertical_distances.begin() + cell.y;
         coords.y +=
             (i == vertical_distances.begin()) ? (*i)/2 : *i, i++) {}
    for (std::vector<float>::iterator i = horizontal_distances.begin();
         i <= horizontal_distances.begin() + cell.x;
         coords.x +=
             (i == horizontal_distances.begin()) ? (*i)/2 : *i, i++) {}
  }

  return coords;
}

std::vector<float> GridMap::getHorizontalDistances() {
  return this->computeLineDistances(this->vertical_lines);
}

std::vector<float> GridMap::getVerticalDistances() {
  return this->computeLineDistances(this->horizontal_lines);
}
