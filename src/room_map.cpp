// Copyright Roberto Capobianco, 2016

#include "semantic_map_extraction/room_map.hpp"

RoomMap::RoomMap(const cv::Mat &original_map) {
  this->original_map = original_map.clone();
  this->original_with_doors_map = original_map.clone();
  this->room_map = cv::Mat(this->original_map.size(), CV_8UC3);
  this->room_map = cv::Scalar::all(0);
}

void RoomMap::addDoors(
    std::vector<std::pair<cv::Point2i, cv::Point2i> > doors) {
  if (doors.size() == 0) {
    cv::Mat tmp;
    cv::Canny(this->original_map, tmp, 0.3, 30, 3, true);
    LinesExtractor extractor(tmp, this->original_map);
    doors = extractor.computeDoors();
  }

  for (std::vector<std::pair<cv::Point2i, cv::Point2i> >::const_iterator
           i = doors.begin(); i != doors.end(); i++)
    cv::line(this->original_with_doors_map,
             i->first, i->second, cv::Scalar(0), 0.5, CV_AA);
}

void RoomMap::addDoors(
    const std::pair<cv::Point2i, cv::Point2i> &door) {
  cv::line(this->original_with_doors_map,
           door.first, door.second, cv::Scalar(0), 0.5, CV_AA);
}

void RoomMap::extractRooms(const std::vector<cv::Point> &tags) {
  cv::Mat thresholded(this->original_map.size(), CV_8U);
  cv::Mat markers(this->original_map.size(), CV_32S);
  int compCount = 200;

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Vec3b> colorTab;

  markers = cv::Scalar::all(0);

  cv::threshold(
      this->original_map, thresholded, 220, 255, cv::THRESH_BINARY_INV);
  cv::circle(
      markers, cv::Point(1, 1), 1, cv::Scalar(compCount), -1);

  for (std::vector<cv::Point>::const_iterator i = tags.begin();
       i != tags.end(); ++i, ++compCount)
    cv::circle(markers, *i, 1, cv::Scalar(compCount+1), -1);

  cv::cvtColor(this->original_with_doors_map, this->room_map, CV_GRAY2BGR);
  cv::cvtColor(thresholded, thresholded, CV_GRAY2BGR);
  cv::watershed(this->room_map, markers);

  for (int i = 0; i < compCount; i++) {
    colorTab.push_back(cv::Vec3b(
        static_cast<uchar>(cv::theRNG().uniform(0, 255)),
        static_cast<uchar>(cv::theRNG().uniform(0, 255)),
        static_cast<uchar>(cv::theRNG().uniform(0, 255))));
  }

  for (int i = 0; i < markers.rows; ++i) {
    for (int j = 0; j < markers.cols; ++j) {
      int idx = markers.at<int>(i, j);

      if (idx == -1 && i > 1 && j > 1)
        idx = markers.at<int>(i - 1, j - 1);

      if (idx <= 0 || idx > compCount) {
        this->room_map.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      } else {
        this->room_map.at<cv::Vec3b>(i, j) = cv::Vec3b(colorTab[idx - 1]);
      }
    }
  }

  this->room_map = this->room_map - thresholded;
  cv::erode(this->room_map, this->room_map, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(this->room_map, this->room_map, cv::Mat(), cv::Point(-1, -1), 1);
}

cv::Mat RoomMap::getRoomByTag(const cv::Point &tag) {
  cv::Vec3b values = this->room_map.at<cv::Vec3b>(tag);
  cv::Scalar color(values[0], values[1], values[2]);
  color_extractor.setColorRange(color, color);
  return color_extractor.extractColor(this->room_map);
}

