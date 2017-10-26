// Copyright Roberto Capobianco, 2016

// ---------------------------------------------------------------------------
// objects.hpp
// Semantic objects handler class
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _OBJECTS_HPP_
#define _OBJECTS_HPP_

#include <string>
#include <vector>

#include "opencv2/highgui/highgui.hpp"


enum type {DOOR, NORMAL};
enum direction {UP, RIGHT, DOWN, LEFT};

class Objects {
 public:
  Objects(const std::string name,
          const cv::Point2i &coords,
          const float &angle,
          const cv::Vec3f &dims,
          const std::string &properties,
          const type &object_type);
  std::string getName();
  cv::Point getCoords();
  float getAngle();
  direction getDir();
  cv::Vec3f getDims();
  std::string getProperties();
  std::string getAddProperties();
  void updateProperties(std::string properties);
  void updateCoords(cv::Point coords);
  type getType();
  std::string getStringType();
  std::vector<cv::Point2i> getVertexCoords();

 private:
  std::string name;
  cv::Point coords;
  float angle;
  direction dir;
  cv::Vec3f dims;
  std::string properties;
  std::string add_properties;
  type object_type;
};

#endif
