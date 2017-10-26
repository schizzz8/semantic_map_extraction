// Copyright Roberto Capobianco, 2016

// ---------------------------------------------------------------------------
// semantic_map_extraction.hpp
// Semantic extraction node handler class and main
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _SEMANTIC_NODE_HPP_
#define _SEMANTIC_NODE_HPP_


#include <ros/ros.h>
#include <tinyxml.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "semantic_map_extraction/Obs.h"
#include "semantic_map_extraction/GenerateKB.h"
#include "semantic_map_extraction/GetCellByCoords.h"
#include "semantic_map_extraction/GetCoordsByCell.h"
#include "semantic_map_extraction/GetCoordsByName.h"
#include "semantic_map_extraction/DeleteMap.h"

#include "opencv2/highgui/highgui.hpp"

#include "grid_map.hpp"
#include "room_map.hpp"
#include "semantic_matrix.hpp"
#include "objects.hpp"
#include "gui.hpp"

class SemanticNode {
 public:
  SemanticNode(ros::NodeHandle *n,
               std::string robotname = "robot0",
               std::string base_frame_id = "base_footprint",
               std::string path = "./",
               std::string StatXMLFilePath = "./mapXMLfile.xml",
               std::string DinXMLFilePath = "./mapXMLfile.xml",
               int timeout = 30,
               bool wait_service = false,
               bool load_dyn_map = true);
  SemanticNode(ros::NodeHandle *n,
               const cv::Mat &image,
               std::string robotname = "robot0",
               std::string base_frame_id = "base_footprint",
               std::string path = "./",
               std::string StatXMLFilePath = "./mapXMLfile.xml",
               std::string DinXMLFilePath = "mapXMLfile.xml",
               int timeout = 30,
               bool wait_service = false,
               bool load_dyn_map = true);
  ~SemanticNode();
  void mapProcessing();
  void areaTagsProcessing();
  void objectTagsProcessing();
  void addElement();
  void mapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& map);
  void ObservationTopicSubscriber(
      const semantic_map_extraction::Obs::ConstPtr& observation);
  void ASRSubscriber(const std_msgs::String::ConstPtr& string);
  void RemoveObjectSubscriber(const std_msgs::String::ConstPtr& string);
  void UpdateObjectPropertySubscriber(const std_msgs::String::ConstPtr& s);
  void UpdateObjectCoordsSubscriber(const std_msgs::String::ConstPtr& s);
  void RobotPoseTimer(const ros::TimerEvent&);
  bool cellByCoordsHandler(
      semantic_map_extraction::GetCellByCoords::Request &req,
      semantic_map_extraction::GetCellByCoords::Response &res);
  bool coordsByCellHandler(
      semantic_map_extraction::GetCoordsByCell::Request &req,
      semantic_map_extraction::GetCoordsByCell::Response &res);
  bool coordsByNameHandler(
      semantic_map_extraction::GetCoordsByName::Request &req,
      semantic_map_extraction::GetCoordsByName::Response &res);
  bool deleteMapHandler(
      semantic_map_extraction::DeleteMap::Request &req,
      semantic_map_extraction::DeleteMap::Response &res);
  void showImages();
  void saveImages(std::string basename = "map");

 private:
  ros::NodeHandle* n;
  std::string robotname;
  std::string base_frame_id;
  cv::Mat image;
  float resolution;
  float origin_x;
  float origin_y;
  std::string path;
  std::string StatXMLFilePath;
  std::string DynXMLFilePath;
  int timeout;
  bool wait_service;
  bool load_dyn_map;

  tf::TransformListener* listener;

  struct {
    ros::Time time;
    std::string last_writer;
    std::string name;
    cv::Point2f pt;
    float angle;
    cv::Vec3f size;
    std::string properties;
    type ob_type;
  } buffer;

  std::vector<std::pair<std::string, cv::Point> > tags;
  std::vector<std::pair<std::string, cv::Point> > imposed_tags;
  std::vector<bool> static_tags;
  std::vector<Objects> objects;
  std::vector<bool> static_objects;
  std::vector<float> horizontal_distances;
  std::vector<float> vertical_distances;
  std::vector<float> horizontal_cell_centers;
  std::vector<float> vertical_cell_centers;
  GridMap* grid_map;
  RoomMap* room_map;
  SemanticMatrix* semantic_matrix;

  GUI* sem_gui;

  void param_init(bool first_run = true);
  void check_object_area(std::vector<cv::Point2i> cell);
  void areasUpdate();
  void callService();
  void loadXMLMap(std::string XMLFilePath);
  void saveXMLMap(std::string XMLFilePath);
};

#endif
