// Copyright Roberto Capobianco, 2016

// ---------------------------------------------------------------------------
// cell.hpp
// Semantic Cell handler class
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _SEMCELL_HPP_
#define _SEMCELL_HPP_

#include <string>
#include <vector>

#include <ros/ros.h>


class Cell {
 public:
  Cell() : door_adjacency(4, false) {}
  Cell(const std::string &room_tag,
       const std::vector<std::string> &objects,
       const std::vector<bool> &door_adjacency,
       const std::vector<std::string> &additionals);
  void setRoomTag(const std::string &room_tag);
  void setObjects(const std::vector<std::string> &objects);
  void setDoorAdjacency(const std::vector<bool> &door_adjacency);
  void setAdditionals(const std::vector<std::string> &additionals);
  std::string getRoomTag();
  std::vector<std::string> getObjects();
  std::vector<bool> getDoorAdjacency();
  std::vector<std::string> getAdditionals();
  std::string toString();
 private:
  std::string room_tag;
  std::vector<std::string> objects;
  std::vector<bool> door_adjacency;
  std::vector<std::string> additionals;

  std::string getVectorString(std::vector<std::string> str_vec);
  std::string getVectorString(std::vector<bool> str_vec);
};

#endif
