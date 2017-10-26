// Copyright Roberto Capobianco, 2016

#include <iostream>
#include "semantic_map_extraction/cell.hpp"


std::string Cell::getVectorString(std::vector<std::string> str_vec) {
  std::string str = "0";

  for (std::vector<std::string>::iterator i = str_vec.begin();
       i != str_vec.end(); i++) {
    if (i == str_vec.begin()) {
      str = *i;
    } else {
      str += "#" + *i;
    }
  }

  return str;
}

std::string Cell::getVectorString(std::vector<bool> str_vec) {
  std::string str = "0";

  for (std::vector<bool>::iterator i = str_vec.begin(); i != str_vec.end(); i++)
    if (i == str_vec.begin())
      str = (*i) ? "true" : "false";
    else
      str += (*i) ? "#true" : "#false";

  return str;
}

Cell::Cell(
    const std::string &room_tag, const std::vector<std::string> &objects,
    const std::vector<bool> &door_adjacency,
    const std::vector<std::string> &additionals) {
  this->room_tag = room_tag;
  this->objects = objects;

  if (door_adjacency.size() == 4)
    this->door_adjacency = door_adjacency;
  else
    ROS_WARN("Door adjacency size must be 4.");

  this->additionals = additionals;

  if (this->room_tag.empty())
    ROS_ERROR("The area has not been inserted.");
}

void Cell::setRoomTag(const std::string &room_tag) {
  this->room_tag = room_tag;

  if (this->room_tag.empty())
    ROS_ERROR("The area has not been inserted.");
}

void Cell::setObjects(const std::vector<std::string> &objects) {
  this->objects = objects;

  if (this->room_tag.empty())
    ROS_WARN("The object is out of any area.");
}

void Cell::setDoorAdjacency(const std::vector<bool> &door_adjacency) {
  if (door_adjacency.size() == 4)
    this->door_adjacency = door_adjacency;
  else
    ROS_ERROR("Door adjacency size must be 4.");

  if (this->room_tag.empty())
    ROS_WARN("The door is out of any area.");
}

void Cell::setAdditionals(const std::vector<std::string> &additionals) {
  this->additionals.clear();

  for (int i = 0; i < additionals.size(); ++i) {
    this->additionals.push_back(additionals.at(i));
  }
}

std::string Cell::getRoomTag() {
  return this->room_tag;
}

std::vector<std::string> Cell::getObjects() {
  return this->objects;
}

std::vector<bool> Cell::getDoorAdjacency() {
  return this->door_adjacency;
}

std::vector<std::string> Cell::getAdditionals() {
  return this->additionals;
}

std::string Cell::toString() {
  std::string str;
  std::string area = (this->room_tag.empty())
      ? "0" : this->room_tag;

  if (this->room_tag.empty() && this->objects.empty()) {
    str = "0";
  } else {
    str = area + ";" + getVectorString(this->objects) + ";"
        + getVectorString(this->door_adjacency) + ";"
        + getVectorString(this->additionals);
  }
  
  return str;
}
