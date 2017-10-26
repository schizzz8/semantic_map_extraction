// Copyright Roberto Capobianco, 2016

// ---------------------------------------------------------------------------
// semantic_matrix.hpp
// Semantic Matrix handler class
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _SEMMATRIX_HPP
#define _SEMMATRIX_HPP

#include <stdint.h>
#include <time.h>

#include <sstream>
#include <string>
#include <vector>
#include <map>

#include "opencv2/highgui/highgui.hpp"

#include "cell.hpp"
#include "objects.hpp"
#include "room_map.hpp"


class SemanticMatrix {
public:
    SemanticMatrix(const int rows, const int cols);
    Cell* getCellByIndexes(const int row, const int col);
    void setRoomTags(const std::vector<std::pair<std::string, cv::Point> > &tags,
                     RoomMap &room_map,
                     const std::vector<float> &check_x,
                     const std::vector<float> &check_y);
    void setObjectTags(const std::vector<std::vector<cv::Point2i> > &cell_objects,
                       std::vector<Objects> &objects);
    std::vector<std::__cxx11::string> getObjectTags(const std::vector<std::vector<cv::Point2i> > &cell_objects);
    void removeObjectByName(const std::string object_name);
    void removeObjectAdditionals(const std::string additionals);
    void showMatrixImage();
    void showMapImage(const cv::Mat &map,
                      const std::vector<float> &horizontal_distances,
                      const std::vector<float> &vertical_distances);
    std::string saveMatrixImage(std::string path="./",
                                std::string basename="map");
    std::string saveMapImage(const cv::Mat &map,
                             const std::vector<float> &horizontal_distances,
                             const std::vector<float> &vertical_distances,
                             std::string path="./",
                             std::string basename="map");
    std::vector<std::string> msg_format();

private:
    int rows;
    int cols;
    std::vector<std::vector<Cell> > semantic_matrix;

    void addCellValues(int row, int col,
                       std::string object_name = std::string(),
                       std::string additionals = std::string(),
                       bool change_adjacency = false,
                       direction adjacency_dir = RIGHT);
    const Cell& retrieveCellValues(int row, int col);
    void setDoor(const std::vector<cv::Point2i> &door_cells, Objects &door);
    void setObject(const std::vector<cv::Point2i> &object_cells, Objects &object);
    std::vector<std::__cxx11::string> getObject(const std::vector<cv::Point2i> &object_cells);
    cv::Mat drawMatrix();
    cv::Mat drawMap(const cv::Mat &map,
                    const std::vector<float> &horizontal_distances,
                    const std::vector<float> &vertical_distances);
};

#endif
