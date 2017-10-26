
// Copyright Roberto Capobianco, 2016

#include "semantic_map_extraction/semantic_matrix.hpp"

#include <set>

void SemanticMatrix::addCellValues(int row, int col,
                                   std::string object_name,
                                   std::string additionals,
                                   bool change_adjacency,
                                   direction adjacency_dir) {
    if (row < 0 || row >= rows || col < 0 || col >= cols) {
        ROS_FATAL("Out of matrix");
        throw "Out of the matrix";
    }

    Cell* elem = &(semantic_matrix[row][col]);

    if (change_adjacency) {
        std::vector<bool> adjacency = elem->getDoorAdjacency();
        adjacency[adjacency_dir] = true;
        elem->setDoorAdjacency(adjacency);
    }

    if (!object_name.empty()) {
        std::vector<std::string> objects_vec = elem->getObjects();
        objects_vec.push_back(object_name);
        elem->setObjects(objects_vec);
    }

    if (!additionals.empty()) {
        std::vector<std::string> additionals_vec = elem->getAdditionals();
        additionals_vec.push_back(additionals);
        elem->setAdditionals(additionals_vec);
    } else if (!object_name.empty()) {
        std::vector<std::string> additionals_vec = elem->getAdditionals();
        additionals_vec.push_back("0");
        elem->setAdditionals(additionals_vec);
    }
}

const Cell& SemanticMatrix::retrieveCellValues(int row, int col){
    return semantic_matrix[row][col];
}

void SemanticMatrix::setDoor(
        const std::vector<cv::Point2i> &door_cells, Objects &door) {
    if (door_cells.size() != 2)
        throw "Door must be defined by two points";


    direction dir = door.getDir();
    std::string name = door.getName();
    std::string additionals = door.getProperties();

    for (int idy = std::min(door_cells[0].y, door_cells[1].y);
         idy <= std::max(door_cells[0].y, door_cells[1].y); ++idy) {
        for (int idx = std::min(door_cells[0].x, door_cells[1].x);
             idx <= std::max(door_cells[0].x, door_cells[1].x); ++idx) {
            addCellValues(idy, idx, name, additionals, true, dir);

            switch (dir) {
            case UP:
                if (idy > 0)
                    addCellValues(idy - 1, idx, name, additionals, true, DOWN);
                break;
            case RIGHT:
                if (idx < cols - 1)
                    addCellValues(idy, idx + 1, name, additionals, true, LEFT);
                break;
            case DOWN:
                if (idy < rows - 1)
                    addCellValues(idy + 1, idx, name, additionals, true, UP);
                break;
            case LEFT:
                if (idx > 0)
                    addCellValues(idy, idx - 1, name, additionals, true, RIGHT);
                break;
            }
        }
    }
}

void SemanticMatrix::setObject(const std::vector<cv::Point2i> &object_cells, Objects &object) {

    if (object_cells.size() != 4)
        throw "Objects must be defined by four points";

    int min_x = object_cells[0].x;
    int max_x = object_cells[0].x;
    int min_y = object_cells[0].y;
    int max_y = object_cells[0].y;
    std::string name = object.getName();
    std::string additionals = object.getProperties();

    for (std::vector<cv::Point2i>::const_iterator i = object_cells.begin() + 1;
         i != object_cells.end(); ++i) {
        min_x = std::min(min_x, i->x);
        max_x = std::max(max_x, i->x);
        min_y = std::min(min_y, i->y);
        max_y = std::max(max_y, i->y);
    }

    for (int idy = min_y; idy <= max_y; idy++)
        for (int idx = min_x; idx <= max_x; idx++)
            addCellValues(idy, idx, name, additionals);
}

std::vector<std::string> SemanticMatrix::getObject(const std::vector<cv::Point2i> &object_cells){

    if (object_cells.size() != 4)
        throw "Objects must be defined by four points";

    std::set<std::string> objects_set;

    int min_x = object_cells[0].x;
    int max_x = object_cells[0].x;
    int min_y = object_cells[0].y;
    int max_y = object_cells[0].y;

    for (std::vector<cv::Point2i>::const_iterator i = object_cells.begin() + 1;
         i != object_cells.end(); ++i) {
        min_x = std::min(min_x, i->x);
        max_x = std::max(max_x, i->x);
        min_y = std::min(min_y, i->y);
        max_y = std::max(max_y, i->y);
    }

    for (int idy = min_y; idy <= max_y; idy++)
        for (int idx = min_x; idx <= max_x; idx++){
            Cell cell = retrieveCellValues(idy, idx);
            std::vector<std::string> cell_objects = cell.getObjects();
            for(std::vector<std::string>::iterator it = cell_objects.begin();
                it != cell_objects.end();
                ++it)
                objects_set.insert(*it);
        }

    std::vector<std::string> objects;
    for(std::set<std::string>::iterator it = objects_set.begin();
        it != objects_set.end();
        ++it)
        objects.push_back(*it);

    return objects;
}

cv::Mat SemanticMatrix::drawMatrix() {
    cv::Mat matrix(rows, cols, CV_8UC3);

    std::vector<std::string> tags;
    std::vector<cv::Vec3b> colorTab;

    for (int i=0; i < rows; i++) {
        for (int j=0; j < cols; j++) {
            Cell elem = semantic_matrix[i][j];
            std::string buffer = elem.getRoomTag();

            if (!buffer.empty()) {
                std::vector<std::string>::iterator it;
                int index = 0;
                for (it = tags.begin();
                     it != tags.end() && (*it).compare(buffer) != 0; ++it, ++index) {}

                if (it == tags.end()) {
                    tags.push_back(buffer);
                    uchar b, g, r;
                    int diff;

                    do {
                        b = (uchar)cv::theRNG().uniform(200, 240);
                        g = (uchar)cv::theRNG().uniform(190, 255);
                        r = (uchar)cv::theRNG().uniform(200, 220);

                        diff = 1000;
                        for (std::vector<cv::Vec3b>::iterator i = colorTab.begin();
                             i != colorTab.end(); ++i)
                            diff = std::min(std::abs((*i)[0] - b)
                                    + std::abs((*i)[1] - g)
                                    + std::abs((*i)[2] - r),
                                    diff);
                    } while (diff < 20);

                    colorTab.push_back(cv::Vec3b(b, g, r));
                }

                if (elem.getDoorAdjacency()[0]
                        || elem.getDoorAdjacency()[1]
                        || elem.getDoorAdjacency()[2]
                        || elem.getDoorAdjacency()[3]) {
                    matrix.at<cv::Vec3b>(i, j) = cv::Vec3b(240, 0, 0);
                } else if (!elem.getObjects().empty()) {
                    matrix.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 240);
                } else {
                    matrix.at<cv::Vec3b>(i, j) = cv::Vec3b(colorTab[index]);
                }
            } else {
                if (elem.getDoorAdjacency()[0]
                        || elem.getDoorAdjacency()[1]
                        || elem.getDoorAdjacency()[2]
                        || elem.getDoorAdjacency()[3]) {
                    matrix.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
                    // cv::Vec3b(240,0,0);
                } else if (!elem.getObjects().empty()) {
                    matrix.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 240);
                } else {
                    matrix.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
                }
            }
        }
    }

    return matrix;
}


cv::Mat SemanticMatrix::drawMap(
        const cv::Mat &map,
        const std::vector<float> &horizontal_distances,
        const std::vector<float> &vertical_distances) {
    float width = 0;
    float height = 0;
    cv::Mat image;
    bool redraw = false;

    if (map.empty()) {
        redraw = true;

        for (std::vector<float>::const_iterator i = horizontal_distances.begin();
             i != horizontal_distances.end(); i++) {
            width += *i;
        }

        for (std::vector<float>::const_iterator i = vertical_distances.begin();
             i != vertical_distances.end(); i++) {
            height += *i;
        }

        image = cv::Mat(height, width, CV_8UC3);
    } else {
        image = map.clone();
        cv::cvtColor(image, image, CV_GRAY2BGR);
    }

    std::vector<std::string> tags;
    std::vector<cv::Vec3b> colorTab;
    std::map<std::string, float> obj_size_x;
    std::map<std::string, float> obj_size_y;

    for (int i=0; i < rows; i++) {
        for (int j=0; j < cols; j++) {
            Cell elem = semantic_matrix[i][j];
            std::string buffer = elem.getRoomTag();

            float x_init, x_end, y_init, y_end;
            cv::Vec3b color;

            y_init = 0;

            for (int k = 0; k < i; ++k) {
                y_init += vertical_distances[k];
            }

            x_init = 0;

            for (int k = 0; k < j; ++k) {
                x_init += horizontal_distances[k];
            }

            y_end = y_init + vertical_distances[i];
            x_end = x_init + horizontal_distances[j];

            int index = 0;

            if (!buffer.empty()) {
                std::vector<std::string>::iterator it;

                for (it = tags.begin(); it != tags.end() && (*it).compare(buffer) != 0;
                     ++it, ++index) {}

                if (it == tags.end()) {
                    tags.push_back(buffer);
                    uchar b, g, r;
                    int diff;

                    do {
                        b = (uchar)cv::theRNG().uniform(200, 240);
                        g = (uchar)cv::theRNG().uniform(190, 255);
                        r = (uchar)cv::theRNG().uniform(200, 220);

                        diff = 1000;
                        for (std::vector<cv::Vec3b>::iterator i = colorTab.begin();
                             i != colorTab.end(); i++)
                            diff = std::min(std::abs((*i)[0] - b)
                                    + std::abs((*i)[1] - g)
                                    + std::abs((*i)[2] - r),
                                    diff);
                    } while (diff < 20);

                    colorTab.push_back(cv::Vec3b(b, g, r));
                }
            }

            if (elem.getDoorAdjacency()[0]) {
                cv::line(image, cv::Point(x_init, y_init),
                         cv::Point(x_end, y_init), cv::Scalar(240, 0, 0), 2);
            } else if (elem.getDoorAdjacency()[1]) {
                cv::line(image, cv::Point(x_end, y_init),
                         cv::Point(x_end, y_end), cv::Scalar(240, 0, 0), 2);
            } else if (elem.getDoorAdjacency()[2]) {
                cv::line(image, cv::Point(x_init, y_end),
                         cv::Point(x_end, y_end), cv::Scalar(240, 0, 0), 2);
            } else if (elem.getDoorAdjacency()[3]) {
                cv::line(image, cv::Point(x_init, y_init),
                         cv::Point(x_init, y_end), cv::Scalar(240, 0, 0), 2);
            } else if (!elem.getObjects().empty()) {
                cv::rectangle(image,
                              cv::Point(x_init, y_init),
                              cv::Point(x_end, y_end),
                              cv::Scalar(0, 0, 240),
                              CV_FILLED);
            } else {
                if (redraw) {
                    if (!buffer.empty()) {
                        color = cv::Vec3b(colorTab[index]);
                    } else {
                        color = cv::Vec3b(190, 190, 190);
                    }

                    cv::rectangle(image, cv::Point(x_init, y_init),
                                  cv::Point(x_end, y_end), cv::Scalar(color[0],
                                  color[1],
                            color[2]),
                            CV_FILLED);
                }
            }
        }
    }

    return image;
}

SemanticMatrix::SemanticMatrix(const int r, const int c) {
    rows = r;
    cols = c;
    semantic_matrix.assign(rows, std::vector<Cell>(cols));
}

Cell* SemanticMatrix::getCellByIndexes(const int row, const int col) {
    return &semantic_matrix[row][col];
}

void SemanticMatrix::setRoomTags(
        const std::vector<std::pair<std::string, cv::Point> > &tags,
        RoomMap &room_map,
        const std::vector<float> &check_x,
        const std::vector<float> &check_y) {
    for (std::vector<std::pair<std::string, cv::Point> >::const_iterator
         it = tags.begin(); it != tags.end(); ++it) {
        cv::Mat check = room_map.getRoomByTag((*it).second);
        int row = 0;

        for (std::vector<float>::const_iterator i = check_y.begin();
             i != check_y.end(); ++i, ++row) {
            int col = 0;

            for (std::vector<float>::const_iterator j = check_x.begin();
                 j != check_x.end(); ++j, ++col) {
                if (check.at<uchar>(cv::Point(*j, *i)) > 200)
                    semantic_matrix[row][col].setRoomTag((*it).first);
            }
        }
    }
}

void SemanticMatrix::setObjectTags(const std::vector<std::vector<cv::Point2i> > &cell_objects,
                                   std::vector<Objects> &objects) {
    assert(cell_objects.size() == objects.size());

    std::vector<std::vector<cv::Point2i> >::const_iterator j = cell_objects.begin();

    for (std::vector<Objects>::iterator i = objects.begin(); i != objects.end(); ++i, ++j) {
        if (i->getType() == DOOR)
            setDoor(*j, *i);
        else
            setObject(*j, *i);
    }
}

std::vector<std::string> SemanticMatrix::getObjectTags(const std::vector<std::vector<cv::Point2i> > &cell_objects){

    std::set<std::string> objects_set;

    for(std::vector<std::vector<cv::Point2i> >::const_iterator it = cell_objects.begin();
        it != cell_objects.end();
        ++it){
        std::vector<std::string> objects = getObject(*it);

        for(std::vector<std::string>::iterator jt = objects.begin();
            jt != objects.end();
            ++jt)
            objects_set.insert(*jt);
    }

    std::vector<std::string> output_objects;
    for(std::set<std::string>::iterator it = objects_set.begin();
        it != objects_set.end();
        ++it)
        output_objects.push_back(*it);

    return output_objects;
}

void SemanticMatrix::removeObjectByName(const std::string object_name) {
    for (int i=0; i < rows; i++) {
        for (int j=0; j < cols; j++) {
            Cell* elem = &(semantic_matrix[i][j]);

            std::vector<std::string> objects_vec
                    = elem->getObjects();
            std::vector<std::string>::iterator position
                    = std::remove(objects_vec.begin(), objects_vec.end(), object_name);

            if (position != objects_vec.end()) {
                objects_vec.erase(position);
                elem->setObjects(objects_vec);
            }
        }
    }
}

void SemanticMatrix::removeObjectAdditionals(const std::string additionals) {
    for (int i=0; i < rows; i++) {
        for (int j=0; j < cols; j++) {
            Cell* elem = &(semantic_matrix[i][j]);

            std::vector<std::string> additionals_vec
                    = elem->getAdditionals();
            std::vector<std::string>::iterator position
                    = std::remove(additionals_vec.begin(),
                                  additionals_vec.end(),
                                  additionals);

            if (position != additionals_vec.end()) {
                additionals_vec.erase(position);
                elem->setAdditionals(additionals_vec);
            }
        }
    }
}

void SemanticMatrix::showMatrixImage() {
    cv::Mat matrix = drawMatrix();

    cv::namedWindow("semantic_matrix", CV_WINDOW_NORMAL);
    cv::imshow("semantic_matrix", matrix);
}

void SemanticMatrix::showMapImage(
        const cv::Mat &map, const std::vector<float> &horizontal_distances,
        const std::vector<float> &vertical_distances) {
    cv::Mat image = drawMap(map, horizontal_distances, vertical_distances);

    cv::namedWindow("semantic_map", CV_WINDOW_NORMAL);
    cv::imshow("semantic_map", image);
}

std::string SemanticMatrix::saveMatrixImage(
        std::string path, std::string basename) {
    cv::Mat src = drawMatrix().clone(), dst;
    cv::Size s(src.size().width*10, src.size().height*10);
    cv::resize(src, dst, s, 0, 0, CV_INTER_AREA);

    char buffer[80];
    std::time_t rawtime;
    std::time(&rawtime);
    struct std::tm* timeinfo = std::localtime(&rawtime);
    std::strftime(buffer, 80, "%F_%T", timeinfo);

    if (!cv::imwrite(path + "/" + basename + "_semantic_matrix_"
                     + std::string(buffer) + ".png", dst)) {
        throw "Cannot save semantic matrix image";
    }

    return path + "/" + basename + "_semantic_matrix_"
            + std::string(buffer) + ".png";
}

std::string SemanticMatrix::saveMapImage(
        const cv::Mat &map,
        const std::vector<float> &horizontal_distances,
        const std::vector<float> &vertical_distances,
        std::string path,
        std::string basename) {
    cv::Mat src = drawMap(map, horizontal_distances,
                          vertical_distances).clone();
    cv::Mat dst;

    char buffer[80];
    std::time_t rawtime;
    std::time(&rawtime);
    struct std::tm* timeinfo = std::localtime(&rawtime);
    std::strftime(buffer, 80, "%F_%T", timeinfo);

    if (!cv::imwrite(path + "/" + basename
                     + "_semantic_map_" + std::string(buffer) + ".png", src)) {
        throw "Cannot save semantic map image";
    }

    return path + "/" + basename + "_semantic_map_"
            + std::string(buffer) + ".png";
}

std::vector<std::string> SemanticMatrix::msg_format() {
    std::vector<std::string> msg;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            msg.push_back((semantic_matrix[i][j]).toString());
        }
    }

    return msg;
}

