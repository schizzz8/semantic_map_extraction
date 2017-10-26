// Copyright Roberto Capobianco, 2016

#include "semantic_map_extraction/semantic_map_extraction.hpp"

#include <algorithm>

void SemanticNode::param_init(bool first_run) {
    if (first_run) {
        this->resolution = -1;
        this->origin_x = 0;
        this->origin_y = 0;
    }

    this->buffer.time = ros::Time(0);

    while (ros::Time::now() == ros::Time(0)) {}

    this->grid_map = NULL;
    this->room_map = NULL;
    this->semantic_matrix = NULL;
    this->sem_gui = new GUI("Semantic map");
    this->listener = new tf::TransformListener();

    std::ifstream ifile1(std::string(this->StatXMLFilePath).c_str());
    std::ifstream ifile2(std::string(this->DynXMLFilePath).c_str());

    if (!this->StatXMLFilePath.empty() && ifile1)
        this->loadXMLMap(this->StatXMLFilePath);

    if (!this->DynXMLFilePath.empty() && ifile2 && this->load_dyn_map)
        this->loadXMLMap(this->DynXMLFilePath);
}

// to impose an area value also to cells outside the map (mislocalized objects)
void SemanticNode::check_object_area(std::vector<cv::Point2i> cell) {
    tf::StampedTransform transform;

    try {
        listener->waitForTransform("map",
                                   this->robotname + this->base_frame_id,
                                   ros::Time(0),
                                   ros::Duration(3));
        listener->lookupTransform("map",
                                  this->robotname + this->base_frame_id,
                                  ros::Time(0),
                                  transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }

    float x_robot = transform.getOrigin().x();
    float y_robot = transform.getOrigin().y();

    x_robot = (this->resolution != -1)
            ? (x_robot - this->origin_x)/this->resolution : x_robot;
    y_robot = (this->resolution != -1)
            ? this->image.size().height
              - (y_robot - this->origin_y)/this->resolution : y_robot;

    cv::Point2i robot_cell = (this->grid_map)->getCellFromImageCoords(
                cv::Point2i(x_robot, y_robot));

    for (std::vector<cv::Point2i>::iterator
         i = cell.begin(); i != cell.end(); i++) {
        try {
            if (((this->semantic_matrix)->
                 getCellByIndexes(i->y, i->x))->getRoomTag().empty()) {
                ((this->semantic_matrix)->
                        getCellByIndexes(i->y, i->x))->
                        setRoomTag(((this->semantic_matrix)->
                                    getCellByIndexes(robot_cell.y,
                                                     robot_cell.x))->getRoomTag());

                this->imposed_tags.push_back(
                            std::make_pair(
                                (
                                    (this->semantic_matrix)->
                                    getCellByIndexes(
                                        robot_cell.y,
                                        robot_cell.x))->getRoomTag(), cv::Point(i->x, i->y)));
            }
        }
        catch(char const* str) {
            ROS_ERROR("%s", str);
        }
    }
}

void SemanticNode::areasUpdate() {
    std::vector<cv::Point> tag_coords;

    for (std::vector<std::pair<std::string, cv::Point> >::const_iterator
         it = this->tags.begin(); it != this->tags.end(); it++)
        tag_coords.push_back((*it).second);

    (this->room_map)->extractRooms(tag_coords);

    try {
        (this->semantic_matrix)->setRoomTags(
                    this->tags, *this->room_map,
                    this->horizontal_cell_centers, this->vertical_cell_centers);
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }

    for (std::vector<std::pair<std::string, cv::Point> >::const_iterator
         it = this->imposed_tags.begin();
         it != this->imposed_tags.end(); it++) {
        try {
            ((this->semantic_matrix)->getCellByIndexes(
                        it->second.y,
                        it->second.x))->setRoomTag(it->first);
        }
        catch(char const* str) {
            ROS_ERROR("%s", str);
        }
    }
}

void SemanticNode::callService() {
    ros::ServiceClient client = n->serviceClient<
            semantic_map_extraction::GenerateKB>("/create_kb");
    semantic_map_extraction::GenerateKB msg;
    msg.request.width = this->horizontal_distances.size();
    msg.request.height = this->vertical_distances.size();

    std::vector<float> real_world_cell_centers_x, real_world_cell_centers_y;

    if (this->resolution != -1) {
        for (std::vector<float>::iterator i = this->horizontal_cell_centers.begin();
             i != this->horizontal_cell_centers.end(); i++) {
            real_world_cell_centers_x.push_back(
                        (*i)*this->resolution + this->origin_x);
        }

        for (std::vector<float>::iterator i = this->vertical_cell_centers.begin();
             i != this->vertical_cell_centers.end(); i++) {
            real_world_cell_centers_y.push_back(
                        -(*i - this->image.size().height)*this->resolution + this->origin_y);
        }
    } else {
        real_world_cell_centers_x = this->horizontal_cell_centers;
        real_world_cell_centers_y = this->vertical_cell_centers;
    }

    msg.request.cell_centers_x = real_world_cell_centers_x;
    msg.request.cell_centers_y = real_world_cell_centers_y;

    std::cout << "Matrix Width: " << this->horizontal_distances.size()
              << ", height " << this->vertical_distances.size()
              << ", total cells "
              << this->horizontal_distances.size()*this->vertical_distances.size()
              << std::endl;

    std::cout << "Image width: " << this->image.cols
              << ", image height " << this->image.rows
              << ", total pixels " << this->image.cols*this->image.rows
              << std::endl;

    msg.request.matrix = (this->semantic_matrix)->msg_format();
    std::vector<std::string> t = (this->semantic_matrix)->msg_format();

    /*for (std::vector<std::string>::iterator i=t.begin(); i!= t.end(); i++)
    std::cout << *i << "&";*/

    if (this->wait_service)
        ros::service::waitForService("/create_kb");

    if (client.call(msg))
        ROS_INFO("Semantic matrix sent");
    else
        ROS_INFO("Failed to call service create_kb");
}

void SemanticNode::loadXMLMap(std::string XMLFilePath) {
    TiXmlDocument xml(XMLFilePath);

    if (xml.LoadFile()) {
        ROS_INFO("XML file %s succesfully loaded", XMLFilePath.c_str());

        TiXmlHandle XMLHandle(&xml);
        TiXmlElement* elem;
        TiXmlHandle RootHandle(0);

        RootHandle = XMLHandle.FirstChildElement().Element();
        elem = RootHandle.FirstChild("areas").FirstChild().Element();

        for (; elem; elem = elem->NextSiblingElement()) {
            float x = 0, y = 0;
            const char *tag = elem->Attribute("name");
            elem->QueryFloatAttribute("x", &x);
            elem->QueryFloatAttribute("y", &y);

            if (tag) {
                this->tags.push_back(std::make_pair(std::string(tag), cv::Point(x, y)));

                if (XMLFilePath.compare(this->StatXMLFilePath) == 0)
                    this->static_tags.push_back(true);
                else
                    this->static_tags.push_back(false);
            }
        }

        elem = RootHandle.FirstChild("imposed_areas").FirstChild().Element();

        for (; elem; elem = elem->NextSiblingElement()) {
            float x = 0, y = 0;
            const char *tag = elem->Attribute("name");
            elem->QueryFloatAttribute("x", &x);
            elem->QueryFloatAttribute("y", &y);

            if (tag)
                this->imposed_tags.push_back(
                        std::make_pair(std::string(tag), cv::Point(x, y)));
        }

        elem = RootHandle.FirstChild("objects").FirstChild().Element();

        for (; elem; elem = elem->NextSiblingElement()) {
            float x = 0, y = 0, theta = 0, dimX = 0, dimY = 0, dimZ = 0;
            int object_type = 0;
            const char *obj = elem->Attribute("name");
            elem->QueryFloatAttribute("x", &x);
            elem->QueryFloatAttribute("y", &y);
            elem->QueryFloatAttribute("theta", &theta);
            elem->QueryFloatAttribute("dimX", &dimX);
            elem->QueryFloatAttribute("dimY", &dimY);
            elem->QueryFloatAttribute("dimZ", &dimZ);
            elem->QueryIntAttribute("type", &object_type);
            const char *properties = elem->Attribute("properties");

            if (obj && properties) {
                this->objects.push_back(
                            Objects(obj,
                                    cv::Point2i(x, y),
                                    theta,
                                    cv::Vec3f(dimX, dimY, dimZ),
                                    properties,
                                    (type) object_type));

                if (XMLFilePath.compare(this->StatXMLFilePath) == 0)
                    this->static_objects.push_back(true);
                else
                    this->static_objects.push_back(false);
            }
        }
    } else {
        ROS_ERROR("No XML file found: doesn't matter, going on...");
    }
}

void SemanticNode::saveXMLMap(std::string XMLFilePath) {
    TiXmlDocument xml;
    TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
    xml.LinkEndChild(decl);

    TiXmlElement* root = new TiXmlElement("semantic_map");
    xml.LinkEndChild(root);

    TiXmlComment* comment = new TiXmlComment();
    comment->SetValue(
                "Semantic map saved XML file. Coords are already image coordinates.");
    root->LinkEndChild(comment);

    TiXmlElement* areas = new TiXmlElement("areas");
    root->LinkEndChild(areas);

    int tidx = 0;
    for (std::vector<std::pair<std::string, cv::Point> >::iterator
         i = this->tags.begin(); i != this->tags.end(); i++) {
        if ((this->static_tags[tidx]
             && XMLFilePath.compare(this->StatXMLFilePath) == 0)
                || (!this->static_tags[tidx]
                    && XMLFilePath.compare(this->DynXMLFilePath) == 0)) {
            TiXmlElement* tag;
            cv::Point pt = i->second;

            tag = new TiXmlElement("tag");
            areas->LinkEndChild(tag);

            tag->SetAttribute("name", i->first);
            tag->SetDoubleAttribute("x", pt.x);
            tag->SetDoubleAttribute("y", pt.y);
        }

        tidx++;
    }

    TiXmlElement* imposed_areas = new TiXmlElement("imposed_areas");
    root->LinkEndChild(imposed_areas);

    for (std::vector<std::pair<std::string, cv::Point> >::iterator
         i = this->imposed_tags.begin(); i != this->imposed_tags.end(); i++) {
        TiXmlElement* tag;
        cv::Point pt = i->second;

        tag = new TiXmlElement("tag");
        imposed_areas->LinkEndChild(tag);

        tag->SetAttribute("name", i->first);
        tag->SetDoubleAttribute("x", pt.x);
        tag->SetDoubleAttribute("y", pt.y);
    }

    TiXmlElement* objs = new TiXmlElement("objects");
    root->LinkEndChild(objs);

    int idx = 0;
    for (std::vector<Objects>::iterator i = this->objects.begin();
         i != this->objects.end(); i++) {
        if ((this->static_objects[idx]
             && XMLFilePath.compare(this->StatXMLFilePath) == 0)
                || (!this->static_objects[idx]
                    && XMLFilePath.compare(this->DynXMLFilePath) == 0)) {
            TiXmlElement* obj;
            cv::Point pt = i->getCoords();
            cv::Vec3f dims = i->getDims();

            obj = new TiXmlElement("object");
            objs->LinkEndChild(obj);

            obj->SetAttribute("name", i->getName());
            obj->SetDoubleAttribute("x", pt.x);
            obj->SetDoubleAttribute("y", pt.y);
            obj->SetDoubleAttribute("theta", i->getAngle());
            obj->SetDoubleAttribute("dimX", dims[0]);
            obj->SetDoubleAttribute("dimY", dims[1]);
            obj->SetDoubleAttribute("dimZ", dims[2]);
            obj->SetAttribute("type", i->getType());
            obj->SetAttribute("properties", i->getProperties());
        }

        idx++;
    }

    ROS_INFO("XML file succesfully produced: saving...");
    char buffer[80];
    std::time_t rawtime;
    std::time(&rawtime);
    struct std::tm* timeinfo = std::localtime(&rawtime);
    std::strftime(buffer, 80, "%F_%T", timeinfo);

    std::ostringstream oss;
    oss << XMLFilePath << "~" << buffer;

    xml.SaveFile(XMLFilePath);
    xml.SaveFile(oss.str());
}

SemanticNode::SemanticNode(ros::NodeHandle *n,
                           std::string robotname,
                           std::string base_frame_id,
                           std::string path,
                           std::string StatXMLFilePath,
                           std::string DynXMLFilePath,
                           int timeout,
                           bool wait_service,
                           bool load_dyn_map) {
    this->n = n;
    this->robotname = robotname;
    this->base_frame_id = base_frame_id;
    this->path = path;
    this->StatXMLFilePath = StatXMLFilePath;
    this->DynXMLFilePath = DynXMLFilePath;
    this->timeout = timeout;
    this->wait_service = wait_service;
    this->load_dyn_map = load_dyn_map;
    this->param_init();
}

SemanticNode::SemanticNode(
        ros::NodeHandle *n,
        const cv::Mat &image,
        std::string robotname,
        std::string base_frame_id,
        std::string path,
        std::string StatXMLFilePath,
        std::string DynXMLFilePath,
        int timeout,
        bool wait_service,
        bool load_dyn_map) {
    this->n = n;
    this->robotname = robotname;
    this->base_frame_id = base_frame_id;
    this->image = image.clone();
    this->path = path;
    this->StatXMLFilePath = StatXMLFilePath;
    this->DynXMLFilePath = DynXMLFilePath;
    this->timeout = timeout;
    this->wait_service = wait_service;
    this->load_dyn_map = load_dyn_map;
    this->param_init();
}

SemanticNode::~SemanticNode() {
    delete this->semantic_matrix;
    delete this->room_map;
    delete this->grid_map;
}

void SemanticNode::areaTagsProcessing() {
    std::vector<std::pair<cv::Point2i, cv::Point2i> > doors_to_process;

    for (std::vector<Objects>::iterator
         i = this->objects.begin(); i != this->objects.end(); i++)
        if (i->getType() == DOOR) {
            std::vector<cv::Point2i> vert = i->getVertexCoords();
            doors_to_process.push_back(std::make_pair(vert[0], vert[1]));
        }

    (this->room_map)->addDoors(doors_to_process);

    this->areasUpdate();
}

void SemanticNode::objectTagsProcessing() {
    std::vector<std::vector<cv::Point2i> > cell_objects;

    for (std::vector<Objects>::iterator
         i = this->objects.begin(); i != this->objects.end(); i++) {
        std::vector<cv::Point2i> vert = i->getVertexCoords();

        std::vector<cv::Point2i> cell;
        for (std::vector<cv::Point2i>::iterator
             it = vert.begin(); it != vert.end(); it++)
            cell.push_back((this->grid_map)->getCellFromImageCoords(*it));

        cell_objects.push_back(cell);
    }

    try {
        (this->semantic_matrix)->setObjectTags(cell_objects, objects);
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }

    this->callService();
}

void SemanticNode::addElement() {
    char buffer[80];
    std::time_t rawtime;
    std::time(&rawtime);
    struct std::tm* timeinfo = std::localtime(&rawtime);
    std::strftime(buffer, 80, "%F_%T", timeinfo);

    std::ostringstream oss;
    oss << this->buffer.name << "~" << buffer;

    Objects ob(oss.str(),
               this->buffer.pt,
               this->buffer.angle,
               this->buffer.size,
               this->buffer.properties,
               this->buffer.ob_type);

    this->objects.push_back(ob);
    this->static_objects.push_back(false);
    std::vector<cv::Point2i> vert = ob.getVertexCoords();

    std::vector<cv::Point2i> cell;
    for (std::vector<cv::Point2i>::iterator it = vert.begin(); it != vert.end(); it++)
        cell.push_back((this->grid_map)->getCellFromImageCoords(*it));

    std::vector<Objects> obj = std::vector<Objects>(1, ob);
    std::vector<std::vector<cv::Point2i> > cell_obj = std::vector<std::vector<cv::Point2i> >(1, cell);

    try {
        std::vector<std::string> tags = (this->semantic_matrix)->getObjectTags(cell_obj);

        ROS_INFO("Retrieving objects for %s:",this->buffer.name.c_str());
        for(std::vector<std::string>::iterator ii = tags.begin();
            ii != tags.end();
            ++ii){
            std::cerr << *ii << std::endl;
        }
        ROS_INFO("...");


	bool found = false;
        for(std::vector<std::string>::iterator it = tags.begin();
            it != tags.end();
            ++it){
            std::string tag = *it;

	    if(tag.find(this->buffer.name) != std::string::npos)
	      found = true;
        }

        if(found){
            ROS_INFO("Object %s already inserted in the semantic map",this->buffer.name.c_str());
        } else {
            (this->semantic_matrix)->setObjectTags(cell_obj, obj);
        }
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }

    this->check_object_area(cell);
    ROS_INFO("Object added.");

    if (this->buffer.ob_type == DOOR) {
        (this->room_map)->addDoors(std::make_pair(vert[0], vert[1]));
        ROS_INFO("Door added.");
    }

    this->saveImages();
    this->callService();
    this->saveXMLMap(this->DynXMLFilePath);
}

void SemanticNode::mapProcessing() {
    float accumulator;

    ROS_INFO("Preparing data...");
    this->grid_map = new GridMap(this->image);
    this->room_map = new RoomMap(this->image);
    ROS_INFO("Grid computation.");
    (this->grid_map)->produce();
    ROS_INFO("Preparing semantic matrix...");
    this->vertical_distances = (this->grid_map)->getVerticalDistances();
    this->horizontal_distances = (this->grid_map)->getHorizontalDistances();

    for (std::vector<float>::iterator i = this->horizontal_distances.begin();
         i != this->horizontal_distances.end(); i++) {
        if (i == this->horizontal_distances.begin())
            accumulator = (*i)/2;
        else
            accumulator += *i;

        this->horizontal_cell_centers.push_back(accumulator);
    }

    for (std::vector<float>::iterator i = this->vertical_distances.begin();
         i != this->vertical_distances.end(); i++) {
        if (i == this->vertical_distances.begin())
            accumulator = (*i)/2;
        else
            accumulator += *i;

        this->vertical_cell_centers.push_back(accumulator);
    }

    semantic_matrix = new SemanticMatrix(
                this->vertical_distances.size(), this->horizontal_distances.size());
}

void SemanticNode::mapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int width = map->info.width;
    int height = map->info.height;

    this->image = cv::Mat(height, width, CV_8U);
    this->resolution = map->info.resolution;
    this->origin_x = map->info.origin.position.x;
    this->origin_y = map->info.origin.position.y;

    ROS_INFO("Map received.");

    for (int i = 0, i_rev = height - 1; i < height; i++, i_rev--)
        for (int j = 0; j < width; j++)
            switch (map->data[i_rev*width + j]) {
            default:
            case -1:
                this->image.data[i*width + j] = 150;
                break;
            case 0:
                this->image.data[i*width + j] = 255;
                break;
            case 100:
                this->image.data[i*width + j] = 0;
                break;
            }

    ROS_INFO("Image extracted from map.");
    this->mapProcessing();
    this->areaTagsProcessing();
    this->objectTagsProcessing();
    this->saveImages();
}

void SemanticNode::ObservationTopicSubscriber(const semantic_map_extraction::Obs::ConstPtr& observation) {
    float object_x = (this->resolution != -1)
            ? (observation->posx - this->origin_x)/this->resolution
            : observation->posx;
    float object_y = (this->resolution != -1)
            ? this->image.size().height
              - (observation->posy - this->origin_y)/this->resolution
            : observation->posy;
    float dim_x = (this->resolution != -1)
            ? observation->dimx/this->resolution : observation->dimx;
    float dim_y = (this->resolution != -1)
            ? observation->dimy/this->resolution : observation->dimy;
    float dim_z = (this->resolution != -1)
            ? observation->dimz/this->resolution : observation->dimz;

    this->buffer.pt = cv::Point2i(object_x, object_y);
    this->buffer.angle = observation->theta;
    this->buffer.size = cv::Vec3f(dim_x, dim_y, dim_z);

    // DEBUG //
    std::cerr << "Received POSX=" << observation->posx
              << ", POSY=" << observation->posy
              << ", THETA=" << observation->theta
              << ", DIMS (X,Y,Z) = (" << observation->dimx
              << ", " << observation->dimy << ", " << observation->dimz << ")" << std::endl;

    std::cerr << "Converted image coords POSX=" << object_x
              << ",POSY=" << object_y
              << ", THETA=" << observation->theta
              << ", DIMS (X,Y,Z) = (" << dim_x
              << ", " << dim_y
              << ", " << dim_z << ") " << std::endl;
    std::cerr << "PARAMS = (" << observation->properties << ") " << std::endl;

    std::string properties = observation->properties;
    std::replace(properties.begin(), properties.end(), '#', '_');
    this->buffer.properties = properties;

    if (this->buffer.time != ros::Time(0)
            && (ros::Time::now() - this->buffer.time).toSec()
            < this->timeout && buffer.last_writer.compare("obs") != 0) {
        this->addElement();
        this->buffer.time = ros::Time(0);
        this->buffer.last_writer = "obs";
    } else {
        this->buffer.time = ros::Time::now();
        this->buffer.last_writer = "obs";
    }
}

void SemanticNode::ASRSubscriber(const std_msgs::String::ConstPtr& string) {
    std::string tag = string->data;
    size_t pos_start = tag.find("(\"");
    size_t pos_end = tag.find("\")");

    try {
        if (pos_start == std::string::npos || pos_end == std::string::npos) {
            throw "Tag not recognized from ASR";
        }

        this->buffer.name = tag.substr(pos_start + 2, pos_end - pos_start - 2);
        std::replace(this->buffer.name.begin(), this->buffer.name.end(), ' ', '_');
        this->buffer.ob_type =
                (this->buffer.name.find("door") != std::string::npos ||
                this->buffer.name.find("DOOR") != std::string::npos)
                ? DOOR : NORMAL;

        if (this->buffer.time != ros::Time(0)
                && (ros::Time::now() - this->buffer.time).toSec()
                < this->timeout && buffer.last_writer.compare("asr") != 0) {
            this->addElement();
            this->buffer.time = ros::Time(0);
            this->buffer.last_writer = "asr";
        } else {
            this->buffer.time = ros::Time::now();
            this->buffer.last_writer = "asr";
        }
    }
    catch(char const* err) {
        ROS_ERROR("%s", err);
    }
}

void SemanticNode::UpdateObjectCoordsSubscriber(
        const std_msgs::String::ConstPtr& s) {
    std::string string = s->data;
    std::string delimiter = ":";

    size_t pos = string.find(delimiter);
    std::string object_name;

    if (pos != std::string::npos) {
        object_name = string.substr(0, pos);
        string.erase(0, pos + delimiter.length());
    }

    std::string coords = string;
    std::replace(coords.begin(), coords.end(), ',', ' ');

    std::stringstream ss(coords);
    float x, y;
    ss >> x >> y;

    float object_x = (this->resolution != -1)
            ? (x - this->origin_x)/this->resolution
            : x;
    float object_y = (this->resolution != -1)
            ? this->image.size().height
              - (y - this->origin_y)/this->resolution
            : y;

    ROS_INFO("Updating coords of object %s...", object_name.c_str());
    ROS_INFO("New coords: %s", coords.c_str());

    std::vector<Objects>::iterator i = this->objects.begin();

    for (; i != objects.end(); i++) {
        if (object_name == i->getName()) {
            i->updateCoords(cv::Point2i(object_x, object_y));
            (this->semantic_matrix)->removeObjectByName(object_name);
            (this->semantic_matrix)->removeObjectAdditionals(i->getProperties());
            std::vector<cv::Point2i> vert = i->getVertexCoords();

            std::vector<cv::Point2i> cell;
            for (std::vector<cv::Point2i>::iterator
                 it = vert.begin(); it != vert.end(); it++)
                cell.push_back((this->grid_map)->getCellFromImageCoords(*it));

            std::vector<Objects> obj = std::vector<Objects>(1, *i);
            std::vector<std::vector<cv::Point2i> >
                    cell_obj = std::vector<std::vector<cv::Point2i> >(1, cell);

            (this->semantic_matrix)->setObjectTags(cell_obj, obj);
            std::cout << "d" << std::endl;
        }
    }

    ROS_INFO("Object updated!");

    this->saveXMLMap(this->DynXMLFilePath);
    this->callService();
    this->saveImages();
}


void SemanticNode::UpdateObjectPropertySubscriber(
        const std_msgs::String::ConstPtr& s) {
    ROS_INFO("Updating map!");
    std::string string = s->data;
    std::string delimiter = ":";

    size_t pos = string.find(delimiter);
    std::string object_name;

    if (pos != std::string::npos) {
        object_name = string.substr(0, pos);
        string.erase(0, pos + delimiter.length());
    }

    std::string properties = string;
    std::replace(properties.begin(), properties.end(), '#', '_');

    ROS_INFO("Updating properties of object %s...", object_name.c_str());
    ROS_INFO("New properties: %s", properties.c_str());

    std::vector<Objects>::iterator i = this->objects.begin();

    for (; i != objects.end(); i++) {
        if (object_name == i->getName()) {
            (this->semantic_matrix)->removeObjectByName(object_name);
            (this->semantic_matrix)->removeObjectAdditionals(i->getProperties());

            i->updateProperties(properties);

            std::vector<cv::Point2i> vert = i->getVertexCoords();

            std::vector<cv::Point2i> cell;
            for (std::vector<cv::Point2i>::iterator
                 it = vert.begin(); it != vert.end(); it++)
                cell.push_back((this->grid_map)->getCellFromImageCoords(*it));

            std::vector<Objects> obj = std::vector<Objects>(1, *i);
            std::vector<std::vector<cv::Point2i> >
                    cell_obj = std::vector<std::vector<cv::Point2i> >(1, cell);

            (this->semantic_matrix)->setObjectTags(cell_obj, obj);
        }
    }

    ROS_INFO("Object updated!");

    this->saveXMLMap(this->DynXMLFilePath);
    this->callService();
    this->saveImages();
}


void SemanticNode::RemoveObjectSubscriber(
        const std_msgs::String::ConstPtr& string) {
    std::string object_name = string->data;

    ROS_INFO("Deleting object %s...", object_name.c_str());

    std::vector<Objects>::iterator i = this->objects.begin();

    for (; i != objects.end(); i++) {
        if (object_name == i->getName())
            break;
    }

    if (i == objects.end()) {
        throw "Object not found";
    }

    try {
        (this->semantic_matrix)->removeObjectByName(object_name);
        (this->semantic_matrix)->removeObjectAdditionals(i->getProperties());
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }


    if (i != objects.end())
        objects.erase(i);

    ROS_INFO("Object deleted!");

    this->saveXMLMap(this->DynXMLFilePath);
    this->callService();
    this->saveImages();
}


void SemanticNode::RobotPoseTimer(const ros::TimerEvent&) {
    tf::StampedTransform transform;

    float x_robot = 0;
    float y_robot = 0;
    float theta_robot = 0;

    if (this->resolution != -1) {
        try {
            this->listener->waitForTransform(
                        "map",
                        this->robotname + this->base_frame_id,
                        ros::Time(0), ros::Duration(1.0));
            this->listener->lookupTransform("map",
                                            this->robotname + this->base_frame_id,
                                            ros::Time(0),
                                            transform);
        }
        catch(tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }

        x_robot = (transform.getOrigin().x() - this->origin_x)/this->resolution;
        y_robot = this->image.size().height -
                (transform.getOrigin().y() - this->origin_y)/this->resolution;
        theta_robot = tf::getYaw(transform.getRotation());
    }

    if (grid_map != 0) {
        cv::Point2i cell(grid_map->getCellFromImageCoords(
                             cv::Point2i(x_robot, y_robot)));
        (this->sem_gui)->drawRobot(cell, theta_robot, horizontal_distances.size(),
                                   vertical_distances.size());
    }
}


bool SemanticNode::cellByCoordsHandler(
        semantic_map_extraction::GetCellByCoords::Request &req,
        semantic_map_extraction::GetCellByCoords::Response &res) {
    float coord_x, coord_y;

    coord_x = (this->resolution != -1) ?
                (req.real_x - this->origin_x)/this->resolution : req.real_x;
    coord_y = (this->resolution != -1) ?
                this->image.size().height - (req.real_y - this->origin_y)/this->resolution
              : req.real_y;

    cv::Point2i cell(grid_map->getCellFromImageCoords(
                         cv::Point2i(coord_x, coord_y)));

    res.cell_x = cell.x;
    res.cell_y = cell.y;

    return true;
}


bool SemanticNode::coordsByCellHandler(
        semantic_map_extraction::GetCoordsByCell::Request &req,
        semantic_map_extraction::GetCoordsByCell::Response &res) {
    cv::Point2i coords(grid_map->getImageCoordsFromCell(
                           cv::Point2i(req.cell_x, req.cell_y)));
    res.real_x = (this->resolution != -1) ?
                coords.x*this->resolution + this->origin_x
              : coords.x;
    res.real_y = (this->resolution != -1) ?
                -(coords.y - this->image.size().height)*this->resolution + this->origin_y
              : coords.y;

    return true;
}


bool SemanticNode::coordsByNameHandler(
        semantic_map_extraction::GetCoordsByName::Request &req,
        semantic_map_extraction::GetCoordsByName::Response &res) {
    cv::Point coords;
    bool found = false;

    for (std::vector<std::pair<std::string, cv::Point> >::iterator
         i = this->tags.begin(); i != this->tags.end(); i++) {
        if (i->first == req.name) {
            coords = i->second;
            found = true;
            break;
        }
    }

    if (!found) {
        for (std::vector<std::pair<std::string, cv::Point> >::iterator
             i = this->imposed_tags.begin();
             i != this->imposed_tags.end(); i++) {
            if (i->first == req.name) {
                coords = i->second;
                found = true;
                break;
            }
        }
    }

    if (!found) {
        for (std::vector<Objects>::iterator
             i = this->objects.begin();
             i != this->objects.end(); i++) {
            if (i->getName() == req.name) {
                std::vector<cv::Point2i> vert = i->getVertexCoords();
                coords = *(vert.begin());
                found = true;
                break;
            }
        }
    }

    if (found) {
        res.real_x = (this->resolution != -1) ?
                    coords.x*this->resolution + this->origin_x
                  : coords.x;
        res.real_y = (this->resolution != -1) ?
                    -(coords.y - this->image.size().height)*
                    this->resolution + this->origin_y
                  : coords.y;
    } else {
        res.real_x = 1e3;
        res.real_y = 1e3;
    }

    return true;
}


bool SemanticNode::deleteMapHandler(
        semantic_map_extraction::DeleteMap::Request &req,
        semantic_map_extraction::DeleteMap::Response &res) {
    if (std::remove(this->DynXMLFilePath.c_str()) == 0) {
        ROS_INFO("Map deleted: reprocessing everything.");
        delete this->semantic_matrix;
        delete this->room_map;
        delete this->grid_map;

        this->objects.clear();
        this->static_objects.clear();
        this->tags.clear();
        this->imposed_tags.clear();
        this->horizontal_distances.clear();
        this->vertical_distances.clear();
        this->horizontal_cell_centers.clear();
        this->vertical_cell_centers.clear();

        this->saveXMLMap(this->DynXMLFilePath);
        this->param_init(false);
        this->mapProcessing();
        this->areaTagsProcessing();
        this->objectTagsProcessing();

        this->saveImages();
        res.ack = true;
    } else {
        ROS_ERROR("Some problem occurred while deleting the map.");
        res.ack = false;
    }

    return true;
}


void SemanticNode::showImages() {
    ROS_INFO("Map stack showing...");
    (this->grid_map)->showStack();
    ROS_INFO("Semantic matrix showing...");
    try {
        (this->semantic_matrix)->showMatrixImage();
        (this->semantic_matrix)->showMapImage(
                    this->image, horizontal_distances,
                    vertical_distances);
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }

    cv::waitKey(1);
}

void SemanticNode::saveImages(std::string basename) {
    ROS_INFO("Map stack saving...");
    (this->grid_map)->saveStack(this->path, basename);
    ROS_INFO("Semantic matrix saving...");

    try {
        std::string image_tmp =
                (this->semantic_matrix)->saveMatrixImage(this->path, basename);
        (this->semantic_matrix)->saveMapImage(
                    this->image,
                    horizontal_distances,
                    vertical_distances,
                    this->path,
                    basename);
        (this->sem_gui)->setImage(image_tmp);
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "semantic_map_extraction_node");
    ros::NodeHandle n("~");
    std::string path;
    std::string robotname;
    std::string base_frame_id;
    std::string StatXMLFilePath;
    std::string DynXMLFilePath;
    int timeout;
    bool wait_service;
    bool load_dyn_map;

    n.param<std::string>("save_path", path, "./");
    n.param<std::string>("robot_name", robotname, "");
    n.param<std::string>("base_frame_id", base_frame_id, "base_footprint");
    n.param<std::string>("statXMLFilePath", StatXMLFilePath, "./mapXMLfile.xml");
    n.param<std::string>("dynXMLFilePath", DynXMLFilePath,
                         "./augmentedMapXMLfile.xml");
    n.param<int>("add_objects_timeout", timeout, 3000000);
    n.param<bool>("wait_prolog_service", wait_service, false);
    n.param<bool>("load_dynamic_map", load_dyn_map, false);
    ros::Subscriber sub_map;
    ros::Subscriber sub_obs;
    ros::Subscriber sub_asr;
    ros::Subscriber sub_remove_object_by_name;
    ros::Subscriber sub_update_object_properties_by_name;
    ros::Subscriber sub_update_object_coords_by_name;
    ros::ServiceServer srv_cell_by_coords;
    ros::ServiceServer srv_coords_by_cell;
    ros::ServiceServer srv_coords_by_name;
    ros::ServiceServer srv_delete_map;
    ros::Timer robot_pose_timer;

    SemanticNode* sem_node = NULL;

    std::ostringstream error_msg;

    error_msg << "Usage: semantic_map_extraction [MODALITY] [FILE]" << std::endl
              << "MODALITY: 0 if you want to use the map_server, 1 otherwhise"
              << std::endl
              << "FILE: used in MODALITY=1, represents the image to work on"
              << std::endl;

    if (argc < 2) {
        std::cerr << error_msg.str();
        exit(1);
    }

    int modality = 0;
    std::istringstream(argv[1]) >> modality;

    if ((argc < 3 && modality == 1) || (modality != 0 && modality != 1)) {
        std::cerr << error_msg.str();
        exit(1);
    }

    if (modality == 0) {
        sem_node = new SemanticNode(&n, robotname, base_frame_id, path, StatXMLFilePath,
                                    DynXMLFilePath, timeout, wait_service,
                                    load_dyn_map);

        sub_map = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000,
                                                       &SemanticNode::mapSubscriber,
                                                       sem_node);
    } else {
        std::string file = argv[2];
        std::ifstream f(file.c_str());

        if (!f.good()) {
            f.close();
            std::cerr << "File " << file << " does not exist!" << std::endl;
            exit(1);
        }

        f.close();

        cv::Mat src = cv::imread(file, 0);

        ROS_INFO("File uploaded.");

        sem_node = new SemanticNode(&n, src, robotname, base_frame_id, path, StatXMLFilePath,
                                    DynXMLFilePath, timeout, wait_service,
                                    load_dyn_map);
        sem_node->mapProcessing();
        sem_node->areaTagsProcessing();
        sem_node->objectTagsProcessing();
        sem_node->saveImages();
    }

    sub_obs = n.subscribe<semantic_map_extraction::Obs>(
                "/" + robotname + "/ObservationTopic",
                1000, &SemanticNode::ObservationTopicSubscriber, sem_node);
    sub_asr = n.subscribe<std_msgs::String>(
                "/" + robotname + "/ASR", 1000, &SemanticNode::ASRSubscriber, sem_node);
    sub_remove_object_by_name = n.subscribe<std_msgs::String>(
                "/remove_object_by_name", 1000, &SemanticNode::RemoveObjectSubscriber,
                sem_node);
    sub_update_object_properties_by_name = n.subscribe<std_msgs::String>(
                "/update_object_properties_by_name", 1000,
                &SemanticNode::UpdateObjectPropertySubscriber,
                sem_node);
    sub_update_object_coords_by_name = n.subscribe<std_msgs::String>(
                "/update_object_coords_by_name", 1000,
                &SemanticNode::UpdateObjectCoordsSubscriber,
                sem_node);
    srv_cell_by_coords = n.advertiseService("get_cell_by_coords",
                                            &SemanticNode::cellByCoordsHandler,
                                            sem_node);
    srv_coords_by_cell = n.advertiseService("get_coords_by_cell",
                                            &SemanticNode::coordsByCellHandler,
                                            sem_node);
    srv_coords_by_name = n.advertiseService("get_coords_by_name",
                                            &SemanticNode::coordsByNameHandler,
                                            sem_node);
    srv_delete_map = n.advertiseService("delete_map",
                                        &SemanticNode::deleteMapHandler,
                                        sem_node);

    robot_pose_timer = n.createTimer(ros::Duration(0.1),
                                     &SemanticNode::RobotPoseTimer,
                                     sem_node);

    ros::spin();

    return 0;
}
