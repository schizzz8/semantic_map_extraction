// ---------------------------------------------------------------------------
// ground_truth_creator.hpp
// Ground truth creator for semantic maps
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------

#ifndef _GROUND_TRUTH_CREATOR_HPP_
#define _GROUND_TRUTH_CREATOR_HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace ground_truth_creator {
	cv::Mat image;
	cv::Mat tmp_image;
	float resolution;
	float origin_x;
	float origin_y;
	bool drawing_box;
	cv::Rect box;
	std::string winname;
	std::ofstream gt_writer;
	std::string gt_filename;
	
	void start_gtc() {
		resolution = -1;
		origin_x = 0;
		origin_y = 0;
		drawing_box = false;
		winname = "Ground Truth Creator";
	}
	
	void mouseCallback(int event, int x, int y, int flags, void* param) {
		switch(event){
			case CV_EVENT_MOUSEMOVE: 
				if(drawing_box){
					box.width = x - box.x;
					box.height = y - box.y;
				}
			
				tmp_image = image.clone();
				cv::rectangle(tmp_image, cv::Point(box.x, box.y), cv::Point(box.x + box.width, box.y + box.height), cv::Scalar(0, 0, 255), CV_FILLED);
				cv::imshow(winname, tmp_image);
				cv::waitKey(0);
			
				break;

			case CV_EVENT_LBUTTONDOWN:
				drawing_box = true;
				box = cv::Rect(x, y, 0, 0);
				break;

			case CV_EVENT_LBUTTONUP:
				if(drawing_box) {
					drawing_box = false;
					float x_init, x_end, y_init, y_end, z_init, z_end;
					std::string object_type;
					std::cout << "Enter y_init (world coordinates) of the object: " << std::endl;
					std::cin >> y_init;
					std::cout << "Enter y_end (world coordinates) of the object: " << std::endl;
					std::cin >> y_end;
					std::cout << "Enter the type of the object (e.g., door, window, etc.): " << std::endl;
					std::cin >> object_type;
				
					cv::rectangle(image, cv::Point(box.x, box.y), cv::Point(box.x + box.width, box.y + box.height), cv::Scalar(0, 0, 255), CV_FILLED);
				
					if(resolution != -1) {
						x_init = box.x * resolution + origin_x;
						x_end = (box.x + box.width) * resolution + origin_x;
					
						//origin_y of the map is used with z, since it represents the depth and y represents the height
						z_init = -(box.y - image.size().height) * resolution + origin_y;
						z_end = -(box.y + box.height - image.size().height) * resolution + origin_y;
					}
					else {
						x_init = box.x;
						x_end = box.x + box.width;
					
						z_init = box.y;
						z_end = box.y + box.height;
					}
				
					gt_writer << object_type << "\t" << "x_init:" << x_init << "\t" << "x_end:" << x_end << "\t" << "y_init:" << y_init << "\t" << "y_end:" << y_end
																	 << "\t" << "z_init:" << z_init << "\t" << "z_end:" << z_end << std::endl;
				
					cv::imshow(winname, image);
					cv::imwrite(gt_filename + ".png", image);
					cv::waitKey(0);
				}
				
				break;
		}
	}

	void mapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& map) {
		int width = map->info.width;
		int height = map->info.height;

		image = cv::Mat(height, width, CV_8U);
		resolution = map->info.resolution;
		origin_x = map->info.origin.position.x;
		origin_y = map->info.origin.position.y;

		ROS_INFO("Map received.");

		for(int i = 0, i_rev = height - 1; i < height; i++, i_rev--) {
			for(int j = 0; j < width; j++) {
				switch(map->data[i_rev*width + j]) {
					default:
						case -1:
							image.data[i*width + j] = 150;
							break;
						case 0:
							image.data[i*width + j] = 255;
							break;
						case 100:
							image.data[i*width + j] = 0;
							break;
				}
			}
		}
		
		cv::cvtColor(image, image, CV_GRAY2BGR);
		ROS_INFO("Image extracted from map.");
		cv::imshow(winname, image);
		cv::setMouseCallback(winname, mouseCallback);
		cv::waitKey(0);
	}
}

#endif
