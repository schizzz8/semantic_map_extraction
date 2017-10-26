#include "ground_truth_creator.hpp"

using namespace ground_truth_creator;

int main(int argc, char** argv) {
	ros::init(argc, argv, "ground_truth_creator");
	ros::NodeHandle n("~");
	ros::Subscriber sub_map;
	
	std::ostringstream error_msg;

	error_msg << "Usage: ground_truth_creator [GROUND_TRUTH_FILE] [MODALITY] [FILE]" << std::endl
						<< "GROUND_TRUTH_FILE: full/path/to/the/filename (without extension) on which to write/append ground truth information" << std::endl
						<< "MODALITY: 0 if you want to use the map_server, 1 otherwhise" << std::endl
		 				<< "FILE: used ony in MODALITY=1 and it represents the image to work on" << std::endl;

	if(argc < 3) {
		std::cerr << error_msg.str();
		exit(1);
	}

	int modality = 0;

	gt_filename = argv[1];
	gt_writer.open((gt_filename + ".txt").c_str(), std::ofstream::out); // | std::ofstream::app);
	std::istringstream(argv[2]) >> modality;

	if((argc < 4 && modality == 1) || (modality != 0 && modality != 1)) {
		std::cerr << error_msg.str();
		exit(1);
	}

	if(modality == 0) {
		start_gtc();
		sub_map = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &mapSubscriber);
	}
	else {
		std::string file = argv[3];
		std::ifstream f(file.c_str());

		if(!f.good()) {
			f.close();
			std::cerr << "File " << file << " does not exist!" << std::endl;
			exit(1);
		}

		f.close();

		cv::Mat src = cv::imread(file, 0);
		ROS_INFO("File uploaded.");
		image = src.clone();
		cv::cvtColor(image, image, CV_GRAY2BGR);
		start_gtc();
		cv::imshow(winname, image);
		cv::setMouseCallback(winname, mouseCallback);
		cv::waitKey(0);
	}
	
	ros::spin();
	gt_writer.close();
	
	return 0;
}
