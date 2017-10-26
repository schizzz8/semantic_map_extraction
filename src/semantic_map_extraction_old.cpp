#include "ros/ros.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

#include "semantic_matrix.hpp"
#include "color_extractor.hpp"

#define MIN_DETECTABLE_ROOM_SIDE 5
#define MAX_ROOM_WALL_OFFSET 25
#define SEGMENT_NEIGHBORHOOD_TO_BE_CHECKED 2
#define MIN_ROOM_PIXEL_INTENSITY 150
#define DOOR_TAG_MAX_DISTANCE 1

#define MAX_THRESHOLD 100.0
#define ORIGINAL_MAP_WINDOW "Original map"
#define MODIFIED_MAP_WINDOW "Detected lines"
#define MASKED_MAP_WINDOW "Grilled map"
#define MASKED_MAP_OBJECTS_WINDOW "Grilled map with objects"
#define SEGMENTED_MAP_WINDOW "Detected map rooms"
	
using namespace cv;
using namespace std;

void drawObjects(const multimap<string,vector<float> > objects, const float resolution, Mat &image) {
	ifstream objects_features("kb_tags.txt");
	string line;

	while (getline(objects_features, line)) {
		istringstream ss(line);
		string object_type, object_form;
		float dim1, dim2;

		ss >> object_type;
		ss >> dim1 >> dim2;
		ss >> object_form;

		dim1 = dim1/(100*resolution);
		dim2 = dim2/(100*resolution);

		for(multimap<string, vector<float> >::const_iterator i = objects.begin(); i != objects.end(); i++) {
			if((*i).first.find(object_type) != std::string::npos) {
				if(object_form == "RECTANGLE") {
					float bigger = (dim1 > dim2) ? dim1 : dim2;
					float smaller = (dim1 > dim2) ? dim2 : dim1;
					
					float x, y, angle, err;
					vector<Point> vertices;

					if(abs(fmod((*i).second[2], CV_PI/2)) < CV_PI/4)
						err = fmod((*i).second[2], CV_PI/2);
					else
						err = fmod((*i).second[2], CV_PI/2) - CV_PI/2;

					angle = (*i).second[2] - err;
					
					x = (*i).second[0] - cvRound(bigger*cos(angle)/2);
					y = (*i).second[1] + cvRound(bigger*sin(angle)/2);	
					vertices.push_back(Point(x, y));

					x = (*i).second[0] + cvRound(bigger*cos(angle)/2);
					y = (*i).second[1] - cvRound(bigger*sin(angle)/2);
					vertices.push_back(Point(x, y));

					x = (*i).second[0] + cvRound(bigger*cos(angle)/2) + cvRound(smaller*sin(angle));
					y = (*i).second[1] - cvRound(bigger*sin(angle)/2) + cvRound(smaller*cos(angle));
					vertices.push_back(Point(x, y));

					x = (*i).second[0] - cvRound(bigger*cos(angle)/2) + cvRound(smaller*sin(angle));
					y = (*i).second[1] + cvRound(bigger*sin(angle)/2) + cvRound(smaller*cos(angle));
					vertices.push_back(Point(x, y));

					fillConvexPoly(image, &vertices[0], 4, Scalar(255));
				}
				else if (object_form == "CIRCLE")
					circle(image, Point((*i).second[0], (*i).second[1]), dim1, Scalar(255), -1);

				putText(image, (*i).first, Point((*i).second[0], (*i).second[1]), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
			}
		}
	}
}

void getObjectPositions(multimap<string,vector<float> > &objects, multimap<string,vector<float> > &rooms, const float resolution, const float origin_x, const float origin_y, const int rows, const int cols) {
	ifstream places("FinalRun.places");
	//ifstream places_obj("MapTags.txt");
	string line;
	string line_obj;
	
	while (getline(places, line)) { // && getline(places_obj, line_obj)) {
		istringstream ss(line);
		//istringstream ss1(line_obj);
		string object_name;
		float x_rob, y_rob, theta_rob, x_obj, y_obj, theta_obj, position_x, position_y;
		vector<float> coords;

		ss >> object_name;
		ss >> x_obj >> y_obj >> theta_obj >> x_rob >> y_rob >> theta_rob;
		//ss1 >> object_name >> x_obj >> y_obj >> theta_obj;

		position_x = cvRound((-origin_x + 1 + x_rob + cos(theta_rob)*x_obj - sin(theta_rob)*y_obj)/resolution);
		position_y = rows + cvRound((origin_y - 1 - y_rob - sin(theta_rob)*x_obj - cos(theta_rob)*y_obj)/resolution);

		coords.push_back(position_x);
		coords.push_back(position_y);
		coords.push_back(theta_obj + theta_rob + CV_PI/2);

		if(object_name.find("ROOM") != std::string::npos)
			rooms.insert(pair<string,vector<float> >(object_name, coords));
		else
			objects.insert(pair<string,vector<float> >(object_name, coords));
	}
}

bool checkActivePixel(const int row, const int col, const int neighborhoodToCheck, const bool moveOnX, const bool moveOnY, const Mat gray_img) {
	for(int n = 0; n < neighborhoodToCheck; n++) {
		const uchar *tmp_row_add = 0, *tmp_row_sub = 0;
		bool check_add = false, check_sub = false;

		int dx = n*moveOnX, dy = n*moveOnY;

		if(row + dy < gray_img.rows) {
			tmp_row_add = gray_img.ptr<uchar>(row + dy);
			check_add = true;
		}
		else
			check_add = false;

		if(row >= dy) {
			tmp_row_sub = gray_img.ptr<uchar>(row - dy);
			check_sub = true;
		}
		else
			check_sub = false;

		if((check_add && ((col + dx < gray_img.cols && tmp_row_add[col + dx] > MIN_ROOM_PIXEL_INTENSITY) || (col >= dx &&
		tmp_row_add[col - dx] > MIN_ROOM_PIXEL_INTENSITY))) || (check_sub && ((col + dx < gray_img.cols &&
		tmp_row_sub[col + dx] > MIN_ROOM_PIXEL_INTENSITY) || (col >= dx && tmp_row_sub[col - dx] > MIN_ROOM_PIXEL_INTENSITY))))
			return true;
	}

	return false;
}

void detectLineSegments(const multimap<string,vector<float> > room_map, const Vec2f line, const Mat gray_img, vector<vector<Point> > &segments) {
	int count = 0;
	int num_decreases = 0;
	bool connected = false;
	bool old_toPrint = false;
	bool canBeDrawn = false;

	vector<Point> vertices;
	Mat tmp = Mat::zeros(gray_img.rows, gray_img.cols, CV_8U);

	cvtColor(tmp, tmp, CV_GRAY2BGR);

	for(multimap<string, vector<float> >::const_iterator i = room_map.begin(); i != room_map.end(); i++)
		circle(tmp, Point((*i).second[0], (*i).second[1]), 0.02*gray_img.rows, Scalar(255,255,255), -1);

	cvtColor(tmp, tmp, CV_BGR2GRAY);
	/*namedWindow("b", CV_WINDOW_NORMAL);
	imshow("b", gray_img+tmp);*/

	float rho = line[0], theta = line[1];
	double x0 = cvRound(rho*cos(theta)), y0 = cvRound(rho*sin(theta));

	for(int i = y0; i <= y0 + cvRound(gray_img.rows*cos(theta)); i++) {
		for(int j = x0; j <= x0 + cvRound(gray_img.cols*sin(theta)); j++) {
			//check the consecutive pixels next to the line pixels
			bool active_pixel = false;
			bool toPrint = false;

			for(int n = 0; n < MAX_ROOM_WALL_OFFSET; n++)
				if(checkActivePixel(i + cvRound(n*cos(theta)*connected), j + cvRound(n*sin(theta)*connected), SEGMENT_NEIGHBORHOOD_TO_BE_CHECKED, cvRound(cos(theta)), cvRound(sin(theta)), gray_img)) {
					active_pixel = true;

					if(n > 0)
						toPrint = true;
					break;
				}

			if(checkActivePixel(i, j, DOOR_TAG_MAX_DISTANCE, cvRound(cos(theta)), cvRound(sin(theta)), tmp))
				canBeDrawn = true;

			if (active_pixel) {
				if(connected || (!connected && (++count + num_decreases) >= MIN_DETECTABLE_ROOM_SIDE)) {
					connected = true;		
					num_decreases = 0;
					count = MIN_DETECTABLE_ROOM_SIDE;

					//Save the first connected Point
					if((toPrint && !old_toPrint) || (!toPrint && old_toPrint)) {
						vertices.push_back(Point(j, i));

						if(!toPrint && old_toPrint) {
							if(canBeDrawn)
								segments.push_back(vertices);

							vertices.clear();
							canBeDrawn = false;
						}
					}

					old_toPrint = toPrint;
				}
			}
			else if(connected && (num_decreases -= 2) && --(--count) <= 0) {
				connected = false;
				count = 0;
				num_decreases = 0;
				vertices.clear();
				canBeDrawn = false;
				old_toPrint = toPrint;
			}
			else if(!connected) {
				count = 0;
				vertices.clear();
				canBeDrawn = false;
				old_toPrint = toPrint;
			}
		}
	}
}

/*double angle( Point pt1, Point pt2, Point pt0 ) {
	double dx1 = pt1.x - pt0.x; 	
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}*/

void detectRooms(const multimap<string,vector<float> > room_map, const vector<Vec2f> lines, const Mat gray_img, Mat &segmented_image) {
	vector<vector<Point> > rooms;
	vector<vector<Point> > segments;
	
	for(size_t i = 0; i < lines.size(); i++)
		detectLineSegments(room_map, lines[i], gray_img, segments);

	Mat tmp = Mat::zeros(gray_img.rows, gray_img.cols, CV_8U);
	cvtColor(gray_img, segmented_image, CV_GRAY2BGR);
	
	RNG rng(12345);
	
	for(size_t i = 0; i < segments.size(); i++)
		line(segmented_image, segments[i][0], segments[i][1], Scalar(255,0,0), 0.5, CV_AA);
	//blur(segmented_image, segmented_image, Size(6,6));
	dilate(segmented_image, segmented_image, cv::Mat(), Point(-1,-1), 2);

	namedWindow("a", CV_WINDOW_NORMAL);
	imshow("a", segmented_image);
	cvtColor(segmented_image, tmp, CV_BGR2GRAY);
	vector<vector<Point> > contours;
	findContours(tmp, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	for(size_t i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(segmented_image, contours, i, color, -1);
	}

	erode(segmented_image, segmented_image, cv::Mat(), Point(1,-1), 1);
}

void printLines(vector<Vec2f> lines, Mat &img_dst) {
	float rho, theta;

	for(size_t i = 0; i < lines.size(); i++) {
		rho = lines[i][0];
		theta = lines[i][1];
		double x0 = rho*cos(theta), y0 = rho*sin(theta);
		Point pt1(cvRound(x0), cvRound(y0 + img_dst.rows*cos(theta)));
		Point pt2(cvRound(x0 + img_dst.cols*sin(theta)), cvRound(y0));
		line(img_dst, pt1, pt2, Scalar(0,0,255), 0.5, CV_AA);
	}
}

void divideSegment(vector<Vec2f> &lines, const int first_line, const int second_line, const float max_dist) {
	float first_rho = lines[first_line][0];
	float second_rho = lines[second_line][0];
	float insertion_angle = lines[first_line][1]; //They are parallel lines

	if(floor(fabs(first_rho - second_rho)/max_dist) < 2.0)
		return;

	Vec2f new_line((first_rho + second_rho)/2, insertion_angle);
	lines.push_back(new_line);
	int inserted_line = lines.size() - 1;

	divideSegment(lines, first_line, inserted_line, max_dist);
	divideSegment(lines, inserted_line, second_line, max_dist);
}

void realizeGrill(vector<Vec2f> &lines, const float distX, const float distY) {
	float first_line_rho, first_line_theta, second_line_rho, second_line_theta;
	
	size_t size = lines.size();

	for(size_t i = 0; i < size; i++) {
		first_line_rho = lines[i][0];
		first_line_theta = lines[i][1];
		
		bool first_segment = true;
		float segment_dist = 0.0;
		size_t nearest_second_line_index = 0;

		//Detect the smallest segment for each line
		for(size_t j = 0; j < size; j++) {
			second_line_rho = lines[j][0];
			second_line_theta = lines[j][1];

			if(fabs(fmod(first_line_theta, (float) CV_PI) - fmod(second_line_theta, (float) CV_PI)) < 0.01) {
				//The second condition guarantees that already checked lines are no more considered for the distances
				if(first_segment && first_line_rho - second_line_rho > 0.0) { 
					segment_dist = fabs(first_line_rho - second_line_rho);
					first_segment = false;
					nearest_second_line_index = j;
				}
				else if(fabs(first_line_rho - second_line_rho) < segment_dist && first_line_rho - second_line_rho > 0.0) {
					segment_dist = fabs(first_line_rho - second_line_rho);
					nearest_second_line_index = j;
				}
			}
		}

		//For each detected segment, divide it (if needed)
		if(segment_dist != 0)
			divideSegment(lines, i, nearest_second_line_index, (fabs(fmod(first_line_theta, (float) CV_PI)) < CV_PI/4) ? distX : distY);
	}
}

void getMinDistances(const vector<Vec2f> lines, float &distX, float &distY) {
		float first_line_rho, first_line_theta, second_line_rho, second_line_theta;

		bool firstX = true, firstY = true;

		for(size_t i = 0; lines.size() != 0 && i < lines.size() - 1; i++) {
			first_line_rho = lines[i][0];
			first_line_theta = lines[i][1];

			for(size_t j = i + 1; j < lines.size(); j++) {
				second_line_rho = lines[j][0];
				second_line_theta = lines[j][1];

				if(fabs(fmod(first_line_theta, (float) CV_PI) - fmod(second_line_theta, (float) CV_PI)) < 0.01) {
					bool* first_ptr = NULL;
					float* dist_ptr = NULL;

					first_ptr = (fabs(fmod(first_line_theta, (float) CV_PI)) < CV_PI/4) ? &firstX : &firstY;
					dist_ptr = (fabs(fmod(first_line_theta, (float) CV_PI)) < CV_PI/4) ? &distX : &distY;
						
					if(*first_ptr) {
						*dist_ptr = fabs(first_line_rho - second_line_rho);
						*first_ptr = false;
					}
					else if(fabs(first_line_rho - second_line_rho) < *dist_ptr)
						*dist_ptr = fabs(first_line_rho - second_line_rho);
				}
			}
		}
}

void linesExtractor(int threshold, void* image, const float resolution, const float origin_x, const float origin_y) {
	Mat img = *(static_cast<Mat*>(image)); //Mat::zeros(IMG_ROWS, IMG_COLS, CV_8U);
	//resize(*(static_cast<Mat*>(image)), img, img.size());
	Mat dst, cdst;

	Canny(img, dst, 1, 300, 3, true);
	//cvtColor(dst, cdst, CV_GRAY2BGR);
	cdst = img.clone();
	cvtColor(cdst, cdst, CV_GRAY2BGR);
	ROS_INFO("Image lines extraction.");

	float rho, theta;
	vector<Vec2f> lines;
	vector<Vec2f> good_lines;
	vector<float> rhos;
	vector<float> thetas;
	bool toPrint = true;

	for(int j = MAX_THRESHOLD; j > threshold; j--) {
		HoughLines(dst, lines, 2, CV_PI/2, (j > 0) ? j : 1, 0, 0);

		for(size_t i = 0; i < lines.size(); i++) {
			rho = lines[i][0];
			theta = lines[i][1];
			toPrint = true;

			int pos = 0;
			for(vector<float>::iterator it = rhos.begin(); it != rhos.end(); it++, pos++)
				if(fabs(*it - rho) < max(MAX_THRESHOLD/(float)j, 5.0) && fabs(fmod(thetas.at(pos), (float) CV_PI) - fmod(theta, (float) CV_PI)) < 0.01) {
					toPrint = false;
					break;
				}

			if(toPrint) {
				rhos.push_back(rho);
				thetas.push_back(theta);
				good_lines.push_back(lines[i]);
			}
		}
	}

	float distX = 0.0, distY = 0.0;
	getMinDistances(good_lines, distX, distY);
	ROS_INFO("Smallest sizes detected - x: %f, y: %f.", distX, distY);

	vector<Vec2f> wall_lines(good_lines);

	if(distX != 0.0 && distY != 0.0) {
		ROS_INFO("Realizing grill.");
		realizeGrill(good_lines, distX, distY);
		ROS_INFO("Grill realized.");
	}

	printLines(good_lines, cdst); //printLines(wall_lines, cdst);
	/*Mat cdst2 = cdst.clone();
	printLines(good_lines, cdst2);
	for(int i = 0; i < cdst.rows; i++)
		for(int j = 0; j < cdst.cols; j++)
			if((cdst.at<cv::Vec3b>(i,j)[0] && cdst2.at<cv::Vec3b>(i,j)[0]) || (cdst.at<cv::Vec3b>(i,j)[1] && cdst2.at<cv::Vec3b>(i,j)[1]) || (cdst.at<cv::Vec3b>(i,j)[2] && cdst2.at<cv::Vec3b>(i,j)[2])) {
			if(cdst2.at<cv::Vec3b>(i,j)[0] != 255 && cdst2.at<cv::Vec3b>(i,j)[1] != 255 && cdst2.at<cv::Vec3b>(i,j)[2] != 255) {
				cdst2.at<cv::Vec3b>(i,j)[0] = 0;
				cdst2.at<cv::Vec3b>(i,j)[1] = 150;
				cdst2.at<cv::Vec3b>(i,j)[2] = 0;
			}}
	cdst = cdst2;*/
	ROS_INFO("Lines printed in the image.");

	multimap<string,vector<float> > objects;
	multimap<string,vector<float> > rooms;
	ROS_INFO("Object positions computed.");

	/*if(resolution != -1)
		getObjectPositions(objects, rooms, resolution, origin_x, origin_y, cdst.rows, cdst.cols);
*/
	imshow(ORIGINAL_MAP_WINDOW, img);
	imwrite("original_map.png", img);
	imshow(MODIFIED_MAP_WINDOW, cdst);
	imwrite("grill.png", cdst);

	//Room detection
	Mat segmented_image;
	detectRooms(rooms, good_lines, dst, segmented_image);

	//Masking
	vector<vector<Point> > contours;
	Mat background_modified_img = img.clone();

	for(int i = 0; i < background_modified_img.rows; i++) {
		uchar* col = background_modified_img.ptr<uchar>(i);
		for(int j = 0; j < background_modified_img.cols; j++){
			if(col[j] < 225)
				col[j] = 0;}
	}

	dilate(background_modified_img, background_modified_img, cv::Mat(), Point(-1,-1), 2);
	findContours(background_modified_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	Mat mask = Mat::zeros(background_modified_img.rows, background_modified_img.cols, CV_8U), tmp, masked_grill, masked_rooms;
    	drawContours(mask, contours, -1, Scalar(255), CV_FILLED);
	//Grill mask
    	cdst.copyTo(masked_grill, mask);
	//Room colors mask
	segmented_image.copyTo(masked_rooms, mask);
	Mat cimg;

	cvtColor(img, cimg, CV_GRAY2BGR);
	cvtColor(mask, mask, CV_GRAY2BGR);
	masked_grill += cimg - mask;
	masked_rooms += cimg - mask;

	dilate(masked_rooms, masked_rooms, cv::Mat(), Point(-1,-1), 2);
	cvtColor(dst, tmp, CV_GRAY2BGR);

	/*for(int i = 0; i < masked_rooms.rows; i++)
		for(int j = 0; j < masked_rooms.cols; j++)
			if((masked_rooms.at<cv::Vec3b>(i,j)[0] && masked_grill.at<cv::Vec3b>(i,j)[0]) || (masked_rooms.at<cv::Vec3b>(i,j)[1] && masked_grill.at<cv::Vec3b>(i,j)[1]) || (masked_rooms.at<cv::Vec3b>(i,j)[2] && masked_grill.at<cv::Vec3b>(i,j)[2])) {
				masked_rooms.at<cv::Vec3b>(i,j)[0] = 0;
				masked_rooms.at<cv::Vec3b>(i,j)[1] = 0;
				masked_rooms.at<cv::Vec3b>(i,j)[2] = 0;
			}
	*/

	erode(masked_rooms, masked_rooms, cv::Mat(), Point(-1,-1), 4);
	masked_grill += tmp;
	masked_rooms += tmp;

   	namedWindow(MASKED_MAP_WINDOW, CV_WINDOW_NORMAL);
	imshow(MASKED_MAP_WINDOW, masked_grill);
	imwrite("masked_grill.png", masked_grill);

	Mat objects_map = masked_grill.clone();
	drawObjects(objects, resolution, objects_map);

	namedWindow(MASKED_MAP_OBJECTS_WINDOW, CV_WINDOW_NORMAL);
	imshow(MASKED_MAP_OBJECTS_WINDOW, objects_map);
	imwrite("masked_grill_objects.png", objects_map);

	drawObjects(objects, resolution, masked_rooms);
   	namedWindow(SEGMENTED_MAP_WINDOW, CV_WINDOW_NORMAL);
	imshow(SEGMENTED_MAP_WINDOW, masked_rooms);
	imwrite("room_colors.png", masked_rooms);

	/*vector<Point> approx;
	vector<vector<Point> > squares;

	for(size_t i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

		if(approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1 && isContourConvex(Mat(approx))) {
			double maxCosine = 0;

			for(int j = 2; j < 5; j++) {
				double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
				maxCosine = MAX(maxCosine, cosine);
			}
			
			if( maxCosine < 0.3 )
				squares.push_back(approx);
		}
	}
Mat sa = Mat::zeros( dst.size(), CV_8UC3 );
for( size_t i = 0; i < squares.size(); i++ )
	    {cout << i << endl;
	        const Point* p = &squares[i][0];
	        int n = (int)squares[i].size();
	        polylines(sa, &p, &n, 1, true, Scalar(0,255,0), 0.5, CV_AA);

	    }RNG rng(12345);
	for( size_t i = 0; i < contours.size(); i++ ){Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );drawContours(sa, contours, i, color, 0.5, CV_AA, NULL, 0, Point() );}
	    imshow("wndname", sa);*/
}

void linesExtraction(Mat image, const float resolution = -1, const float origin_x = 0, const float origin_y = 0) {
	//int threshold = 50;
	
	namedWindow(ORIGINAL_MAP_WINDOW, CV_WINDOW_NORMAL);
	namedWindow(MODIFIED_MAP_WINDOW, CV_WINDOW_NORMAL);

	//createTrackbar("Lines detection threshold", MODIFIED_MAP_WINDOW, &threshold, MAX_THRESHOLD, linesExtractor, &image);
	linesExtractor(0, &image, resolution, origin_x, origin_y);
	//labeling(&image);
	
	ROS_INFO("Lines extracted.");
	waitKey();
}

void mapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	int width = map->info.width;
	int height = map->info.height;
	
	Mat image(height, width, CV_8U);
	ROS_INFO("Map received.");

	for(int i = 0, i_rev = height - 1; i < height; i++, i_rev--)
		for(int j = 0; j < width; j++)
			switch(map->data[i_rev*width + j]) {
				case -1:
					image.data[i*width + j] = 127;
					break;
				case 0:
					image.data[i*width + j] = 255;
					break;
				case 100:
					image.data[i*width + j] = 0;
					break;
			}

	ROS_INFO("Image extracted from map.");

	linesExtraction(image, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "semantic_extraction");
	ros::NodeHandle n;

	ostringstream error_msg;

	error_msg << "Usage: semantic_map_extraction [MODALITY] [FILE]" << endl << "MODALITY: 0 if you want to use the map_server, 1 otherwhise" << endl
		 << "FILE: used ony in MODALITY=1 and it represents the image to work on" << endl;

	if(argc < 2) {
		cerr << error_msg.str();
		exit(1);
	}

	int modality = 0;
	string file;
	istringstream(argv[1]) >> modality;

	if((argc < 3 && modality == 1) || (modality != 0 && modality != 1)) {
		cerr << error_msg.str();
		exit(1);
	}

	if(modality == 0) {
		ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapSubscriber);
		ros::spin();
	}
	else {
		file = argv[2];

		ifstream f(file.c_str());

		if(!f.good()) {
			f.close();
			cerr << "File " << file << " does not exist!" << endl;
			exit(1);
		}

		f.close();

		Mat src = imread(file, 0); // imread("mappe/prova.ppm", 0);  imread("dis-B1-2011-09-27.png", 0);
		linesExtraction(src);
	}

	return 0;
}
