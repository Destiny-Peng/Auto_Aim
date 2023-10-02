#pragma once
#ifndef LIGHTBAR_H
#define LIGHTBAR_H

#include <iostream>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv; 

class LightBar {
private:

	RotatedRect lightbar;

	double Distance(const Point2f& point1, const Point2f& point2) {
    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;
    double distance = sqrt(dx * dx + dy * dy);
    return distance;
}

public:
	int Armor_index = 0;

	RotatedRect get_lightBar(){
		return lightbar;
	}

	LightBar(RotatedRect LightBarIn = RotatedRect()) {
		lightbar = LightBarIn;
	}
	
	double get_area(RotatedRect rect) {
		float width = rect.size.width;
		float height = rect.size.height;
		return width * height;
	}

	double get_slope(RotatedRect rect) {
		vector<Point2f> shortSideMidpoints = get_middle_point(rect);
		if ((shortSideMidpoints[1].x - shortSideMidpoints[0].x)){
			return atan((shortSideMidpoints[1].y - shortSideMidpoints[0].y) / (shortSideMidpoints[1].x - shortSideMidpoints[0].x));
		}
		else {
			return CV_PI/2;
		}
	}

	bool is_an_armor(LightBar LightBar1, LightBar LightBar2) {
		double delta_slope = 0.085;
		double delta_area = 0.4;
		if ((-delta_slope < get_slope(LightBar1.lightbar) - get_slope(LightBar2.lightbar)) &&(get_slope(LightBar1.lightbar) - get_slope(LightBar2.lightbar) < delta_slope)
			&&(delta_area < get_area(LightBar1.lightbar) / get_area(LightBar2.lightbar) < 1 / delta_area)&&(get_area(LightBar1.lightbar) / get_area(LightBar2.lightbar) < 1 / delta_area)){
			return true;
			}
		else return false;
	}

	vector<Point2f> get_middle_point(RotatedRect rect){
		Point2f vertices[4];
		rect.points(vertices);
		double distances[4];
		for (int i = 0; i < 4; i++) {
			distances[i] = Distance(vertices[i] ,vertices[(i + 1) % 4]);
		}
		vector<Point2f> shortSideMidpoints;
		if (distances[0]<=distances[1]){
			shortSideMidpoints.push_back((vertices[0] + vertices[1]) * 0.5);
			shortSideMidpoints.push_back((vertices[2] + vertices[3]) * 0.5);
		}
		else {
			shortSideMidpoints.push_back((vertices[1] + vertices[2]) * 0.5);
			shortSideMidpoints.push_back((vertices[0] + vertices[3]) * 0.5);	
		}
		return shortSideMidpoints;
	}

};


#endif
