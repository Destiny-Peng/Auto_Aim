#pragma once
#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Armor.hpp"
#include "time_stamp.hpp"
using namespace std;
using namespace cv;
class PreProcessing
{
private:
	Mat imageRedChannel, binaryImageG, binaryImageR, binaryImage, grayImage, ProcessedImage, ImageIn, imgBlur, imgdilate, contourimg, newcontours;
	vector<Mat> channels;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat kernal = getStructuringElement(MORPH_RECT, Size(7, 7));
	Mat kernal2 = getStructuringElement(MORPH_RECT, Size(5, 5));
	int thereshold = 1;

	double calculateCircularity(const std::vector<Point> &contour)
	{
		double area = contourArea(contour);
		double perimeter = arcLength(contour, true);
		return (4 * CV_PI * area) / (perimeter * perimeter);
	}

public:
	vector<LightBar> Contours2LightBar(vector<vector<Point>> contours)
	{
		vector<RotatedRect> rects;
		for (vector<Point> contour : contours)
			rects.push_back(minAreaRect(contour));
		vector<LightBar> lightbars;
		for (int i = 0; i < rects.size(); i++)
			lightbars.push_back(LightBar(rects[i]));
		return lightbars;
	}

	vector<Vec3f> Contours2Ellipse(vector<vector<Point>> contours, double dp = 1, double minDist = 50, double param1 = 100, double param2 = 47, int minRadius = 0, int maxRadius = 100)
	{
		vector<Vec3f> ellipses;
		Mat img = Mat::zeros(480, 640, CV_8U);
		for (int i = 0; i < contours.size(); i++)
			drawContours(img, contours, i, Scalar(0, 255, 0), 2);
		HoughCircles(img, ellipses, HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
		return ellipses;
	}

	vector<vector<Point>> PreProcess(Mat input, int red = 198, int grey = 98, int redmax = 243, int greymax = 157)
	{
		split(input, channels);
		imageRedChannel = channels[2];
		cvtColor(input, grayImage, COLOR_BGR2GRAY);
		inRange(imageRedChannel, red, redmax, binaryImageR);
		inRange(grayImage, grey, greymax, binaryImageG);
		threshold(binaryImageG & binaryImageR, binaryImage, thereshold, 255, THRESH_BINARY);
		dilate(binaryImage, imgdilate, kernal);
		// imshow("imgdilate", binaryImage);
		vector<vector<Point>> cs = getContours(imgdilate);
		contourimg = DrawContours(input, cs);
		// imshow("contourimg", contourimg);
		dilate(contourimg, newcontours, kernal2);
		vector<vector<Point>> cs2 = getContours(newcontours);
		ProcessedImage = DrawContours(input, cs2);
		// imshow("ProcessedImage",ProcessedImage);
		return cs2;
	}

	vector<vector<Point>> getContours(Mat InputImg)
	{
		vector<vector<Point>> FilteredContours;
		findContours(InputImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++)
		{
			double circularity = calculateCircularity(contours[i]);
			int area = contourArea(contours[i]);
			if ((area > 50) && (1))
			{
				FilteredContours.push_back(contours[i]);
			}
		}
		return FilteredContours;
	}

	Mat DrawContours(Mat frame, vector<vector<Point>> cs)
	{
		Mat img = Mat::zeros(frame.rows, frame.cols, CV_8U);
		for (int i = 0; i < cs.size(); i++)
		{
			int area = contourArea(cs[i]);
			if (1)
			{
				drawContours(img, cs, -1, Scalar(255, 0, 255), 2);
			}
		}
		return img;
	}

	vector<Armor> ArmorFliter(vector<Armor> putin)
	{
		vector<Armor> returnarmors;
		for (int i = 0; i < putin.size(); i++)
		{
			if (putin[i].get_number())
				returnarmors.push_back(putin[i]);
		}
		return returnarmors;
	}

	vector<Armor> getArmor(vector<LightBar> lightBars)
	{
		vector<Armor> armors;
		vector<Armor> return_armors;
		if (lightBars.size() >= 2)
		{
			for (size_t i = 0; i < lightBars.size() - 1; i++)
			{
				for (size_t j = i + 1; j < lightBars.size(); j++)
				{
					if (lightBars[0].is_an_armor(lightBars[i], lightBars[j]))
					{
						Armor armor(lightBars[i], lightBars[j]);
						if (lightBars[i].Armor_index && (armor.Score() >= armors[lightBars[i].Armor_index - 1].Score()))
							armors[lightBars[i].Armor_index - 1] = armor;
						else if (lightBars[j].Armor_index && (armor.Score() >= armors[lightBars[j].Armor_index - 1].Score()))
							armors[lightBars[j].Armor_index - 1] = armor;
						else if ((!(lightBars[i].Armor_index || lightBars[j].Armor_index)) && (armor.Score() > 80))
						{
							lightBars[i].Armor_index = armors.size() + 1;
							lightBars[j].Armor_index = armors.size() + 1;

							// cout << armor.Score()<<endl;

							armors.push_back(armor);
						}
					}
				}
			}
			return armors;
		}
		else
		{
			return armors;
		}
	}

	void drawLightBar(Mat &imgin, vector<LightBar> lightbars)
	{
		RotatedRect rect;
		for (int i = 0; i < lightbars.size(); i++)
		{
			rect = lightbars[i].get_lightBar();
			Point2f vertices[4];
			rect.points(vertices);
			for (int j = 0; j < 4; j++)
			{
				line(imgin, vertices[j], vertices[(j + 1) % 4], Scalar(255, 0, 0), 2);
			}
			// putText(imgin,"1",lightbars[i].get_middle_point(lightbars[i].get_lightBar())[0],FONT_HERSHEY_DUPLEX,2,Scalar(0,69,255),2);
			// putText(imgin,"2",lightbars[i].get_middle_point(lightbars[i].get_lightBar())[1],FONT_HERSHEY_DUPLEX,2,Scalar(0,69,255),2);
			// line(imgin,lightbars[i].get_middle_point(lightbars[i].lightbar)[0],lightbars[i].get_middle_point(lightbars[i].lightbar)[1],Scalar(0,255,0),2);
		}
	}

	void drawArmor(Mat &imgin, vector<Armor> armors)
	{
		LightBar lightbarin1, lightbarin2;
		for (int i = 0; i < armors.size(); i++)
		{
			lightbarin1 = armors[i].get_lightbar1();
			lightbarin2 = armors[i].get_lightbar2();
			Point2f points[4] = {lightbarin1.get_middle_point(lightbarin1.get_lightBar())[0], lightbarin1.get_middle_point(lightbarin1.get_lightBar())[1], lightbarin2.get_middle_point(lightbarin2.get_lightBar())[0], lightbarin2.get_middle_point(lightbarin2.get_lightBar())[1]};
			if (!armors[i].tracktarget)
			{
				for (int j = 0; j < 4; j++)
				{
					circle(imgin, armors[i].getPoints()[j], 15, Scalar(0, 69, 255), FILLED);
					line(imgin, armors[i].getPoints()[j], armors[i].getPoints()[(j + 1) % 4], Scalar(255, 0, 255), 2);
				}
				putText(imgin, to_string(armors[i].result), points[1], FONT_HERSHEY_DUPLEX, 3, Scalar(125, 125, 125), 2, FILLED);
			}
			else
			{
				for (int j = 0; j < 4; j++)
				{
					circle(imgin, armors[i].getPoints()[j], 15, Scalar(0, 255, 69), FILLED);
					line(imgin, armors[i].getPoints()[j], armors[i].getPoints()[(j + 1) % 4], Scalar(255, 255, 0), 2);
				}
				putText(imgin, to_string(armors[i].result), points[1], FONT_HERSHEY_DUPLEX, 3, Scalar(125, 125, 125), 2, FILLED);
			}
			if (armors[i].targetnumber)
			{
				putText(imgin, to_string(armors[i].targetnumber), points[2], FONT_HERSHEY_DUPLEX, 3, Scalar(125, 185, 78), 2, FILLED);
			}
		}
	}
};
#endif