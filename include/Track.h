#pragma once
#ifndef TRACK_H
#define TRACK_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "LightBar.h"
#include "Armor.h"
#include "PreProcess.h"

using namespace std;
using namespace cv; 

class Track{
private:
    vector<vector<Armor>> TrackingArmors;
    LightBar blanklightbar;
    Armor blankarmor = Armor(blanklightbar,blanklightbar);
    PreProcessing preprocess;
    vector<int> erasednumbers;
    bool found = false;

    int get_last_index(vector<Armor> target){
        int lastIndex = -1;
        for (int i = target.size()-1;i>-1;i--) {
            if (!target[i].isEmpty) {
                lastIndex = i;
                break;
            }
        }
        return lastIndex;
    }

    int get_last_time(vector<Armor> target){
        int lastIndex = -1;
        for (int i = target.size()-1;i>-1;i--) {
            if (target[i].tracktarget) {
                lastIndex = i;
                break;
            }
        }
        return lastIndex;
    }

    int get_first_index(vector<Armor> target){
        int firstIndex = -1;
        for (int i = target.size()-1;i>-1;i--) {
            if (!target[i].isEmpty) {
                firstIndex = i;
            }
        }
        return firstIndex;
    }

    Armor predict(vector<Armor> armorsin){
        vector<Point2f> points= get_centers(armorsin);

        double vx = (points[points.size()-1].x-points[0].x)/points.size();
        double vy = (points[points.size()-1].y-points[0].y)/points.size();
        cout << "speed =" << "vx:" << vx <<" "<< "vy:" << vy<<endl;
        Point2f predictcenter = Point2f(points[points.size()-1].x+vx,points[points.size()-1].y+vy);
        Point2f vertices1[4],vertices2[4];
		armorsin[get_last_index(armorsin)].get_lightbar1().get_lightBar().points(vertices1);
        armorsin[get_last_index(armorsin)].get_lightbar2().get_lightBar().points(vertices2);
        for (int i = 0;i<4;i++){
            vertices1[i] += predictcenter - armorsin[get_last_index(armorsin)].center();
            vertices2[i] += predictcenter - armorsin[get_last_index(armorsin)].center();
        }
        vector<Point2f> pointVec1(vertices1, vertices1 + 4);
        vector<Point2f> pointVec2(vertices2, vertices2 + 4);
        LightBar lightbar1 = (minAreaRect(pointVec1));
        LightBar lightbar2 = (minAreaRect(pointVec2));
        Armor returnarmor = Armor(lightbar1,lightbar2);
        returnarmor.tracktarget = false;
        returnarmor.result = armorsin[get_last_index(armorsin)].result;
        return returnarmor;
    }

    vector<Point2f> get_centers(vector<Armor> target){
        int firstIndex = get_first_index(target);
        int lastIndex = get_last_index(target);
        cout << "firstIndex:" << firstIndex << "   lastIndex" <<lastIndex << endl;
        vector<Point2f> centers;
        for(int i= firstIndex; i<=lastIndex; i++){
            centers.push_back(target[i].center());
        }
        cout << "center's size:" <<centers.size() <<endl;
        return centers;
    } 

public:
    void track(vector<Armor> &armors,const int time,int skip_frame = 3,int distance = 30){

        //delete old target and add predict target
        for(int j = TrackingArmors.size()-1;j>-1;j--){
            for (int num : erasednumbers) {
                    if (num == j) {
                        found = true; 
                        break;
                    }
                }
            if (found) {
                found = false;
                continue;
            }

            TrackingArmors[j].push_back(predict(TrackingArmors[j]));
            cout << "predict" << endl;

            if ((time - get_last_time(TrackingArmors[j]) > skip_frame)) {
                // vector<vector<Armor>>::iterator it = TrackingArmors.begin() + j;
                // TrackingArmors.erase(it);
                erasednumbers.push_back(j);
                cout <<"erase" << endl;
            }   
        }

        //get new target and track old target
        for (int i = 0;i<armors.size();i++){
            for(int j = 0;j<TrackingArmors.size();j++){
                for (int num : erasednumbers) {
                    if (num == j) {
                        found = true; 
                        break;
                    }
                }
                if (found) {
                found = false;
                continue;
                }
                if (norm(armors[i].center()-TrackingArmors[j][time].center())<distance){
                    TrackingArmors[j][time] = armors[i];
                    armors[i].targetnumber = j+1;
                    armors[i].tracktarget = TrackingArmors[j][time].tracktarget = true;
                    TrackingArmors[j][time].isEmpty = false;
                }
            }

            if (armors[i].tracktarget == false){
                vector<Armor> newtarget;
                for(int i = 0;i<=time;i++)newtarget.push_back(blankarmor);
                newtarget[time] = armors[i];
                newtarget[time].isEmpty = false;
                newtarget[time].tracktarget = true;
                TrackingArmors.push_back(newtarget);
                cout << "creat" << endl;
                cout << "newtarget's time:" <<newtarget.size()-1 <<endl;
            }
        }
        cout << "num of target:" << TrackingArmors.size()-erasednumbers.size() << endl;

        //return predict target
        for (int j = 0;j<TrackingArmors.size();j++){
            for (int num : erasednumbers) {
                    if (num == j) {
                        found = true; 
                        break;
                    }
                }
            if (found) {
                found = false;
                continue;
            }
            if(!TrackingArmors[j][time].tracktarget){
                TrackingArmors[j][time].targetnumber = j+1;
                armors.push_back(TrackingArmors[j][time]);
            }
        }
    }
};


#endif
