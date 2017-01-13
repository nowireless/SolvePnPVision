//
// Created by ryan on 1/9/17.
//
#include <vector>
#include <iostream>

#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

//Is A right of B?
bool rightOf(Point2f a, Point2f b) {
    return a.x > b.x;
}

//Is A above of B?
bool aboveOf(Point2f a, Point2f b) {
    return a.y < b.y;
}

int main() {
    vector<Point2f> polyContour;
    polyContour.push_back(Point2f(822,272));
    polyContour.push_back(Point2f(739,276));
    polyContour.push_back(Point2f(738,552));
    polyContour.push_back(Point2f(831,550));

    double area[4];
    for(int i = 0; i < 4; i++) {
        area[i] = polyContour[i].x * polyContour[i].y;
    }

    int maxAreaPos = 0;
    int minAreaPos = 0;
    for(int i = 0; i < 4; i++) {
        if(area[maxAreaPos] < area[i]) {
            maxAreaPos = i;
        }
        if(area[minAreaPos] > area[i]) {
            minAreaPos = i;
        }
    }

    Point2f extremeBR = polyContour[maxAreaPos];
    Point2f extremeTL = polyContour[minAreaPos];

    int remainingCorners[2];
    int j = 0;
    for(int i = 0; i < 4; i++) {
        if(i == maxAreaPos || i == minAreaPos) continue;
        remainingCorners[j++] = i;
    }

    Point2f extremeBL;
    Point2f extremeTR;

    if(polyContour[remainingCorners[0]].x < polyContour[remainingCorners[1]].x) {
        extremeBL = polyContour[remainingCorners[0]];
        extremeTR = polyContour[remainingCorners[1]];
    } else {
        extremeBL = polyContour[remainingCorners[1]];
        extremeTR = polyContour[remainingCorners[0]];
    }


    vector<Point2f> imagePoints;
    imagePoints.push_back(extremeBR);
    imagePoints.push_back(extremeBL);
    imagePoints.push_back(extremeTL);
    imagePoints.push_back(extremeTR);

    cout <<"BR " << extremeBR << endl;//Y
    cout <<"BL " << extremeBL << endl;//Y
    cout <<"TL " << extremeTL << endl;//N
    cout <<"TR " << extremeTR << endl;//Y

//    for(int i = 0; i < 4; i++) {
//        Point2f point = imagePoints[i];
//        cout << i << " " << point << endl;
//    }

}