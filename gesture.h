#ifndef GESTURE_H
#define GESTURE_H

#include<iostream>
#include <vector>

#define NUM_SKEL_JOINTS 75

#define NON_DETECT -1
#define GESTO_STOP 0
#define GESTO_FORWARD 1
#define GESTO_BACKWARD 2

#define GESTO_RIGHT 3
#define GESTO_LEFT 4


using namespace std;


typedef struct
{
    double x;
    double y;
    double z;
}klb;

typedef struct
{
    klb joints[75];
}skeleton;

typedef struct
{
//    klb joints[4];
    int joints[4];
    double max_thresh;
    double min_thresh;
    double desired_angle;
}Finger;

typedef struct
{
    vector<Finger> goodFingers;
    vector<Finger> badFingers;
//    double max_thresh;
//    double min_thresh;
    int execute;
}GestoStruct;

//typedef struct
//{
//    vector<GestoStruct>
//}AllGests;

class Gesture
{
public:
    Gesture();

    skeleton skelet;

    void updateSkelet(skeleton kostricka);
    double pDistFromLine(double x, double y, double x1, double y1, double x2, double y2);
    double getDegreeAngleOfLine(double x1, double y1, double x2, double y2);

    bool isFingerInLine(std::vector<klb> finger_jointsm, double max_thresh, double min_thresh = 0.0);
    bool isFingerGoodRotate(std::vector<klb> finger_joints, double desired_angle);
    int detectGestures();

//    vector<vector<klb>, vector<double>> gesto1;
//    vector<vector<int>, vector<double>> prst1;
//    vector<vector<vector<int>, vector<double>>> gesto;
    GestoStruct gestoStop;
    GestoStruct gestoForward;
    GestoStruct gestoBackward;
    GestoStruct gestoRight;
    GestoStruct gestoLeft;

    vector<GestoStruct> allGests;
};

#endif // GESTURE_H
