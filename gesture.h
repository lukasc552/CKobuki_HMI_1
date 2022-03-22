#ifndef GESTURE_H
#define GESTURE_H

#include<iostream>
#include <vector>

#define NUM_SKEL_JOINTS 75

#define NON_DETECT -1
#define GEST_STOP 0
#define GEST_FORWARD 1
#define GEST_BACKWARD 2

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
}Finger;

typedef struct
{
    vector<Finger> fingers;
//    double max_thresh;
//    double min_thresh;
    bool execute;
}GestoStruct;

class Gesture
{
public:
    Gesture();

    skeleton skelet;

    void updateSkelet(skeleton kostricka);
    double pDistFromLine(double x, double y, double x1, double y1, double x2, double y2);
    bool isFingerInLine(std::vector<klb> finger_jointsm, double max_thresh, double min_thresh = 0.0);
    int detectGestures();

//    vector<vector<klb>, vector<double>> gesto1;
//    vector<vector<int>, vector<double>> prst1;
//    vector<vector<vector<int>, vector<double>>> gesto;
    GestoStruct gestoStop;
    GestoStruct gestoForward;
    GestoStruct gestoBackward;
};

#endif // GESTURE_H
