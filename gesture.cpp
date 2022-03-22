#include "gesture.h"

Gesture::Gesture()
{
    gestoStop.goodFingers.push_back({{26, 27, 28, 29}, 0.02, 0.0, 95}); // right index finger
    gestoStop.goodFingers.push_back({{30, 31, 32, 33}, 0.02, 0.0, 90}); // right middle f. jonty
    gestoStop.goodFingers.push_back({{34, 35, 36, 37}, 0.02, 0.0, 85});   // right ring finger
//    gestoStop.fingers.push_back({{38, 39, 40, 41}, 10, 0.02});   // right pinky finger
    gestoStop.execute = GESTO_STOP;

    gestoForward.goodFingers.push_back({{26, 27, 28, 29}, 0.02, 0.0, 180});
    gestoForward.goodFingers.push_back({{30, 31, 32, 33}, 0.02, 0.0, 180});
//    gestoForward.badFingers.push_back({{34, 35, 36, 37}, 10, 0.02, 80});
//    gestoForward.badFingers.push_back({{38, 39, 40, 41}, 10, 0.02, 80});
    gestoForward.execute = GESTO_FORWARD;

//    gestoBackward.badFingers.push_back({{26, 27, 28, 29}, 10, 0.02, 80});
//    gestoBackward.badFingers.push_back({{30, 31, 32, 33}, 10, 0.02, 80});
    gestoBackward.goodFingers.push_back({{34, 35, 36, 37}, 0.02, 0.0, 180});
    gestoBackward.goodFingers.push_back({{38, 39, 40, 41}, 0.02, 0.0, 180});
    gestoBackward.execute = GESTO_BACKWARD;

//    gestoLeft.badFingers.push_back({{26, 27, 28, 29}, 0.02, 0.0, 130});
    gestoLeft.goodFingers.push_back({{5, 6, 7, 8}, 0.02, 0.0, 130});
    gestoLeft.execute = GESTO_LEFT;

    gestoRight.goodFingers.push_back({{5, 6, 7, 8}, 0.02, 0.0, 50});
    gestoRight.execute = GESTO_RIGHT;

    allGests.push_back(gestoStop);
    allGests.push_back(gestoForward);
    allGests.push_back(gestoBackward);
    allGests.push_back(gestoLeft);
    allGests.push_back(gestoRight);

}

void Gesture::updateSkelet(skeleton kostricka){
    memcpy(skelet.joints,kostricka.joints,1800);
}

int Gesture::detectGestures(){
    int result = NULL;
    //STOP SIGN
    for(GestoStruct gesto : allGests){

        short goodIter = 0;
        short badIter = 0;
        for(Finger finger : gesto.goodFingers){
            vector<klb> finger_joints_coords;
            for(int k : finger.joints){
                finger_joints_coords.push_back(skelet.joints[k]);
            }
            if(isFingerInLine(finger_joints_coords, finger.max_thresh, finger.min_thresh) && skelet.joints[finger.joints[0]+goodIter].x != 0.0){
                if(isFingerGoodRotate(finger_joints_coords, finger.desired_angle)){
                    goodIter++;
                }else{
                    break;
                }
            }else{
                break;
            }
        }
        std::cout<<"======================="<<std::endl;
        if(goodIter == gesto.goodFingers.size()){
    //        gestoStop.execute = true;
            cout<<"DETEGOVANE GESTO:"<<endl;
            result = gesto.execute;
            switch(result)
            {
            case GESTO_STOP:
                cout<<">>STOP<<"<<endl;
                break;
            case GESTO_FORWARD:
                cout<<">>FORWARD<<"<<endl;
                break;
            case GESTO_BACKWARD:
                cout<<">>BACKWARD<<"<<endl;
                break;
            case GESTO_LEFT:
                cout<<">>LEFT<<"<<endl;
                break;
            case GESTO_RIGHT:
                cout<<">>RIGHT<<"<<endl;
                break;
            }

            return result;
        }
        std::cout<<"======================="<<std::endl;

    }

    if(result == NULL){
        result = NON_DETECT;
        cout<<"ZIADNE GESTO DETEGOVANE"<<endl;
    }


    return result;
}

bool Gesture::isFingerGoodRotate(std::vector<klb> finger_joints, double desired_angle){
    if(finger_joints.size() < 2){
        return false;
    }
    double angle, angle_threshhold = 25; // uhol tu pocitam v stupnoch
    angle = getDegreeAngleOfLine(finger_joints.at(0).x, finger_joints.at(0).y, finger_joints.at(finger_joints.size()-1).x, finger_joints.at(finger_joints.size()-1).y);

    if(desired_angle + angle_threshhold > angle && angle > desired_angle - angle_threshhold){
        return true;
    }else{
        return false;
    }
}

bool Gesture::isFingerInLine(std::vector<klb> finger_joints, double max_thresh, double min_thresh){
    if(finger_joints.size()<4){

        return false;
    }
//    double threshhold = 0.007;

    for(int i = 1; i<3; i++){
        double tempx, tempy;
        tempx = finger_joints.at(i).x;
        tempy = finger_joints.at(i).y;

        double tempdist = pDistFromLine(tempx, tempy, finger_joints.at(0).x, finger_joints.at(0).y, finger_joints.at(3).x, finger_joints.at(3).y);
        std::cout<<"Temp dist: "<<tempdist<<std::endl;
        if(tempdist > max_thresh || tempdist < min_thresh){
            return false;
        }
    }
    return true;
}

double Gesture::getDegreeAngleOfLine(double x1, double y1, double x2, double y2){
    double angle;
    angle = atan2(y2 - y1, x2- x1);
    if(angle < 0) angle += 3.14159;
    return angle*180/3.14159;
}


double Gesture::pDistFromLine(double x, double y, double x1, double y1, double x2, double y2){

    double A = x - x1;
    double B = y - y1;
    double C = x2 - x1;
    double D = y2 - y1;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = -1;
    if (len_sq != 0) //in case of 0 length line
        param = dot / len_sq;

    double xx, yy;

    if (param < 0) {
      xx = x1;
      yy = y1;
    }
    else if (param > 1) {
      xx = x2;
      yy = y2;
    }
    else {
      xx = x1 + param * C;
      yy = y1 + param * D;
    }

    double dx = x - xx;
    double dy = y - yy;
    return sqrt(dx * dx + dy * dy);
}
