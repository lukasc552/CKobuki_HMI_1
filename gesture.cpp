#include "gesture.h"

Gesture::Gesture()
{
    gestoStop.fingers.push_back({{26, 27, 28, 29}, 0.007, 0.0});
    gestoStop.fingers.push_back({{30, 31, 32, 33}, 0.007, 0.0}); // pravy middle f. jonty
    gestoStop.fingers.push_back({{34, 35, 36, 37}, 10, 0.02});
    gestoStop.fingers.push_back({{38, 39, 40, 41}, 10, 0.02});


}

void Gesture::updateSkelet(skeleton kostricka){
    memcpy(skelet.joints,kostricka.joints,1800);
}

int Gesture::detectGestures(){
    int result;
    //STOP SIGN
    short iter = 0;
    for(Finger finger : gestoStop.fingers){
        vector<klb> finger_joints_coords;
        for(int k : finger.joints){
            finger_joints_coords.push_back(skelet.joints[k]);
        }
        if(isFingerInLine(finger_joints_coords, finger.max_thresh, finger.min_thresh) && skelet.joints[finger.joints[0]+iter].x != 0.0){
            iter++;
//            continue;
        }else{
            break;
        }
    }
    std::cout<<"======================="<<std::endl;
    if(iter == gestoStop.fingers.size()){
//        gestoStop.execute = true;
        cout<<"DOBRE"<<endl;
        result = GEST_STOP;
    }else{
        cout<<"ZLE"<<endl;
        result = NON_DETECT;
    }
    std::cout<<"======================="<<std::endl;
    return result;
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
