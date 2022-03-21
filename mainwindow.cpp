#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QTimer"
#include "QPainter"
#include "math.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include <QCoreApplication>
#include <QtConcurrent/QtConcurrent>

using namespace cv;
using namespace std;


//funkcia local robot je na priame riadenie robota, ktory je vo vasej blizskoti, viete si z dat ratat polohu atd (zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
void MainWindow::localrobot(TKobukiData &sens)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
    ///
    ///
    ///
    ///
    ///
    ///



    if(prvyStart)
    {

        GyroUholOld=sens.GyroAngle;
        PomEncoderL= sens.EncoderLeft;
        PomEncoderR= sens.EncoderRight;


        deltaUhol=0.0;
        prvyStart=false;
    }



    PomEncoderL=sens.EncoderLeft;
    if(dl%10==0)
    {
        ///toto je skaredy kod. rozumne je to posielat do ui cez signal slot..
//        ui->lineEdit->setText(QString::number(robotX));
//        ui->lineEdit_2->setText(QString::number(robotY));
//        ui->lineEdit_3->setText(QString::number(robotFi));
    }
    dl++;








}

cv::Mat MainWindow::getLidarFusion(cv::Mat oldFrameBuff){
    cv::Mat frame = oldFrameBuff;

    cv::Mat edgeFrame;
//    Mat thresh;

    for (int i = 0; i < paintLaserData.numberOfScans; i++) {
//        std::cout << "Dist["<<i<<"]: " << laserData.Data[i].scanDistance << " Angle["<<i<<"]: " << laserData.Data[i].scanAngle << std::endl;

        double D = paintLaserData.Data[i].scanDistance/1000;
        double alpha = (360 - paintLaserData.Data[i].scanAngle)*PI/180;
        if(alpha > 2*PI-(fusionData.camWidthAngle/2) || alpha < fusionData.camWidthAngle/2){
//            std::cout << "Citam"<< std::endl;
            double Z = D * cos(alpha);
            double X = D * sin(alpha);

            fusionData.xobr = (frame.cols/2) - (int)((fusionData.f * X)/Z);
            fusionData.yobr = (frame.rows/2) + (int)((fusionData.f * fusionData.Y)/Z);

//            std::cout << "X: " << fusionData.xobr << "; Y: " << fusionData.yobr << std::endl;

            cv::cvtColor(frame, edgeFrame, cv::COLOR_RGB2GRAY);
            cv::Canny(edgeFrame, edgeFrame, 50, 100);
//            cv::cvtColor(edgeFrame, edgeFrame, cv::COLOR_GRAY2BGR);

//            threshold(edgeFrame, thresh, 150, 255, THRESH_BINARY);

//            vector<vector<cv::Point> > contours;
////            vector<vector<cv::Point> > goodContours;
//            vector<cv::Vec4i> hierarchy;
//            findContours(edgeFrame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );


            vector<XY> lowerPoints;
//            vector<XY> upperPoints;
            int lx;
            int ly;
            int ux;
            int uy;
            int x = fusionData.xobr;
//            int y = fusionData.yobr;
            for(int j = fusionData.yobr; j<edgeFrame.rows; j++){
////                Vec3b &color = edgeFrame.at<Vec3b>(fusionData.xobr, j);
//                BGR& bgr = edgeFrame.ptr<BGR>(j)[x];
//                cout<<bgr.blue << ", " << bgr.green << ", " << bgr.red << endl;

                if(edgeFrame.at<uchar>(j, x) == 255){
//                    lowerPoints.push_back({x, j});
                    lx = x;
                    ly = j;
                    break;
                }
            }
            for(int j = fusionData.yobr; j>=0; j--){
                if(edgeFrame.at<uchar>(j, x) == 255){
//                    upperPoints.push_back({fusionData.xobr, j});
                    ux = x;
                    uy = j;
                    break;
                }else{
                }
            }
            cv::Rect rectl(lx, ly, 4, 4);
            cv::rectangle(frame, rectl, cv::Scalar(255, 0, 255));
            cv::Rect rectu(ux, uy, 4, 4);
            cv::rectangle(frame, rectu, cv::Scalar(255, 0, 255));

//            return edgeFrame;
//            return contImg;

            cv::Rect rect(fusionData.xobr, fusionData.yobr, 4, 4);
            if(D < 0.5){
                cv::rectangle(frame, rect, cv::Scalar(0, 0, 255));
            }else if(D >= 0.5 && D < 1.2){
                cv::rectangle(frame, rect, cv::Scalar(255, 255, 0));
            }else{
                cv::rectangle(frame, rect, cv::Scalar(255, 0, 0));
            }
        }
    }

//        cv::Mat edgeFrame;
//        cv::cvtColor(frame, edgeFrame, cv::COLOR_RGB2GRAY);
//        cv::Canny(edgeFrame, edgeFrame, 50, 100);

////        Mat thresh;
////        threshold(edgeFrame, thresh, 150, 255, THRESH_BINARY);


//        vector<vector<cv::Point> > contours;
//        vector<vector<cv::Point> > goodContours;
//        vector<cv::Vec4i> hierarchy;
//        findContours(edgeFrame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

//        for(int j = 0; j < edgeFrame.cols; j++){
//            for(int i = 0; i < edgeFrame.rows; i++){

//            }
//        }



//        int xc, yc;
//        RNG rng(12345);
//        for (int i = 0; i < contours.size(); ++i) {
//            vector<cv::Point> current;
//            cout << "Area: " << contourArea(contours[i]) << endl;
//            if (contourArea(contours[i]) > 5000 && contourArea(contours[i]) < 190000) {

//                approxPolyDP(contours[i], current, 0.01 * arcLength(contours[i], true), true);
//                Moments m = cv::moments(contours[i]);
//                if(m.m00 != 0.0){
//                    xc = m.m10/m.m00;
//                    yc = m.m01/m.m00;
//                }
//                if(current.size() == 4){
//                    goodContours.push_back(contours[i]);
//                    Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
//                    drawContours(frame, contours, i, /*Scalar(0, 0, 255)*/color, 5, LINE_8, hierarchy, 0);
//                    putText(edgeFrame, "Stvorec", Point(xc, yc), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
//                }
//            }
//        }

//        drawContours(frame, goodContours, -1, Scalar(0, 0, 255), -1);
//        cv::cvtColor(edgeFrame, edgeFrame, cv::COLOR_GRAY2BGR);
//            Mat contImg;
//            edgeFrame.copyTo(contImg);

//            vector<vector<Point>> polyCurves;
        //for (int i = 0; i < goodContours.size(); ++i) {
//                current.push_back(contours[i]);
//                for(int j = 0; j<contours[i].size(); j++){
//                }

//            if(current.size() == 4){

//            }else{
//                putText(edgeFrame, "Nieco INE", Point(xc, yc), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
//            }


//                if(current.size() > 0){
//                    polyCurves.push_back(current);
//                }

        //}


//    edgeFrame.copyTo(frame);
    return frame;
}

// funkcia local laser je naspracovanie dat z lasera(zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
int MainWindow::locallaser(LaserMeasurement &laserData)
{


    if(fusionData.prvyStart){
        fusionData.altLidar = 0.21;
        fusionData.altCamera = 0.15;
        fusionData.Y = fusionData.altLidar - fusionData.altCamera;
        fusionData.camWidthAngle = 54*PI/180;
        fusionData.camHeightAngle = 40*PI/180;
        fusionData.f = 628.036;

        fusionData.prvyStart = false;
    }

    //priklad ako zastavit robot ak je nieco prilis blizko
    if(laserData.Data[0].scanDistance<500)
    {

        sendRobotCommand(ROBOT_STOP);
    }

    paintThisLidar(laserData);
    ///PASTE YOUR CODE HERE
    /// ****************
    /// mozne hodnoty v return
    /// ROBOT_VPRED
    /// ROBOT_VZAD
    /// ROBOT_VLAVO
    /// ROBOT_VPRAVO
    /// ROBOT_STOP
    /// ROBOT_ARC
    ///
    /// ****************

    return -1;
}


//--autonomousrobot simuluje slucku robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad polohovy regulator, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
void MainWindow::autonomousrobot(TKobukiData &sens)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
}
//--autonomouslaser simuluje spracovanie dat z robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad sposob obchadzania prekazky, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
int MainWindow::autonomouslaser(LaserMeasurement &laserData)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
    return -1;
}

///kamera nema svoju vlastnu funkciu ktora sa vola, ak chcete niekde pouzit obrazok, aktualny je v premennej
/// robotPicture alebo ekvivalent AutonomousrobotPicture
/// pozor na synchronizaciu, odporucam akonahle chcete robit nieco s obrazkom urobit si jeho lokalnu kopiu
/// cv::Mat frameBuf; robotPicture.copyTo(frameBuf);


//sposob kreslenia na obrazovku, tento event sa spusti vzdy ked sa bud zavola funkcia update() alebo operacny system si vyziada prekreslenie okna

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
//    painter2.setBrush(Qt::black);
    QPen pero_lidar;
    pero_lidar.setStyle(Qt::SolidLine);
    pero_lidar.setWidth(3);
    pero_lidar.setColor(Qt::green);

    QPen pero_joints;
    pero_joints.setColor(Qt::cyan);

    QRect rect(20,120,700,500);
    QRect rect2(20,120,700,500);
    QRect rect3(20,120,700,500);
    rect = ui->camera->geometry();
    rect2 = ui->lidar->geometry();
    rect3 = ui->skeleton->geometry();

//    rect2.translate(0, 15);
    painter.drawRect(rect);
    painter.drawRect(rect2);
    painter.drawRect(rect3);

    bool sync = false;
    if(updateCameraPicture==1 && showCamera==true)
    {
        updateCameraPicture=0;
        cv::resize(robotPicture, robotPicture, cv::Size(rect.width(), rect.height()), 0, 0, cv::INTER_CUBIC);
        QImage imgIn= QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);

        painter.drawImage(rect.topLeft().x(), rect.topLeft().y(), imgIn);
//        painter.drawImage(0, 0, imgIn);
      //  cv::imshow("client",robotPicture);
    }

    if(updateLaserPicture==1 && showLidar==true)
    {
        /// ****************
        ///you can change pen or pen color here if you want
        /// ****************
//        std::cout << "Lidar kreslim" << std::endl;
        painter.setPen(pero_lidar);
        for(int k=0;k<paintLaserData.numberOfScans;k++)
        {
            int dist=paintLaserData.Data[k].scanDistance/30;
            int xp=rect2.width()-(rect2.width()/2 + dist*sin((360.0-paintLaserData.Data[k].scanAngle)*PI/180.0))+rect2.topLeft().x();
            int yp=rect2.height()-(rect2.height()/2 + dist*cos((360.0-paintLaserData.Data[k].scanAngle)*PI/180.0))+rect2.topLeft().y();
            if(rect2.contains(xp,yp)){
//                std::cout << "Contains" << std::endl;
                painter.drawEllipse(QPoint(xp, yp),2,2);
            }
            //if(xp<rect2.width()+1 && xp>19 && yp<rect2.height()+1 && yp>121)



        }
    }
    if(updateSkeletonPicture==1 && showSkeleton==true)
    {
//        updateSkeletonPicture = 0;

//        painter.setPen(Qt::red);
        painter.setPen(pero_joints);
        for(int i=0;i<75;i++)
        {
            int xp/*=720-720 * kostricka.joints[i].x*/;
            int yp/*=120+ 500 *kostricka.joints[i].y*/;
//            std::cout << klby[0] << ": " << kostricka.joints[0].x << kostricka.joints[0].y << std::endl;
            xp = rect3.width() - rect3.width() * kostricka.joints[i].x + rect3.topLeft().x();
            yp = rect3.height() * kostricka.joints[i].y + rect3.topLeft().y();
            if(rect3.contains(xp,yp)){
                painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }

}



///konstruktor aplikacie, nic co tu je nevymazavajte, ak potrebujete, mozete si tu pridat nejake inicializacne parametre
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    robotX=0;
    robotY=0;
    robotFi=0;

    showCamera=true;
    showLidar=true;
    showSkeleton=true;
    applyDelay=false;
    dl=0;
    stopall=1;
    prvyStart=true;
    fusionData.prvyStart = true;
    updateCameraPicture=0;
    ipaddress="127.0.0.1";
    std::function<void(void)> f =std::bind(&robotUDPVlakno, (void *)this);
    robotthreadHandle=std::move(std::thread(f));
    std::function<void(void)> f2 =std::bind(&laserUDPVlakno, (void *)this);
    laserthreadHandle=std::move(std::thread(f2));


    std::function<void(void)> f3 =std::bind(&skeletonUDPVlakno, (void *)this);
    skeletonthreadHandle=std::move(std::thread(f3));

    //--ak by ste nahodou chceli konzolu do ktorej mozete vypisovat cez std::cout, odkomentujte nasledujuce dva riadky
   // AllocConsole();
   // freopen("CONOUT$", "w", stdout);


    QFuture<void> future = QtConcurrent::run([=]() {
        imageViewer();
        // Code in this block will run in another thread
    });



        Imager.start();

}

///funkcia co sa zavola ked stlacite klavesu na klavesnici..
/// pozor, ak niektory widget akceptuje klavesu, sem sa nemusite (ale mozete) dostat
/// zalezi na to ako konkretny widget spracuje svoj event
void MainWindow::keyPressEvent(QKeyEvent* event)
{
    //pre pismena je key ekvivalent ich ascii hodnoty
    //pre ine klavesy pozrite tu: https://doc.qt.io/qt-5/qt.html#Key-enum
    std::cout<<event->key()<<std::endl;
    if(event->key() == Qt::Key_A){
        sendRobotCommand(ROBOT_VLAVO,3.14159/4);
    }else if(event->key() == Qt::Key_D){
        sendRobotCommand(ROBOT_VPRAVO,-3.14159/4);
    }else if(event->key() == Qt::Key_W){
        sendRobotCommand(ROBOT_VPRED,300);
    }else if(event->key() == Qt::Key_S){
        sendRobotCommand(ROBOT_VZAD,-250);
    }else if(event->key() == Qt::Key_Q){
        sendRobotCommand(ROBOT_STOP,0);
    }


}
//--cokolvek za tymto vas teoreticky nemusi zaujimat, su tam len nejake skarede kody































































MainWindow::~MainWindow()
{
    stopall=0;
    laserthreadHandle.join();
    robotthreadHandle.join();
    skeletonthreadHandle.join();
    delete ui;
}









void MainWindow::robotprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    rob_slen = sizeof(las_si_other);
    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    DWORD timeout=100;

    setsockopt(rob_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
#ifdef _WIN32
    Sleep(100);
#else
    usleep(100*1000);
#endif
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(stopall==1)
    {

        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);

        int returnval=robot.fillData(sens,(unsigned char*)buff);
        if(returnval==0)
        {
            //     memcpy(&sens,buff,sizeof(sens));

            std::chrono::steady_clock::time_point timestampf=std::chrono::steady_clock::now();

            autonomousrobot(sens);

            if(applyDelay==true)
            {
                struct timespec t;
                RobotData newcommand;
                newcommand.sens=sens;
                //    memcpy(&newcommand.sens,&sens,sizeof(TKobukiData));
                //        clock_gettime(CLOCK_REALTIME,&t);
                newcommand.timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                sensorQuerry.push_back(newcommand);
                for(int i=0;i<sensorQuerry.size();i++)
                {
                    if(( std::chrono::duration_cast<std::chrono::nanoseconds>(timestampf-sensorQuerry[i].timestamp)).count()>(2.5*1000000000))
                    {
                        localrobot(sensorQuerry[i].sens);
                        sensorQuerry.erase(sensorQuerry.begin()+i);
                        i--;
                        break;

                    }
                }

            }
            else
            {
                sensorQuerry.clear();
                localrobot(sens);
            }
        }


    }

    std::cout<<"koniec thread2"<<std::endl;
}
/// vravel som ze vas to nemusi zaujimat. tu nic nieje
/// nosy litlle bastard
void MainWindow::laserprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char las_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    setsockopt(las_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#else
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#endif
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, rob_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(stopall==1)
    {

        if ((las_recv_len = recvfrom(las_s, (char *)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, &las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        int returnValue=autonomouslaser(measure);

        if(applyDelay==true)
        {
            struct timespec t;
            LidarVector newcommand;
            memcpy(&newcommand.data,&measure,sizeof(LaserMeasurement));
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            lidarQuerry.push_back(newcommand);
            for(int i=0;i<lidarQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-lidarQuerry[i].timestamp)).count()>(2.5*1000000000))
                {
                    returnValue=locallaser(lidarQuerry[i].data);
                    if(returnValue!=-1)
                    {
                        //sendRobotCommand(returnValue);
                    }
                    lidarQuerry.erase(lidarQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {


            returnValue=locallaser(measure);
            if(returnValue!=-1)
            {
                //sendRobotCommand(returnValue);
            }
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}

void MainWindow::on_pushButton_3_clicked()
{
    char tt=0x01;
    sendRobotCommand(tt,250);
}

void MainWindow::on_pushButton_7_clicked()
{
    char tt=0x00;
    sendRobotCommand(tt);

}

void MainWindow::on_pushButton_5_clicked()
{
    char tt=0x02;
    sendRobotCommand(tt,-250);
}


void MainWindow::on_pushButton_4_clicked()
{
    char tt=0x04;
    sendRobotCommand(tt,3.14159/4);
}

void MainWindow::on_pushButton_6_clicked()
{
    char tt=0x03;
    sendRobotCommand(tt,-3.14159/4);
}

void MainWindow::on_pushButton_clicked()
{

}

void MainWindow::sendRobotCommand(char command,double speed,int radius)
{
    globalcommand=command;
 //   if(applyDelay==false)
    {

        std::vector<unsigned char> mess;
        switch(command)
        {
        case  ROBOT_VPRED:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VZAD:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VLAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_VPRAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_STOP:
            mess=robot.setTranslationSpeed(0);
            break;
        case ROBOT_ARC:
            mess=robot.setArcSpeed(speed,radius);
            break;


        }
        if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
        {

        }
    }
  /*  else
    {
        struct timespec t;
        RobotCommand newcommand;
        newcommand.command=command;
        newcommand.radius=radius;
        newcommand.speed=speed;
        //clock_gettime(CLOCK_REALTIME,&t);
        newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
        commandQuery.push_back(newcommand);
    }*/
}
/*void MainWindow::autonomousRobotCommand(char command,double speed,int radius)
{
    return;
    std::vector<unsigned char> mess;
    switch(command)
    {
    case  ROBOT_VPRED:
        mess=robot.setTranslationSpeed(speed);
        break;
    case ROBOT_VZAD:
        mess=robot.setTranslationSpeed(speed);
        break;
    case ROBOT_VLAVO:
        mess=robot.setRotationSpeed(speed);
        break;
    case ROBOT_VPRAVO:
        mess=robot.setRotationSpeed(speed);
        break;
    case ROBOT_STOP:
        mess=robot.setTranslationSpeed(0);
        break;
    case ROBOT_ARC:
        mess=robot.setArcSpeed(speed,radius);
        break;

    }
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}
void MainWindow::robotexec()
{


    if(applyDelay==true)
    {
        struct timespec t;

        // clock_gettime(CLOCK_REALTIME,&t);
        auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
        for(int i=0;i<commandQuery.size();i++)
        {
       //     std::cout<<(std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()<<std::endl;
            if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()>(2.5*1000000000))
            {
                char cmd=commandQuery[i].command;
                std::vector<unsigned char> mess;
                switch(cmd)
                {
                case  ROBOT_VPRED:
                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VZAD:
                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VLAVO:
                    mess=robot.setRotationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VPRAVO:
                    mess=robot.setRotationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_STOP:
                    mess=robot.setTranslationSpeed(0);
                    break;
                case ROBOT_ARC:
                    mess=robot.setArcSpeed(commandQuery[i].speed,commandQuery[i].radius);
                    break;

                }
                if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
                {

                }
                commandQuery.erase(commandQuery.begin()+i);
                i--;

            }
        }
    }
}
*/


void MainWindow::paintThisLidar(LaserMeasurement &laserData)
{
    memcpy( &paintLaserData,&laserData,sizeof(LaserMeasurement));
    updateLaserPicture=1;
//    update();
}

void MainWindow::on_pushButton_8_clicked()//forward
{
    CommandVector help;
    help.command.commandType=1;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=0;
    help.command.desiredDist=100;
    struct timespec t;

    // clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}

void MainWindow::on_pushButton_10_clicked()//right
{
    CommandVector help;
    help.command.commandType=2;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=-20;
    help.command.desiredDist=0;
    struct timespec t;

    // clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}

void MainWindow::on_pushButton_11_clicked()//back
{
    CommandVector help;
    help.command.commandType=1;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=0;
    help.command.desiredDist=-100;
    struct timespec t;

    //   clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}

void MainWindow::on_pushButton_9_clicked()//left
{
    CommandVector help;
    help.command.commandType=2;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=20;
    help.command.desiredDist=0;
    struct timespec t;

    //   clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}

void MainWindow::on_pushButton_12_clicked()
{
//    zX=ui->lineEdit_4->text().toDouble();
//    zY=ui->lineEdit_5->text().toDouble();
    toleranciaUhla=2;
    naviguj=true;
}

void MainWindow::on_pushButton_13_clicked()
{
//    ui->lineEdit_4->setText("stlacil sa gombik");
}


void MainWindow::skeletonprocess()
{

    std::cout<<"init skeleton"<<std::endl;
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    ske_slen = sizeof(ske_si_other);
    if ((ske_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char ske_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    std::cout<<setsockopt(ske_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout)<<std::endl;
    std::cout<<setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene))<<std::endl;
#else
    setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene));
#endif
    // zero out the structure
    memset((char *) &ske_si_me, 0, sizeof(ske_si_me));

    ske_si_me.sin_family = AF_INET;
    ske_si_me.sin_port = htons(23432);
    ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    ske_si_posli.sin_family = AF_INET;
    ske_si_posli.sin_port = htons(23432);
    ske_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    std::cout<<::bind(ske_s , (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) )<<std::endl;;
    char command=0x00;

    skeleton bbbk;
    double measure[225];
    while(stopall==1)
    {

        if ((ske_recv_len = ::recvfrom(ske_s, (char *)&bbbk.joints, sizeof(char)*1800, 0, (struct sockaddr *) &ske_si_other, &ske_slen)) == -1)
        {

        //    std::cout<<"problem s prijatim"<<std::endl;
            continue;
        }


        memcpy(kostricka.joints,bbbk.joints,1800);
     updateSkeletonPicture=1;
  //      std::cout<<"doslo "<<ske_recv_len<<std::endl;
      //  continue;
        for(int i=0;i<75;i+=3)
        {
//            std::cout<<klby[i]<<" "<<bbbk.joints[i].x<<" "<<bbbk.joints[i].y<<" "<<bbbk.joints[i].z<<std::endl;
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}

void MainWindow::on_checkBox_2_clicked(bool checked)
{
    showLidar=checked;
}


void MainWindow::on_checkBox_3_clicked(bool checked)
{
    showCamera=checked;
}


void MainWindow::on_checkBox_4_clicked(bool checked)
{
    showSkeleton=checked;
}


void MainWindow::on_checkBox_clicked(bool checked)
{
    applyDelay=checked;
}

void MainWindow::imageViewer()
{
    cv::VideoCapture cap;
    cap.open("http://127.0.0.1:8889/stream.mjpg");
    cv::Mat frameBuf;
    while(1)
    {
        cap >> frameBuf;


        if(frameBuf.rows<=0)
        {
            std::cout<<"nefunguje"<<std::endl;
            continue;
        }

        if(applyDelay==true)
        {
            struct timespec t;
            CameraVector newcommand;
            frameBuf.copyTo(newcommand.data);
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            cameraQuerry.push_back(newcommand);
            for(int i=0;i<cameraQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-cameraQuerry[i].timestamp)).count()>(2.5*1000000000))
                {

                    cameraQuerry[i].data.copyTo(robotPicture);
                    cameraQuerry.erase(cameraQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {

           frameBuf = getLidarFusion(frameBuf);
           frameBuf.copyTo(robotPicture);
        }
        frameBuf.copyTo(AutonomousrobotPicture);
        updateCameraPicture=1;

        update();
        std::cout<<"vycital som"<<std::endl;
       // cv::imshow("client",frameBuf);
        cv::waitKey(1);
        QCoreApplication::processEvents();
    }
}



void MainWindow::on_checkBox_lidar_clicked(bool checked)
{
    showLidar=checked;
}


void MainWindow::on_checkBox_skeleton_clicked(bool checked)
{
    showSkeleton = checked;
}


void MainWindow::on_checkBox_cam_clicked(bool checked)
{
    showCamera = checked;
}


void MainWindow::on_stop_button_clicked()
{
    sendRobotCommand(ROBOT_STOP);
}


void MainWindow::on_pushButton_forward_clicked()
{
    sendRobotCommand(ROBOT_VPRED, 350);
}


void MainWindow::on_pushButton_backward_clicked()
{
    sendRobotCommand(ROBOT_VZAD, -250);
}


void MainWindow::on_pushButton_right_clicked()
{
    sendRobotCommand(ROBOT_VPRAVO, -PI/4);
}


void MainWindow::on_pushButton_left_clicked()
{
    sendRobotCommand(ROBOT_VLAVO, PI/4);
}
