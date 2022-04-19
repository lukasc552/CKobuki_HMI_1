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

    cv::cvtColor(frame, edgeFrame, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(edgeFrame, edgeFrame, Size(3, 3), 2);
    cv::Canny(edgeFrame, edgeFrame, 100, 150);

    short count_collisions = 0;
    for (int i = 0; i < paintLaserData.numberOfScans; i++) {
//        std::cout << "Dist["<<i<<"]: " << laserData.Data[i].scanDistance << " Angle["<<i<<"]: " << laserData.Data[i].scanAngle << std::endl;

        if(paintLaserData.Data[i].scanDistance < 100) continue;

        double D = paintLaserData.Data[i].scanDistance/1000;
        if(D < 0.3){
            count_collisions++;
        }
        double alpha = (360 - paintLaserData.Data[i].scanAngle)*PI/180;
        if(alpha > 2*PI-(fusionData.camWidthAngle/2) || alpha < fusionData.camWidthAngle/2){

            double Z = D * cos(alpha);
            double X = D * sin(alpha);

            fusionData.xobr = (frame.cols/2) - (int)((fusionData.f * X)/Z);
            fusionData.yobr = (frame.rows/2) + (int)((fusionData.f * fusionData.Y)/Z);


            int lx;
            int ly;
            int ux;
            int uy;
            int x = fusionData.xobr;
            for(int j = fusionData.yobr; j<edgeFrame.rows; j++){

                if(edgeFrame.at<uchar>(j, x) == 255){
                    lx = x;
                    ly = j;
                    break;
                }
            }
            for(int j = fusionData.yobr; j>=0; j--){
                if(edgeFrame.at<uchar>(j, x) == 255){
                    ux = x;
                    uy = j;
                    break;
                }
            }
//            cv::Rect rectl(lx, ly, 4, 4);
//            cv::rectangle(frame, rectl, cv::Scalar(255, 0, 255));
//            cv::Rect rectu(ux, uy, 4, 4);
//            cv::rectangle(frame, rectu, cv::Scalar(255, 0, 255));

            short b = 24;
            short g;
            short r = 222;
            g = b + (short)((D-0.5)*(r-b)/(1.6-0.5));
            Scalar color;

            if(D < 0.5){
                color = Scalar(b, b, r);
            }else if(D >= 0.5 && D < 1.6){
                color = Scalar(b, g, r);
            }else{
                color = Scalar(b, r, r);
            }
            if(lx == ux){
                cv::line(frame, Point(lx, ly-10), Point(ux, uy+10), color, 15, 1);
            }


//            points.lower.push_back({lx, ly});
//            points.upper.push_back({ux, uy});
//            points.dist.push_back(paintLaserData.Data[i].scanDistance);

//            return edgeFrame;
//            return contImg;

//            cv::Rect rect(fusionData.xobr, fusionData.yobr, 4, 4);
//            if(D < 0.5){
//                cv::rectangle(frame, rect, cv::Scalar(0, 0, 255));
//            }else if(D >= 0.5 && D < 1.2){
//                cv::rectangle(frame, rect, cv::Scalar(255, 255, 0));
//            }else{
//                cv::rectangle(frame, rect, cv::Scalar(255, 0, 0));
//            }

        }

    }

    //===================================
    if(count_collisions > 0){
        collision = true;
    }else{
        collision = false;
    }

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
        fusionData.camWidthAngle = (54-2)*PI/180;
        fusionData.camHeightAngle = 40*PI/180;
        fusionData.f = 628.036;

        fusionData.prvyStart = false;
    }

    //priklad ako zastavit robot ak je nieco prilis blizko
    if(laserData.Data[0].scanDistance<200)
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
//    painter.setBrush(Qt::)
//    QPainter paint(this);
    QPen pen;
    pen.setWidth(7);
    pen.setColor(Qt::red);
//    painter2.setBrush(Qt::black);
    QPen pero_lidar;
    pero_lidar.setStyle(Qt::SolidLine);
    pero_lidar.setWidth(1);
    pero_lidar.setColor(Qt::green);

    QPen pero_irobot;
    pero_irobot.setStyle(Qt::SolidLine);
    pero_irobot.setWidth(4);
    pero_irobot.setColor(Qt::cyan);

    QPen pero_joints;
    pero_joints.setColor(Qt::cyan);

    QPen penText;
    penText.setColor(QColor(Qt::gray));

    QPen penTextCollision;
    penTextCollision.setColor(QColor(Qt::black));

    QRect rect;
//    QRect rect2(20,120,700,500);
//    QRect rect3(20,120,700,500);
    rect = ui->camera->geometry();
//    rect2 = ui->lidar->geometry();
//    rect3 = ui->skeleton->geometry();

//    rect2.translate(0, 15);

//    painter.drawRect(rect);

//    painter.drawRect(rect2);
//    painter.drawRect(rect3);

    if(updateCameraPicture==1 && showCamera==true)
    {
//        cout<<"Camera"<<endl;

        updateCameraPicture=0;
        cv::resize(robotPicture, robotPicture, cv::Size(rect.width(), rect.height()), 0, 0, cv::INTER_CUBIC);
        QImage imgIn= QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);

        painter.drawImage(rect.topLeft().x(), rect.topLeft().y()+5, imgIn);

        QImage imageSTOP;
        QImage imageForward;
        QImage imageBackward;
        QImage imageLeft;
        QImage imageRight;

        imageSTOP = QImage((uchar*) imgStop.data, imgStop.cols, imgStop.rows, imgStop.step, QImage::Format_BGR888);
        imageForward = QImage((uchar*) imgForward.data, imgForward.cols, imgForward.rows, imgForward.step, QImage::Format_BGR888);
        imageBackward = QImage((uchar*) imgBackward.data, imgBackward.cols, imgBackward.rows, imgBackward.step, QImage::Format_BGR888);
        imageLeft = QImage((uchar*) imgLeft.data, imgLeft.cols, imgLeft.rows, imgLeft.step, QImage::Format_BGR888);
        imageRight = QImage((uchar*) imgRight.data, imgRight.cols, imgRight.rows, imgRight.step, QImage::Format_BGR888);

//        painter.drawImage(rect.topLeft().x()+rect.width()/2, rect.topLeft().y()+5, imageSTOP);
        painter.setOpacity(0.4);
        QRect opRect(rect.bottomLeft().x()+rect.width()*0.35, rect.bottomLeft().y()-(rect.height()*0.3), rect.width()*0.3, rect.height()*0.3/*rect.bottomLeft().x()+(rect.width()*0.2), rect.topLeft().y()*/);
        painter.drawRoundRect(opRect);
        QRect gestRect(rect.topRight().x()-rect.width(), rect.topRight().y(), rect.width()/4, rect.height()/4);
//        painter.drawRect(gestRect);
        QRect arrowRect(rect.topRight().x()-130, rect.topRight().y()+10, 120, 30);
//        painter.drawRect(arrowRect);

        painter.setOpacity(1);
        painter.setPen(penText);
        painter.setFont(QFont("Times", 16, QFont::Bold));
        switch(action)
        {
        case GESTO_STOP:
            painter.drawText(QPoint(arrowRect.topLeft().x(), arrowRect.topLeft().y()+arrowRect.height()/2), "STOP");

//            painter.drawImage(gestRect.topLeft().x()+rect.width()/2, gestRect.topLeft().y()+gestRect.height()/2, imageSTOP);
            break;
        case GESTO_FORWARD:
            painter.drawText(QPoint(arrowRect.topLeft().x(), arrowRect.topLeft().y()+arrowRect.height()/2), "FORWARD");
//            painter.drawImage(gestRect.topLeft().x()+rect.width()/2 - imageForward.width()/2, gestRect.topLeft().y()+gestRect.height()/2 - imageForward.height()/2, imageForward);
            break;
        case GESTO_BACKWARD:
            painter.drawText(QPoint(arrowRect.topLeft().x(), arrowRect.topLeft().y()+arrowRect.height()/2), "BACKWARD");
//            painter.drawImage(gestRect.topLeft().x()+rect.width()/2 - imageBackward.width()/2, gestRect.topLeft().y()+gestRect.height()/2 - imageBackward.height()/2, imageBackward);
            break;
        case GESTO_LEFT:
            painter.drawText(QPoint(arrowRect.topLeft().x(), arrowRect.topLeft().y()+arrowRect.height()/2), "LEFT");
//            painter.drawImage(gestRect.topLeft().x()+rect.width()/2 - imageLeft.width()/2, gestRect.topLeft().y()+gestRect.height()/2 - imageLeft.height()/2, imageLeft);
            break;
        case GESTO_RIGHT:
            painter.drawText(QPoint(arrowRect.topLeft().x(), arrowRect.topLeft().y()+arrowRect.height()/2), "RIGHT");
//            painter.drawImage(gestRect.topLeft().x()+rect.width()/2 - imageRight.width()/2, gestRect.topLeft().y()+gestRect.height()/2 - imageRight.height()/2, imageRight);
            break;
        }

        if(collision){
            QRect collisionTextRect(rect.topLeft().x()+rect.width()/4, rect.topLeft().y()+20, rect.width()/2, 40);
            painter.setPen(penTextCollision);
            painter.setFont(QFont("Times", 30, QFont::Bold));
            painter.drawText(QPoint(collisionTextRect.topLeft().x(), collisionTextRect.topLeft().y()+collisionTextRect.height()/2), "!WARNING! Collision!");
        }

        painter.setOpacity(0.4);

        int hw = opRect.topLeft().x() + opRect.width()/2;
        int hh = opRect.topLeft().y() + opRect.height()/2;

        painter.setPen(pero_irobot);
        painter.drawEllipse(QPoint(hw, hh), 25, 25);
        painter.drawLine(QPoint(hw, hh), QPoint(hw, hh-25));

        painter.setPen(pen);
        QRectF r(hw-40, hh-40, 80, 80);
        QRectF rr(hw-55, hh-55, 110, 110);
        QRectF rrr(hw-70, hh-70, 140, 140);

        for (int i = 0; i < paintLaserData.numberOfScans; i+=2) {
            if(paintLaserData.Data[i].scanDistance < 150) continue;
            double D = paintLaserData.Data[i].scanDistance/1000;
            double alpha = (360.0-paintLaserData.Data[i].scanAngle);
//            cout<<"Angle: "<<360-paintLaserData.Data[i].scanAngle<<endl;
//            if(alpha>PI) alpha -= 2*PI;
            short danger = 0;
            if(D <= 0.7){
                danger++;
                if(D < 0.5){
                    danger++;
                    if(D <= 0.3) danger++;
                }
                if(alpha < 45 || alpha > 315){
                    painter.drawArc(r, 16*60, 16*60);
                    if(danger>1) painter.drawArc(rr, 16*60, 16*60);
                    if(danger>2) painter.drawArc(rrr, 16*60, 16*60);
                }
                if(135 >= alpha && alpha >= 45){
                    painter.drawArc(r, 16*150, 16*60);
                    if(danger>1) painter.drawArc(rr, 16*150, 16*60);
                    if(danger>2) painter.drawArc(rrr, 16*150, 16*60);
                }
                if(225 >= alpha && alpha > 135){
                    painter.drawArc(r, 16*240, 16*60);
                    if(danger>1) painter.drawArc(rr, 16*240, 16*60);
                    if(danger>2) painter.drawArc(rrr, 16*240, 16*60);
                }
                if(315 >=alpha && alpha > 225){
                    painter.drawArc(r, 16*330, 16*60);
                    if(danger>1) painter.drawArc(rr, 16*330, 16*60);
                    if(danger>2) painter.drawArc(rrr, 16*330, 16*60);
                }
            }
        }

//        painter.drawArc(r, 16*60, 16*60);
//        painter.drawArc(r, 16*150, 16*60);
//        painter.drawArc(r, 16*240, 16*60);
//        painter.drawArc(r, 16*330, 16*60);

//        painter.drawArc(hw, hh, 50, 50, 1000, 1000);
//        painter.drawChord(hw, hh, 50, 50, 1000, 1000);

        painter.setOpacity(1);
    }

    if(updateLaserPicture==1 && showLidar==true)
    {
        /// ****************
        ///you can change pen or pen color here if you want
        /// ****************
//        painter.setPen(pero_lidar);
//        for(int k=0;k<paintLaserData.numberOfScans;k++)
//        {
//            if(paintLaserData.Data[k].scanDistance < 150) continue;

//            int dist=paintLaserData.Data[k].scanDistance/30;
//            int xp=rect2.width()-(rect2.width()/2 + dist*sin((360.0-paintLaserData.Data[k].scanAngle)*PI/180.0))+rect2.topLeft().x();
//            int yp=rect2.height()-(rect2.height()/2 + dist*cos((360.0-paintLaserData.Data[k].scanAngle)*PI/180.0))+rect2.topLeft().y();
//            if(rect2.contains(xp,yp)){
////                std::cout << "Contains" << std::endl;
//                painter.drawEllipse(QPoint(xp, yp),2,2);
//            }
//            //if(xp<rect2.width()+1 && xp>19 && yp<rect2.height()+1 && yp>121)
//        }

//        int hw = rect2.topLeft().x() + rect2.width()/2;
//        int hh = rect2.topLeft().y() + rect2.height()/2;
//        painter.setPen(pero_irobot);
//        painter.drawEllipse(QPoint(hw, hh), 10, 10);
//        painter.drawLine(QPoint(hw, hh), QPoint(hw, hh-10));

//        painter.draw
    }
    if(updateSkeletonPicture==1 && showSkeleton==true)
    {
//        updateSkeletonPicture = 0;

//        painter.setPen(Qt::red);
//        painter.setPen(pero_joints);
//        for(int i=0;i<75;i++)
//        {
//            int xp;
//            int yp;
////            std::cout << klby[0] << ": " << kostricka.joints[0].x << kostricka.joints[0].y << std::endl;
//            xp = rect3.width() - rect3.width() * kostricka.joints[i].x + rect3.topLeft().x();
//            yp = rect3.height() * kostricka.joints[i].y + rect3.topLeft().y();
//            if(rect3.contains(xp,yp)){
//                painter.drawEllipse(QPoint(xp, yp),2,2);

//            }
//        }

        detector.updateSkelet(kostricka);

        action = detector.detectGestures();
        switch(action)
        {
        case GESTO_STOP:
            sendRobotCommand(ROBOT_STOP,0);
            break;
        case GESTO_FORWARD:
            sendRobotCommand(ROBOT_VPRED,200);
            break;
        case GESTO_BACKWARD:
            sendRobotCommand(ROBOT_VZAD,-150);
            break;
        case GESTO_LEFT:
            sendRobotCommand(ROBOT_VLAVO,3.14159/5);
            break;
        case GESTO_RIGHT:
            sendRobotCommand(ROBOT_VPRAVO,-3.14159/5);
            break;
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

    imgStop = cv::imread("../gesta_imgs/stop_gesto.jpg");
    imgForward = cv::imread("../gesta_imgs/forward_gesto.jpg");
    imgBackward = cv::imread("../gesta_imgs/backward_gesto.jpg");
    imgLeft = cv::imread("../gesta_imgs/left_gesto.jpg");
    imgRight = cv::imread("../gesta_imgs/left_gesto.jpg");
    cv::flip(imgRight, imgRight, +1);

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
    }
    if(event->key() == Qt::Key_D){
        sendRobotCommand(ROBOT_VPRAVO,-3.14159/4);
    }
    if(event->key() == Qt::Key_W){
        sendRobotCommand(ROBOT_VPRED,300);
    }
    if(event->key() == Qt::Key_S){
        sendRobotCommand(ROBOT_VZAD,-250);
    }
    if(event->key() == Qt::Key_Q){
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
}



void MainWindow::paintThisLidar(LaserMeasurement &laserData)
{
    memcpy( &paintLaserData,&laserData,sizeof(LaserMeasurement));
    updateLaserPicture=1;
//    update();
}

//void MainWindow::on_pushButton_8_clicked()//forward
//{
//    CommandVector help;
//    help.command.commandType=1;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=0;
//    help.command.desiredDist=100;
//    struct timespec t;

//    // clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_10_clicked()//right
//{
//    CommandVector help;
//    help.command.commandType=2;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=-20;
//    help.command.desiredDist=0;
//    struct timespec t;

//    // clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_11_clicked()//back
//{
//    CommandVector help;
//    help.command.commandType=1;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=0;
//    help.command.desiredDist=-100;
//    struct timespec t;

//    //   clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_9_clicked()//left
//{
//    CommandVector help;
//    help.command.commandType=2;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=20;
//    help.command.desiredDist=0;
//    struct timespec t;

//    //   clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_12_clicked()
//{
////    zX=ui->lineEdit_4->text().toDouble();
////    zY=ui->lineEdit_5->text().toDouble();
//    toleranciaUhla=2;
//    naviguj=true;
//}

//void MainWindow::on_pushButton_13_clicked()
//{
////    ui->lineEdit_4->setText("stlacil sa gombik");
//}


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
//        for(int i=0;i<75;i+=3)
//        {
//            std::cout<<klby[i]<<" "<<bbbk.joints[i].x<<" "<<bbbk.joints[i].y<<" "<<bbbk.joints[i].z<<std::endl;
//        }
    }
    std::cout<<"koniec thread"<<std::endl;
}

//void MainWindow::on_checkBox_2_clicked(bool checked)
//{
//    showLidar=checked;
//}


//void MainWindow::on_checkBox_3_clicked(bool checked)
//{
//    showCamera=checked;
//}


//void MainWindow::on_checkBox_4_clicked(bool checked)
//{
//    showSkeleton=checked;
//}


//void MainWindow::on_checkBox_clicked(bool checked)
//{
//    applyDelay=checked;
//}

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



//void MainWindow::on_checkBox_lidar_clicked(bool checked)
//{
//    showLidar=checked;
//}


//void MainWindow::on_checkBox_skeleton_clicked(bool checked)
//{
//    showSkeleton = checked;
//}


//void MainWindow::on_checkBox_cam_clicked(bool checked)
//{
//    showCamera = checked;
//}


//void MainWindow::on_stop_button_clicked()
//{
//    sendRobotCommand(ROBOT_STOP);
//}


//void MainWindow::on_pushButton_forward_clicked()
//{
//    sendRobotCommand(ROBOT_VPRED, 350);
//}


//void MainWindow::on_pushButton_backward_clicked()
//{
//    sendRobotCommand(ROBOT_VZAD, -250);
//}


//void MainWindow::on_pushButton_right_clicked()
//{
//    sendRobotCommand(ROBOT_VPRAVO, -PI/4);
//}


//void MainWindow::on_pushButton_left_clicked()
//{
//    sendRobotCommand(ROBOT_VLAVO, PI/4);
//}


void MainWindow::on_pushButton_stop_clicked()
{
    sendRobotCommand(ROBOT_STOP);
}

