#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <iostream>
#include <windows.h>
#include <cmath>
#include "p_controller_rotation.h"
#include "p_controller_movement.h"
#include <fstream>
#include <string>
using namespace std;
///Jozef Kosecky, Peter Dobias


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    controllerRotation(7.5, 0.01),
    controllerMove(7.5, 0.01, 500)
{
    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="192.168.1.11"; //192.168.1.11 127.0.0.1
//    ipaddress="127.0.0.1";
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;

    datacounter=0;

    init = true;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;

    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }

            isRobotRotate = false;
            if(!isRobotRotate && !isStop){

                updateMap();
            }

        }
    }
}

void MainWindow::updateMap(){
//    cout << "zaciatok UPDATE MAP---------------------" << endl;
    int grid_size = 5; // Grid size in pixels
    int grid_offset = 60; // Grid offset in pixels
    int dist=0;
    double angleRad = 0;
    double xr = 0;
    double yr = 0;

    int x_wall = 0;
    int y_wall = 0;
    int numberFromMap = 0;
    double minDist = 300;
    double angle = 0;

//    copyOfLaserData.numberOfScans
    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
       /* if(copyOfLaserData.Data[k].scanQuality != 0){
            continue;
        }*/



        dist=copyOfLaserData.Data[k].scanDistance/10;
        angle = 360-copyOfLaserData.Data[k].scanAngle;

//        cout << "k: " << k << " angle: " << angle <<  endl;

        if(dist < 15 || dist > 300 || (dist >= 64 && dist <= 70)){
            continue;
        }

//        if(dist < 15){
//            dist = 15;
//        }

//        if(speed >= 0 && isFreeMovement){
//            if((angle <= 90) && dist < minDist){
//                minDist = dist;
//            }
//        }

//        if(minDist < 40 && min_dist_lidar == 300){
//            starMovement = false;
//            min_dist_lidar = minDist;
//            stopRobot();


//            Point point = pointOfChange(map,Point{static_cast<int>(std::round(x)),static_cast<int>(std::round(y))});
//            x_destination = point.x;
//            y_destination = point.y;
//            cout << "min_dist_lidar: " << min_dist_lidar << endl;
//            cout << "Predchadzajuca pozicia x: " << x << " y: " << y << endl;
//            cout << "Blizko steny Nova pozicia na ktoru idem je x: " << x_destination << " y: " << y_destination << endl;
//            distance = getDistanceToEnd();

//            starMovement = true;
//        }

        if(k == copyOfLaserData.numberOfScans - 1){
            lidar_0 = dist;
            if(lidar_0 < 45){
                cout << "predna stena je blizko: " << dist << endl;
                isMovementBasedOnLidar = true;
            }
        }
        if(angle - 270 > 0 && angle - 270 <= 4){
            lidar_270 = dist;
        }
        if(angle - 315 > 0 && angle - 315 <= 4){
            lidar_315 = dist;
        }
        if(angle < 270 && angle >= 225){
            lidar_225_270 = dist;
        }
        if(angle < 315 && angle >= 270){
//            if(true){
//                lidar_315_270 = dist;
//                cout << "Odmerana vzdialenost dist lidar_315_270: " << lidar_315_270 << endl;
//            }
//            if(k == 0){
//                lidar_315_270 = dist;
//                cout << "Odmerana vzdialenost dist lidar_315_270: " << lidar_315_270 << endl;
//            }
//            if(dist < lidar_315_270){
//                lidar_315_270 = dist;
//                cout << "Odmerana vzdialenost  dist lidar_315_270: " << lidar_315_270 << endl;
//            }
//            cout << "Odmerana vzdialenost dist lidar_315_270: " << lidar_315_270 << endl;
            lidar_315_270 = dist;
        }
        if(angle > 315 && angle < 360){
            if(dist < 40){
                cout << "predna stena je blizko: " << dist << endl;
                isMovementBasedOnLidar = true;
            }
            lidar_360_315 = dist;
        }

        if(angle > 3 && angle <= 45){
            lidar_0_45 = dist;
        }
        if(angle > 45 && angle <= 90){
            lidar_45_90 = dist;
        }






        if(rotationspeed == 0){
            angleRad = (((360-copyOfLaserData.Data[k].scanAngle)*PI)/180.0);

            xr = x + dist*cos(gyroRadMap + angleRad);
            yr = y + dist*sin(gyroRadMap + angleRad);

            x_wall = xr/grid_size + grid_offset;
            y_wall = yr/grid_size + grid_offset;


            if((y_wall > 0 && y_wall < numberOfSqareInMap) && (x_wall > 0 && x_wall < numberOfSqareInMap)){
                numberFromMap = map[y_wall][x_wall];
                 map[y_wall][x_wall] = numberFromMap + 1;
            }
        }


    }


    if(tightSpace == false && (lidar_0_45 < 45 || lidar_45_90 < 45) && (lidar_360_315 < 45 || lidar_315_270 < 45)){
        isMovementBasedOnLidar = true;
        tightSpace = true;
    }
}

void MainWindow::walkAlongWallLidar(){
//    calculateXY(robotdata);
//    cout << "walkAlongWallLidar--------------" << endl;

    if((lidar_0_45 < 50 || lidar_45_90 < 50) && (lidar_360_315 < 50 || lidar_315_270 < 50)){
        tightSpace = true;
    }

    if(lidar_0 < 50 || isFrontClose){
        cout << "stavsledovania 1" << endl;
        forwardSpeed = 0;
        rotationspeed = 0.5;
        robot.setRotationSpeed(rotationspeed);
        isFrontClose = true;
    }
    else if(tightSpace){
        cout << "stavsledovania 2" << endl;
        forwardSpeed = 0;
        rotationspeed = 0.5;
        robot.setRotationSpeed(rotationspeed);
    }
    else if(lidar_360_315 < 30 || lidar_315_270 < 30){
        cout << "stavsledovania 5" << endl;
        forwardSpeed = 150;
        rotationspeed = 150;
        robot.setArcSpeed(forwardSpeed, rotationspeed);
    }
    else if(lidar_360_315 < 35 || lidar_315_270 < 35){
        cout << "stavsledovania 6" << endl;
        forwardSpeed = 150;
        rotationspeed = 200;
        robot.setArcSpeed(forwardSpeed, rotationspeed);
    }
    else if(lidar_315 < 35){
        cout << "stavsledovania 7" << endl;
//        cout << "pravy senzor malo" << endl;
        forwardSpeed = 150;
        rotationspeed = 200;
        robot.setArcSpeed(forwardSpeed, rotationspeed);

    }
    else if(lidar_315 > 60){
        cout << "stavsledovania 8" << endl;
//        cout << "pravy senzor vela" << endl;
        forwardSpeed = 150;
        rotationspeed = -200;
        robot.setArcSpeed(forwardSpeed, rotationspeed);

    }
    else if(lidar_0_45 < 40 || lidar_45_90 < 40){
        cout << "stavsledovania 4" << endl;
        forwardSpeed = 150;
        rotationspeed = -150;
        robot.setArcSpeed(forwardSpeed, rotationspeed);
    }
    else{
        cout << "stavsledovania 9" << endl;
//        cout << "idem rovno" << endl;
        if(forwardSpeed < 200){
            forwardSpeed += 1;
        }
        else{
          forwardSpeed = 200;
        }

        rotationspeed = 0;
        robot.setTranslationSpeed(forwardSpeed);
    }

    if(lidar_0 > 50 && lidar_360_315 > 50){
        isFrontClose = false;
    }
    if((lidar_360_315 > 25 || lidar_315_270 > 25) && tightSpace){
        tightSpace = false;
    }
}

/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    if(init){
        initData(robotdata);
    }

    if(starMovement){
        calculateXY(robotdata);

        if((x_final >= x - 2.5) && (x_final <= x + 2.5) &&
                (y_final >= y - 2.5) && (y_final <= y + 2.5)){
            cout << "Som tu 4" << endl;
            cout << "Som v ciely" << endl;
            stopRobot();
            isStop = true;
            starMovement = false;
        }

        if(isLineTracking == false){

            Point center = findLastValidPoint(map, {(int)x,(int)y}, {(int)x_final,(int)y_final});
            if(center.x >= x_final - 5 && center.x <= x_final + 5 &&
                    center.y >= y_final - 5 && center.y <= y_final + 5){
                cout << "Som tu 3" << endl;
                cout << "(" << x << ", " << y << ") lies on the line." << endl;
                x_destination = center.x;
                y_destination = center.y;
                cout << "Predchadzajuca pozicia x: " << x << " y: " << y << endl;
                cout << "Sledovanie ciary Nova pozicia na ktoru idem je x: " << x_destination << " y: " << y_destination << endl;
                distance = getDistanceToEnd();
                isLineTracking = true;
            }
        }

        if(isMovementBasedOnLidar){
            cout << "Som tu 1" << endl;
            walkAlongWallLidar();
            Point center = pointOfChange(map,Point{static_cast<int>(std::round(x)),static_cast<int>(std::round(y))});
//                        Point center = findLastValidPoint(map, {(int)x,(int)y}, {point.x,point.y});
            x_destination = center.x;
            y_destination = center.y;
            cout << "Predchadzajuca pozicia x: " << x << " y: " << y << endl;
            cout << "Nova pozicia na ktoru idem je x: " << x_destination << " y: " << y_destination << endl;
            distance = getDistanceToEnd();
            if(distance > 60 && lidar_0 > 60 && lidar_360_315 > 45 && lidar_315_270 > 30 && tightSpace == false){
                isMovementBasedOnLidar = false;
            }
        }
        else{
            cout << "Som tu 2" << endl;
            forwardSpeed = 0;
            rotationspeed = 0;
            robotMovement(robotdata);
        }


//        robotMovement(robotdata);
    }

    if(datacounter%5)
    {
        emit uiValuesChanged(x,y,robotdata.GyroAngle/100.0);
    }
    datacounter++;

    return 0;

}





void MainWindow::robotSlowdown(){
    forwardSpeed -= 5;
    movementForward(forwardSpeed);
    if(forwardSpeed < 0){
        cout << "zastavujem" << endl;
        stopRobot();
    }
}

void MainWindow::stopRobot(){
    movementForward(0);
    forwardSpeed = 0;
    controllerMove.UpdateOutputToZero();
    isRobotMove = false;
}

double MainWindow::calculateShortestRotation(double correctRotation){
    //get difference between two angles
    if(gyroRad < correctRotation){
        rightRotationAngle = (2*PI) - correctRotation + gyroRad;
        leftRotationAngle = correctRotation - gyroRad;

        if(isConvertAngleLeft){
            isConvertAngleLeft = false;
        }

        if(rightRotationAngle < leftRotationAngle && !isCorrectRotation){
            isConvertAngleRight = true;
        }
    }
    else{
        leftRotationAngle = (2*PI) - gyroRad + correctRotation;
        rightRotationAngle = gyroRad - correctRotation;

        if(isConvertAngleRight){
            isConvertAngleRight = false;
        }

        if(leftRotationAngle < rightRotationAngle && !isCorrectRotation){
            isConvertAngleLeft = true;
        }
    }

    //We work with 360 degrees, but when we want use shortest path and this path pass through 0 degrees, we must convert circle from <0,360> to <0,180> and <-180,0>
    if(isConvertAngleRight){
        correctRotation -= (2*PI);
        if(gyroRad > PI){
            gyroRad -= (2*PI);
        }
//        cout << "prepocet do prava: " << "correctRotation: " << correctRotation << "gyroRad: "<< gyroRad << endl;
    }

    if(isConvertAngleLeft){
        if(gyroRad > PI){
            gyroRad -= (2*PI);
        }
//        cout << "prepocet do lava: " << "correctRotation: " << correctRotation << "gyroRad: "<< gyroRad << endl;
    }

    return correctRotation;
}


void MainWindow::robotMovement(TKobukiData robotdata){

    cout << "-------------------- ROBOT MOVEMENT\n" << endl;


    double correctRotation = getRightOrientation();
    double distanceToEnd = getDistanceToEnd();

    cout << "correctRotation: " << correctRotation << "gyroRad: "<< gyroRad << endl;
    cout << "x_destination: " << x_destination << "y_destination: "<< y_destination << endl;

    correctRotation = calculateShortestRotation(correctRotation);

    // Update the control output based on the measured value
    double output = controllerRotation.Update(correctRotation, gyroRad);

    // Convert the control output to motor speed
    double rotationSpeed = max(-(3.14159/2), min((3.14159/2), output));

    cout << "rotationSpeed: " << rotationSpeed << endl;

    if(rightRotationAngle <= deadbandRotation || leftRotationAngle <= deadbandRotation){
        cout << "zelane otocenie" << endl;
        deadbandRotation = 0.2;
        isCorrectRotation = true;
        isConvertAngleRight = false;
        isConvertAngleLeft = false;
    }
    else{
        cout << "nezelane otocenie" << endl;
        deadbandRotation = 0.02;
        isCorrectRotation = false;
    }

    if(!isStop){
        if(!isCorrectRotation){
            if(isRobotMove){
                cout << "zastavujem pohyb pred rotaciou spomalenie" << endl;
                robotSlowdown();
            }

            if((rotationSpeed >= 0.0) && !isRobotMove){
                cout << "tocim dolava" << endl;
                robot.setRotationSpeed(rotationSpeed);
                isRobotRotate = true;
            }
            else if((rotationSpeed < 0.0) && !isRobotMove){
                cout << "tocim doprava" << endl;
                robot.setRotationSpeed(rotationSpeed);
                isRobotRotate = true;
            }
        }
        else{
            if((x_destination >= x - 2.5) && (x_destination <= x + 2.5) &&
                    (y_destination >= y - 2.5) && (y_destination <= y + 2.5)){
                cout << "zastavujem pohyb" << endl;
                stopRobot();


                if((x_final >= x - 2.5) && (x_final <= x + 2.5) &&
                        (y_final >= y - 2.5) && (y_final <= y + 2.5)){
                    isStop = true;
                    starMovement = false;
                }

                if(!manualNavigation){
                    if(pointReached < 2){
                        cout << "dosiahol som bod" << endl;
                        isLineTracking = false;
                        min_dist_lidar = 300;
                        isFreeMovement = false;
                        cout << "pointReached min_dist_lidar: " << min_dist_lidar << endl;
                        cout << "pointReached LIDAR_270: " << lidar_270 << endl;

                        if((x_final >= x - 2.5) && (x_final <= x + 2.5) &&
                                (y_final >= y - 2.5) && (y_final <= y + 2.5)){
                            isStop = true;
                            starMovement = false;
                        }

                        Point center = pointOfChange(map,Point{static_cast<int>(std::round(x)),static_cast<int>(std::round(y))});
//                        Point center = findLastValidPoint(map, {(int)x,(int)y}, {point.x,point.y});
                        x_destination = center.x;
                        y_destination = center.y;
                        cout << "Predchadzajuca pozicia x: " << x << " y: " << y << endl;
                        cout << "Nova pozicia na ktoru idem je x: " << x_destination << " y: " << y_destination << endl;
                        distance = getDistanceToEnd();


                    }
                    else{
                       isStop = true;
                    }
                }
                else{
                    starMovement = false;
                }

                isRobotRotate = false;
            }
            else{
                distance = getDistanceToEnd();
                isRobotRotate = false;
                double outputMove = controllerMove.Update(0, distanceToEnd);
                cout << "Zrychlujem" << endl;
                cout << "distance: " << distance << "distanceToEnd: "<< distanceToEnd << endl;

                if(distance < 30 || lidar_0 <= 30){
                    forwardSpeed = max((50), min((100), std::abs(outputMove)));
                }
                else if(distance < 75 || lidar_0 <= 75){
                    forwardSpeed = max((50), min((250), std::abs(outputMove)));
                }else{
                    forwardSpeed = max((50), min((350), std::abs(outputMove)));
                }

                cout << "speed: " << forwardSpeed << endl;
                movementForward(forwardSpeed);
                isRobotMove = true;
            }
        }
    }
    cout << "--------------------\n" << endl;
}

double MainWindow::getDistanceToEnd(){
    double distance = std::sqrt(std::pow(x_destination - x, 2) + std::pow(y_destination - y, 2));
    return distance;
}

double MainWindow::getRightOrientation(){
    double deltaX = x_destination - x; // change in x-coordinate
    double deltaY = y_destination - y; // change in y-coordinate
    double angle = atan2(deltaY, deltaX) * 180 / PI; // angle in degrees
    if(angle < 0){
        angle += 360;
    }

    int rounded = std::round(angle);

    if(rounded == 360){
        angle = 0;
    }

    cout << "correctRotation: " << angle << "gyro_angle: "<< gyro << endl;
    return (((angle)*PI)/180.0);
}


void MainWindow::calculateXY(TKobukiData robotdata){
    // pretecenie enkoder
    delta_leftWheel = robotdata.EncoderLeft - encLeftWheel;
    delta_rightWheel = robotdata.EncoderRight - encRightWheel;

    distanceLW = tickToMeter*(delta_leftWheel);
    distanceRW = tickToMeter*(delta_rightWheel);

//    cout << "robotdata.GyroAngle/100: " << robotdata.GyroAngle/100 << " gyroStart: "<< gyroStart << endl;

    gyro = robotdata.GyroAngle/100 - gyroStart;
    double delta_distance  = (distanceLW + distanceRW) / 2.0;
    gyroRad = (((gyro)*PI)/180.0);
    gyroRadMap = gyroRad;

//    cout << "gyro: " << gyroRad << endl;

    if(gyro < 0){
        gyro += 360;
    }
    // pretecenie gyro
    if(gyroRad < 0){
        gyroRad += 2*PI;
        gyroRadMap = gyroRad;
    }

    if(forwardSpeed != 0 && rotationspeed != 0){
        if((distanceRW - distanceLW) != 0){
            x = x + ((d*(distanceRW + distanceLW)) / (2*(distanceRW - distanceLW))) * (sin(gyroRad) - sin(alfa)) * 100;
            y = y - ((d*(distanceRW + distanceLW)) / (2*(distanceRW - distanceLW))) * (cos(gyroRad) - cos(alfa)) * 100;
        }
    }
    else{
        x = x + (delta_distance  * cos(gyroRad))*100;
        y = y + (delta_distance  * sin(gyroRad))*100;
    }



//    cout << "gyro: " << gyroRad << endl;
//    cout << "gyroRad: " << gyroRad << endl;
//    cout << "robotdata.GyroAngle: " << robotdata.GyroAngle/100 << endl;

//    cout << "Priebezna pozicia x: " << x << " y: " << y << endl;

    alfa = gyroRad;
    encLeftWheel = robotdata.EncoderLeft;
    encRightWheel = robotdata.EncoderRight;

}

bool MainWindow::isOnLine(){

    //priamka medzi bodmi
    slope = (y_final - y) / (x_final - x);
    y_intercept = y - slope * x;

    double y_test = slope * x + y_intercept;

    cout << "hodnota testu je: " << abs(y_test - y) << endl;
    return abs(y_test - y) < 2;
}

Point MainWindow::findLastValidPoint(int map[240][240], Point pointStart, Point pointEnd) {
    int grid_size = 5; // Grid size in pixels
    int grid_offset = 60; // Grid offset in pixels
    int wall_distance = 11; // Wall distance in cm/5. So 50cm/5 = 10

    pointStart.x = pointStart.x/grid_size + grid_offset;
    pointStart.y = pointStart.y/grid_size + grid_offset;

    pointEnd.x = pointEnd.x/grid_size + grid_offset;
    pointEnd.y = pointEnd.y/grid_size + grid_offset;

//    cout << "pointStart x: " << pointStart.x << " y: " << pointStart.y << endl;
//    cout << "pointEnd x: " << pointEnd.x << " y: " << pointEnd.y << endl;

    // Use Bresenham's line algorithm to iterate through cells along the line
    int dx = abs(pointEnd.x - pointStart.x);
    int dy = abs(pointEnd.y - pointStart.y);
    int sx = pointStart.x < pointEnd.x ? 1 : -1;
    int sy = pointStart.y < pointEnd.y ? 1 : -1;
    int x = pointStart.x;
    int y = pointStart.y;
    int err = dx - dy;

    bool isObstacle = false;
    int x_test = 0;
    int y_test = 0;


    while (true) {
//        cout << "Mapa x: " << x << " y: " << y << " hodnota: " << map[y][x] << endl;
        // Check if the current cell is a wall

        int radius = 5;   // Radius in cm
        // Convert radius from cm to array index
        int radiusIndex = radius;  // Assuming each array index is 1 cm
        int startX = max(x - radiusIndex, 0);
        int endX = min(x + radiusIndex, 239);
        int startY = max(y - radiusIndex, 0);
        int endY = min(y + radiusIndex, 239);

        // Loop through the elements in the 5x5 area
        for (x_test = startX; x_test <= endX; x_test++) {
            if(x_test > 240){
                break;
            }
            for (y_test = startY; y_test <= endY; y_test++) {
                if(y_test > 240){
                    break;
                }
                // Check if the current element has the value 1
                if (map[y_test][x_test] > validMapPoint + 3) {
                    // Found a 1 in the 2 cm radius
//                    cout << "Nasiel som prekazku a 1 at x: " << x << ", y: " << y << endl;
                    isObstacle = true;
                    break;
                }
            }
            if(isObstacle){
                break;
            }
        }

        if(isObstacle && pointEnd.x != ((int)x_final/grid_size + grid_offset) && pointEnd.y != ((int)y_final/grid_size + grid_offset)){
            isObstacleInPath = true;
            cout << "Nasiel som prekazku a 1 at pointEnd.x: " << pointEnd.x << ", x: " << (int)x_final << " pointEnd.y: " << pointEnd.y << ", y: " << (int)y_final << endl;
            double angle = atan2(pointEnd.y - pointStart.y, pointEnd.x - pointStart.x);
            x_test = (x_test - cos(angle) * wall_distance) * grid_size - grid_offset * grid_size;
            y_test = (y_test - sin(angle) * wall_distance) * grid_size - grid_offset * grid_size;
            return Point{x_test, y_test};
        }

        if (map[y][x] > validMapPoint) {
           // If it is a wall, return the point at the specified distance from the wall
           double angle = atan2(pointEnd.y - pointStart.y, pointEnd.x - pointStart.x);
           x = (x - cos(angle) * wall_distance) * grid_size - grid_offset * grid_size;
           y = (y - sin(angle) * wall_distance) * grid_size - grid_offset * grid_size;
           return Point{x, y};
       }


       // Check if the current cell is the destination point
       if (Point{x, y} == pointEnd) {
           if(isObstacle){
               double angle = atan2(pointEnd.y - pointStart.y, pointEnd.x - pointStart.x);
               x_test = (x_test - cos(angle) * wall_distance) * grid_size - grid_offset * grid_size;
               y_test = (y_test - sin(angle) * wall_distance) * grid_size - grid_offset * grid_size;
               return Point{x_test, y_test};
           }

           // If it is the destination point, return it as the last valid point
           Point finalPoint = pointEnd;
           x = (finalPoint.x - grid_offset) * grid_size;
           y = (finalPoint.y - grid_offset) * grid_size;

//           cout << "pointEnd" << endl;
           return Point{x, y};
       }

        // Update the error and move to the next cell
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;

//            cout << "new x: " << x << endl;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
//            cout << "new y: " << y << endl;
        }
    }
}

Point MainWindow::walkAlongWall(int map[240][240], Point point, int yDirection, int xDirection, int yControl, int xControl,int x_substraction, int y_substraction) {
    int grid_size = 5; // Grid size in pixels
    int grid_offset = 60; // Grid offset in pixels

    int x = point.x/grid_size + grid_offset;
    int y = point.y/grid_size + grid_offset;

    while (true) {
//        cout << "x: " << x << " y: " << y << endl;
        if (map[y + yDirection][x + xDirection] == 0 && map[y + yControl][x + xControl] > validMapPoint) {
//            cout << "x: " << x << " y: " << y << endl;
//            cout << "map[y + yDirection][x + xDirection]: " << map[y + yDirection][x + xDirection] << " map[y + yControl][x + xControl]: " << map[y + yControl][x + xControl] << endl;
            y += yDirection;
            x += xDirection;
        }
        else if(map[y + yDirection][x + xDirection] == 0 && map[y + yControl][x + xControl] == 0){
            cout << "Potiahnutie" << endl;
            cout << "x_substraction: " << x_substraction << " y_substraction: " << y_substraction << endl;
            cout << "x: " << x << " y: " << y << endl;
//            cout << "map[y + yDirection][x + xDirection]: " << map[y + yDirection][x + xDirection] << " map[y + yControl][x + xControl]: " << map[y + yControl][x + xControl] << endl;
            y += yDirection;
            x += xDirection;
//            cout << "map[y + yDirection][x + xDirection]: " << map[y + yDirection][x + xDirection] << " map[y + yControl][x + xControl]: " << map[y + yControl][x + xControl] << endl;

            if(y_substraction < 0){
                y_substraction += 2;
            }
            else if(y_substraction > 0){
                y_substraction -= 2;
            }
//            if(abs(xControl) > 8){
//                int temp = abs(xControl) - 8;
//                if (xControl < 0) {
//                   x -= temp;
//                } else {
//                    x += temp;
//                }

//            }
//            if(abs(yControl) > 8){
//                int temp = abs(yControl) - 8;
//                if (yControl < 0) {
//                   y -= temp;
//                } else {
//                    y += temp;
//                }
//            }

            x = x - (x_substraction / 2);
            y = y - y_substraction;
            break;
        }
        else {
            cout << "koniec" << endl;
//            cout << "map[y + yDirection][x + xDirection]: " << map[y + yDirection][x + xDirection] << " map[y + yControl][x + xControl]: " << map[y + yControl][x + xControl] << endl;
            y += yDirection;
            x += xDirection;
//            cout << "map[y + yDirection][x + xDirection]: " << map[y + yDirection][x + xDirection] << " map[y + yControl][x + xControl]: " << map[y + yControl][x + xControl] << endl;

            cout << "xControl: " << xControl << " yControl: " << yControl << endl;




            cout << "x: " << x << " y: " << y << endl;
            x = x + x_substraction;
            y = y + y_substraction;
            break;
        }
    }
    cout << "walkAlongWall x: " << x << " y: " << y << endl;
    return Point{x, y};
}

Point MainWindow::pointOfChange(int map[240][240], Point point) {
//    cout << "Nova pozicia na ktoru idem je Volanie funkcie " << endl;
    int grid_size = 5; // Grid size in pixels
    int grid_offset = 60; // Grid offset in pixels
    int wall_distance = 8; // Wall distance in cm/5. So 50cm/5 = 10

    int x = point.x/grid_size + grid_offset;
    int y = point.y/grid_size + grid_offset;
    bool isBreak = false;

    cout << "pointOfChange: " << endl;
    cout << "pointStart x: " << x << " y: " << y << endl;

    int sx = 1;
    int sy = 1;

    Point newPoint = Point{x,y};

    int distanceToWallMin = 3;
    int distanceToWallMax = 7;



    for (auto direction : {Direction::RIGHT, Direction::LEFT}) {
        switch (direction) {
            case Direction::LEFT: {
                // LEFT and UP
                cout << "LEFT" << endl;
//                if(lastDirection == Direction::LEFT){
//                    break;
//                }
                sx = 1;
                while(x - sx >= 0){
                     if (map[y][x - sx] > validMapPoint){
                        break;
                     }
                     sx++;
                }
                cout << "sx: " << sx <<endl;
                if(sx <= 12){
                    break;
                }

                sy= 1;
                while(y + sy <= 240){
                    if (map[y][x - 1] == 0 && map[y + sy][x - 1] > validMapPoint) {
                        cout << "Nova pozicia na ktoru idem je Direction::LEFT " << endl;
                        cout << "x: " << x << " y: " << y << endl;
                        newPoint = walkAlongWall(map, point, 0, -1, sy, -1, wall_distance, 0);
                        //blizko steny
                        if(sy < distanceToWallMin){
                            newPoint.y = newPoint.y - (distanceToWallMin - sy);
                        }
                        //daleko steny
                        if(sy > distanceToWallMax){
                            newPoint.y = newPoint.y + (sy - distanceToWallMax);
                        }


                        isBreak = true;
                        lastDirection = Direction::LEFT;
                        break;
                    }
                    sy++;

                    if(sy == 12){
                        break;
                    }
                }
                cout << "sy: " << sy <<endl;
                break;
            }
            case Direction::RIGHT: {
                // RIGHT and DOWN
                cout << "RIGHT" << endl;
//                if(lastDirection == Direction::RIGHT){
//                    break;
//                }

                sx = 1;
                while(x + sx <= 240){
                     if (map[y][x + sx] > validMapPoint){
                        break;
                     }
                     sx++;
                }
                cout << "sx: " << sx <<endl;
                if(sx <= 12){
                    break;
                }

                sy= 1;
                while(y - sy >= 0){
                    if (map[y][x + 1] == 0 && map[y - sy][x + 1] > validMapPoint) {
                        double y_test = slope * x + y_intercept;

                        if (abs(y_test - y) < 1e-9) {
                            cout << "(" << x << ", " << y << ") lies on the line." << endl;
                        }
                        cout << "Nova pozicia na ktoru idem je Direction::RIGHT " << endl;
                        cout << "x: " << x << " y: " << y << endl;
                        newPoint = walkAlongWall(map, point, 0, 1, -sy, 1, -wall_distance, 0);

                        if(abs(sy) < distanceToWallMin){
                            newPoint.y = newPoint.y + (distanceToWallMin - abs(sy));
                        }

                        if(abs(sy) > distanceToWallMax){
                            newPoint.y = newPoint.y - (abs(sy) - distanceToWallMax);
                        }

                        isBreak = true;
                        lastDirection = Direction::RIGHT;
                        break;
                    }
                    sy += 1;

                    //neprehladavam priestor dalej ako 50cm
                    if(sy == 12){
                        break;
                    }
                }
                cout << "sy: " << sy <<endl;
                break;
            }
        }
        if(isBreak == true){
            newPoint.x = (newPoint.x - grid_offset) * grid_size;
            newPoint.y = (newPoint.y - grid_offset) * grid_size;
            return newPoint;
        }
    }

    int offset = 16;
    if(lidar_270 >= 85 && lidar_270 <= 300 && newPoint.x == x && newPoint.y == y){
        cout << "pointOfChange LIDAR_270: " << lidar_270 << endl;
        switch (lastDirection) {
            case Direction::UP: {
                cout << "Nova pozicia na ktoru idem je Direction::RIGHT FAKE LIDAR_270" << endl;
                newPoint.x += offset;
                lastDirection = Direction::RIGHT;
                break;
            }
            case Direction::DOWN: {
                cout << "Nova pozicia na ktoru idem je Direction::LEFT FAKE LIDAR_270" << endl;
                newPoint.x -= offset;
                lastDirection = Direction::LEFT;
                break;
            }
            case Direction::RIGHT: {
                cout << "Nova pozicia na ktoru idem je Direction::DOWN FAKE LIDAR_270" << endl;
                newPoint.y -= offset;
                lastDirection = Direction::DOWN;
                break;
            }
            case Direction::LEFT: {
                cout << "Nova pozicia na ktoru idem je Direction::UP FAKE LIDAR_270" << endl;
                newPoint.y += offset;
                lastDirection = Direction::UP;
                break;
            }
        }
        isFreeMovement = true;
        newPoint.x = (newPoint.x - grid_offset) * grid_size;
        newPoint.y = (newPoint.y - grid_offset) * grid_size;
        return newPoint;
    }

    for (auto direction : {Direction::DOWN, Direction::UP,}) {
        switch (direction) {
            case Direction::UP: {
                // UP and RIGHT
                cout << "UP" << endl;
//                if(lastDirection == Direction::UP){
//                    break;
//                }

                //kontrola ci stena v smere hore nie je blizko a neoplati sa robit takyto pohyb
                sy = 1;
                while(y + sy <= 240){
                     if (map[y + sy][x] > validMapPoint){
                        break;
                     }
                     sy++;
                }
                cout << "sy: " << sy <<endl;
                if(sy <= 12){
                    break;
                }

                sx= 1;
                while(x + sx <= 240){
                    if (map[y + 1][x] == 0 && map[y + 1][x + sx] > validMapPoint) {
                        cout << "Nova pozicia na ktoru idem je Direction::UP " << endl;
                        cout << "x: " << x << " y: " << y << endl;
                        newPoint = walkAlongWall(map, point, 1, 0, 1, sx, 0, -wall_distance);
                        if(sx < distanceToWallMin){
                            newPoint.x = newPoint.x - (distanceToWallMin - sx);
                        }
                        if(abs(sx) > distanceToWallMax){
                            newPoint.x = newPoint.x + (abs(sx) - distanceToWallMax);
                        }
                        isBreak = true;
                        lastDirection = Direction::UP;
                        break;
                    }
                    sx++;

                    if(sx == 12){
                        break;
                    }
                }
                cout << "sx: " << sx <<endl;
                break;
            }
            case Direction::DOWN: {
                // DOWN and LEFT
                cout << "DOWN" << endl;
                if(lastDirection == Direction::UP){
                    break;
                }

                sy = 1;
                while(y - sy >= 0){
                     if (map[y - sy][x] > validMapPoint){
                        break;
                     }
                     sy++;
                }
                cout << "sy: " << sy <<endl;
                if(sy <= 12){
                    break;
                }

                sx= 1;
                while(x - sx >= 0){
                    if (map[y - 1][x] == 0 && map[y - 1][x - sx] > validMapPoint) {
                        cout << "Nova pozicia na ktoru idem je Direction::DOWN " << endl;
                        cout << "x: " << x << " y: " << y << endl;
                        newPoint = walkAlongWall(map, point, -1, 0, -1, -sx, 0, wall_distance);
                        if(abs(sx) < distanceToWallMin){
                            newPoint.x = newPoint.x + (distanceToWallMin - abs(sx));
                        }
                        if(abs(sx) > distanceToWallMax){
                            newPoint.x = newPoint.x - (abs(sx) - distanceToWallMax);
                        }

                        isBreak = true;
                        lastDirection = Direction::DOWN;
                        break;
                    }
                    sx++;

                    if(sx == 12){
                        break;
                    }
                }
                cout << "sx: " << sx <<endl;
                break;
            }
        }
        if(isBreak == true){
            newPoint.x = (newPoint.x - grid_offset) * grid_size;
            newPoint.y = (newPoint.y - grid_offset) * grid_size;
            return newPoint;
        }
    }

    int move = 7;
    if(newPoint.x == x && newPoint.y == y){
        switch (lastDirection) {
            case Direction::UP: {
                cout << "Nova pozicia na ktoru idem je Direction::RIGHT FAKE" << endl;
                newPoint.x += move;
                lastDirection = Direction::RIGHT;
                break;
            }
            case Direction::DOWN: {
                cout << "Nova pozicia na ktoru idem je Direction::LEFT FAKE" << endl;
                newPoint.x -= move;
                lastDirection = Direction::LEFT;
                break;
            }
            case Direction::RIGHT: {
                cout << "Nova pozicia na ktoru idem je Direction::DOWN FAKE" << endl;
                newPoint.y -= move;
                lastDirection = Direction::DOWN;
                break;
            }
            case Direction::LEFT: {
                cout << "Nova pozicia na ktoru idem je Direction::UP FAKE" << endl;
                newPoint.y += move;
                lastDirection = Direction::UP;
                break;
            }
        }
    }
    isFreeMovement = true;
    newPoint.x = (newPoint.x - grid_offset) * grid_size;
    newPoint.y = (newPoint.y - grid_offset) * grid_size;
    return newPoint;
}


void MainWindow::initData(TKobukiData robotdata){
    encLeftWheel = robotdata.EncoderLeft;
    encRightWheel = robotdata.EncoderRight;

    distanceLW = 0;
    distanceRW = 0;
    delta_leftWheel = 0;
    delta_rightWheel = 0;
    x = 0;
    y = 0;
    tickToMeter = 0.000085292090497737556558;
    d = 0.23; //m
    alfa = 0;

    forwardSpeed = 0;

    //stvorec
//    xArray[0] = 0;
//    xArray[1] = 200;
//    xArray[2] = 0;
//    xArray[3] = 0;
//    xArray[4] = 150;

//    yArray[0] = 300;
//    yArray[1] = 325;
//    yArray[2] = -1;
//    yArray[3] = 0;
//    yArray[4] = 350;

    //trojuholnik
//    xArray[0] = 0;
//    xArray[1] = 0;
//    xArray[2] = 0;
//    xArray[3] = 0;
//    xArray[4] = 150;

//    yArray[0] = -20;
//    yArray[1] = 250;
//    yArray[2] = 0;
//    yArray[3] = 0;
//    yArray[4] = 350;

    //Prehladavanie mapy
//    xArray[0] = 0;
//    xArray[1] = 300;
//    xArray[2] = 110;
//    xArray[3] = 270;
//    xArray[4] = 270;
//    xArray[5] = 400;
//    xArray[6] = 270;
//    xArray[7] = 270;
//    xArray[8] = 475;
//    xArray[9] = 475;

//    yArray[0] = 0;
//    yArray[1] = 0;
//    yArray[2] = 155;
//    yArray[3] = 155;
//    yArray[4] = 360;
//    yArray[5] = 360;
//    yArray[6] = 360;
//    yArray[7] = 30;
//    yArray[8] = 30;
//    yArray[9] = 160;

    pointReached = 0;

    x_destination = 0;
    y_destination = 0;

//    x_final = 350;
//    y_final = 0;
    x_final = 420;
    y_final = 180;
//    x_final = 200;
//    y_final = 320;

//    x_final = 280;
//    y_final = 200;

    distance = getDistanceToEnd();
    deadbandRotation = 0.02;

    gyroStart = robotdata.GyroAngle/100;
    gyro = 0;
    gyroRad = 0;
    gyroRadMap = 0;

    isConvertAngleRight = false;
    isConvertAngleLeft = false;
    isCorrectRotation = false;
    isStop = false;
    isRobotMove = false;
    isRobotRotate = false;

    numberOfSqareInMap = 240;

    cout << "nepadol som" << endl;

    //MAP
        for (int i = 0; i < numberOfSqareInMap; i++) {
            for (int j = 0; j < numberOfSqareInMap; j++) {
                map[i][j] = 0;
            }
        }
    cout << "nepadol som2" << endl;

    //    for (int i = 0; i < 120; i++) {
    //        for (int j = 0; j < 120; j++) {
    //            std::cout << map[i][j] << " ";
    //        }
    //        std::cout << std::endl;
    //    }

    starMovement = false;
    manualNavigation = false;
    isLineTracking = false;

    angle_lidar = 0;
    min_dist_lidar = 300;
    isMovementBasedOnLidar = false;

    tightSpace = false;
    validMapPoint = 0;
    init = false;
    isFrontClose = false;
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}


void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardSpeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();

//    this_thread::sleep_for(chrono::milliseconds(500));

}

void MainWindow::movementForward(int speedForward){
    robot.setTranslationSpeed(speedForward);
}

void MainWindow::movementBackwards(){

}

void MainWindow::movementToRight(){

}

void MainWindow::movementToLeft(){

}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(500);
}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
    isRobotRotate = true;
robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
    isRobotRotate = true;
robot.setRotationSpeed(-3.14159/2);
}

void MainWindow::on_pushButton_10_clicked()//start automatic
{
    if(isStop){
        isStop = false;
        starMovement = true;
    }
    else{
        Point center = findLastValidPoint(map, {0,0}, {(int)x_final,(int)y_final});
        x_destination = center.x;
        y_destination = center.y;

        cout << "Sledovanie ciary Nova pozicia na ktoru idem je x: " << x_destination << " y: " << y_destination << endl;

        //priamka medzi bodmi
        slope = (y_destination - 0) / (x_destination - 0);
        y_intercept = 0 - slope * 0;

        starMovement = true;
        manualNavigation = false;
        isLineTracking = true;
    }

}

void MainWindow::on_pushButton_11_clicked()//SAVE MAP
{
    cout << "nova mapa" << endl;

    std::ofstream outfile("C:\\Users\\jkose\\Documents\\vysoka skola\\4.rocnik\\2.semester\\RMR\\mapa7.txt");

//    // iterate through the rows of the array and swap them
//    for (int i = 0; i < numberOfSqareInMap-1; i++) {
//        for (int j = 0; j < numberOfSqareInMap; j++) {
//            tempMap[j] = map[i][j];
//            map[i][j] = map[numberOfSqareInMap-1-i][j];
//            map[numberOfSqareInMap-1-i][j] = tempMap[j];
//        }
//    }



    // Write the array contents to the file
    for (int i = numberOfSqareInMap-1; i >= 0; i--) {
        for (int j = 0; j < numberOfSqareInMap; j++) {
            if(map[i][j] >= 0 && map[i][j] <= validMapPoint){
//                outfile << ' ' << " ";
                outfile << " ";
//                outfile << i << j;
            }
            else{
//                outfile << '*' << " ";
                outfile << 1;
//                outfile << map[i][j];
//                           outfile << i << j;
            }
            cout << map[i][j];
        }

        cout << endl;
        outfile << std::endl;
    }

    // Close the output file
    outfile.close();

    cout<<"zapisal som"<<endl;
}

void MainWindow::on_pushButton_12_clicked() //SEND ROBOT
{

    //pohyb dopredu
    pointReached = 0;
    QString x = ui->lineEdit_6->text();
    QString y = ui->lineEdit_5->text();
    x_final = x.toInt();
    y_final = y.toInt();
    isStop = false;
    starMovement = true;
    manualNavigation = true;
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    forwardSpeed = 0;
    rotationspeed = 0;
    robot.setTranslationSpeed(0);
    isStop = true;
    isRobotRotate = false;
    starMovement = false;

    cout << "Odmerana vzdialenost  dist lidar_315_270: " << lidar_315_270 << " dist lidar_360_315: " << lidar_360_315 << endl;
}



void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}
