#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include<windows.h>
#endif
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"

#include "robot.h"
#include "p_controller_rotation.h"
#include "p_controller_movement.h"

struct Point {
    int x, y;

    bool operator==(const Point& p) const {
        return x == p.x && y == p.y;
    }
};

enum class Direction {
    UP,
    RIGHT,
    DOWN,
    LEFT,
//    UP_RIGHT,
//    DOWN_RIGHT,
//    DOWN_LEFT,
//    UP_LEFT
};

namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];


    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);




private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_10_clicked();
    void on_pushButton_11_clicked();
    void on_pushButton_12_clicked();

    void on_pushButton_clicked();
    void getNewFrame();

    void movementForward(int speedForward);
    void movementToRight();
    void movementToLeft();
    void movementBackwards();
    void robotSlowdown();
    void stopRobot();

    double calculateShortestRotation(double correctRotation);

    void robotMovement(TKobukiData robotdata);
    double getRightOrientation();
    double getDistanceToEnd();
    void calculateXY(TKobukiData robotdata);
    void initData(TKobukiData robotdata);

    void updateMap();

    Point findLastValidPoint(int map[240][240], Point pointStart, Point pointEnd);
    Point pointOfChange(int map[240][240], Point point);
    Point walkAlongWall(int map[240][240], Point point, int yDirection, int xDirection, int yControl, int xControl, int x_substraction, int y_substraction);
    bool isOnLine();

    void walkAlongWallLidar();

private:

    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     TKobukiData robotdata;
     int datacounter;
     QTimer *timer;


     double forwardSpeed;
     double rotationspeed;//omega/s

     double distanceLW;
     double distanceRW;
     short delta_leftWheel;
     short delta_rightWheel;
     unsigned short encLeftWheel;
     unsigned short encRightWheel;
     double x;
     double x_destination;
     double y;
     double y_destination;
     double d;
     double alfa;
     double tickToMeter;

     bool init;
     bool isCorrectRotation;
     bool isStop;
     bool isRobotMove;
     bool isRobotRotate;
     bool isConvertAngleRight;
     bool isConvertAngleLeft;

     double rightRotationAngle;
     double leftRotationAngle;

     double deadbandRotation;
     int xArray[10];
     int yArray[10];

     int pointReached;

     double distance;

     double gyroStart;
     double gyro;
     double gyroRad;
     double gyroRadMap;

    PControllerRotation controllerRotation;
    PControllerMovement controllerMove;

    bool starMovement;
    int map[240][240];
    int numberOfSqareInMap;
    string tempMap[120];

    bool manualNavigation;

    double lidar_0;
    double lidar_90;
    double lidar_270;
    double lidar_315;
    double lidar_315_270;
    double lidar_360_315;
    double lidar_225_270;
    double lidar_0_45;
    double lidar_45_90;
    double angle_lidar;
    double min_dist_lidar;

    bool isLineTracking;

    double slope;
    double y_intercept;
    Direction lastDirection = Direction::UP;

    bool isWallClose;
    bool isMovementBasedOnLidar;

    double x_final;
    double y_final;

    bool isFreeMovement;

    bool isObstacleInPath;

    Point tempPoint;
    double distanceForward;
    double rotationToRight;
    double rotationToLeft;
    double rotationGyroTempRight;
    double rotationGyroTempLeft;


public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo


};

#endif // MAINWINDOW_H
