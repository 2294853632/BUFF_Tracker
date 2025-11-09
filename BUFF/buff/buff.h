#ifndef BUFF_H
#define BUFF_H
#pragma once

#include"configer.h"
#include<cmath>
#include"vector"
#include"map"
#include"random"
#include"circlefitting.h"
#include"kalman.h"



class buff
{
public:
    cv::Point2f point_find;
    bool buff_flag;
    float angle;
    float c;

    float R=126.39;
    std::vector<cv::Point>track;//用于拟合圆的点集
    cv::Point2f Last_center,Last_point_find;//上一帧的数据，丢失目标后发送上一帧数据
//    kalman KF;
public:
    Rect find_R(cv::Mat src,bool color,Point2f& R_center,Ptr<KNearest> model_R);
    void findSquares(const cv::Mat& image,bool color,cv::Point2f circle_center);//红1蓝0
private:
    void draw(cv::Mat dst, std::vector<std::vector<cv::Point>> &aim);
    void draw(cv::Mat dst,cv::RotatedRect aim);

    kalman a;

};

class RotationRectangle
{
friend class BUFFtracker;
public:
    friend class BBox;
  void _init(Point2f Points[4],Point2f Rbox_center);
    Point2f _getlinecenter(Point2f p1,Point2f p2);
    float Width();
    float Height();
    float Area();
    Point2f Center();
    Point2i Center_i();
private:
    Point2f p1,p2,p3,p4;
    Point2f points[4];
    float k13,k24,b13,b24;
    Point2f center12,center34;
    Point2f top,botm;
    float topdis,botmdis;
    float width;
    float height;
    float area;
    Point2f center;
    Point2i center_i;
};



class BBox
{
friend float IOU(BBox a,BBox b);
friend float CIOU(BBox a,BBox b);
friend float GIOU(BBox a,BBox b);
friend float DIOU(BBox a,BBox b);
friend void run();
public:
 friend class  RotationRectangle;
  friend class BUFFtracker;
  void  initbox(float xmin,float ymin,float xmax,float ymax,int BBox_ID);
  float centerdis(BBox other);
  BBox boundof(BBox other);
  float bound_duijiao_dis(BBox other);
  Point2f Center();
  Point2i Center_i();
  float Width();
  float Height();
  float Area();
  void setID(int BBox_id);
  BBox create_new_bbox_center(Point2f center);
private:
    int id;
    float xmin,xmax,ymin,ymax;
Point2f minpt,maxpt;//minpt为左上角点，maxpt为右下角点
    float width;
    float height;
    State state;
    Point2f center;
    Point2i center_i;
    float area;

};






class fanBlade
{
  friend  class BUFFtracker;
public:
void initfan(BBox rect,RotationRectangle rtn_rect);
private:
    BBox bbox;
    RotationRectangle rtnrect;
    State state;

};




float enclidean_dis(Point2f p1,Point2f p2);

Point2f Rotation(float theta, Point2f center);

class Targetstruct
{
   friend bool cmp(Targetstruct a,Targetstruct b);
   friend class BUFFtracker;
   friend vector<Targetstruct> compareByIOU(BBox Rbox,vector<BBox>boxs,IoUType type);
public:
  void _inittarget(BBox box,float iou);
private:
  BBox box;
  float iou;
};






class BUFFtracker
{
    friend void run();
public:
    void initbuff(BBox BladeBox,BBox R_box,bool isimshow);
    bool update(Mat src,bool isopenMaybetarget);
    Mat HSVThreshold(Mat src);
    void getthreshold(MayBeTarget &MT);
    bool _MayTargert(float w,float h,bool flag);
    vector<fanBlade>getBlade(Mat mask);
    BBox Points2BBox(Point2f points[4]);
private:
    BBox BladeBOX;
    BBox R_box;
   vector<fanBlade>BladeList;
        float radius;
        State states[5];
        int fanNUM;//上一帧亮起的扇叶数
        bool isImshow;
        int count=0;
        Point2f center;
        Mat frame;
        int color;//1为红,0为蓝
        Scalar lowhsv;
        Scalar uphsv;
        Mat kernel;
        bool buffsize;//1为BIG，0为Small
        float outR;
        float inR;
        int Start;
        MayBeTarget MT;
};




#endif // BUFF_H
