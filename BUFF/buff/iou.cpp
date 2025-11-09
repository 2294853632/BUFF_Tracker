#include"iou.h"


float IOU(BBox a,BBox b)
{
    double x1=max(a.xmin,b.xmin);
    double y1=max(a.ymin,b.ymin);
    double x2=min(a.xmax,b.xmax);
    double y2=min(a.ymax,b.ymax);
    float intersectionWidth=max(0.0,x2-x1);
    float intersectionHeight=max(0.0,y2-y1);
    float intersectionArea=intersectionWidth*intersectionHeight;
float unionArea=a.area+b.area-intersectionArea;
float iou=intersectionArea/unionArea;
return iou;
}

float GIOU(BBox a,BBox b)
{
float boundarea=a.boundof(b).area;
double x1 = std::max(a.xmin, b.xmin);
double y1 = std::max(a.ymin, b.ymin);
double x2 = std::min(a.xmax, b.xmax);
double y2 = std::min(a.ymax, b.ymax);
float xoverlap=max(0.0,x2-x1);
float yoverlap=max(0.0,y2-y1);
float overarea=xoverlap*yoverlap;
float unionarea=a.area+b.area-overarea;
float giou= IOU(a,b)-(boundarea-unionarea)/boundarea;
return giou;
}


float DIOU(BBox a,BBox b)
{
float d=a.centerdis(b);
float c=a.bound_duijiao_dis(b);
float diou=IOU(a,b)-pow(d,2)/pow(c,2);
return diou;
}


float CIOU(BBox a,BBox b)
{
    float xC = std::max(a.xmin+a.width/2, b.xmin+b.width/ 2);
    float yC = std::max(a.ymin + a.height / 2, b.ymin + b.height / 2);
    float diagonalDistance = std::pow(xC - a.xmin + xC - b.xmin, 2) + std::pow(yC - a.ymin + yC - b.ymin, 2);
    float diagonalLength = std::pow(std::max(a.width, b.width), 2) + std::pow(std::max(a.height, b.height), 2);
    float ciouTerm = diagonalDistance / diagonalLength;
    return IOU(a,b) - ciouTerm;
}

