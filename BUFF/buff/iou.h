#ifndef IOU_H
#define IOU_H
#include"buff.h"
float IOU(BBox a,BBox b);
float GIOU(BBox a,BBox b);
float DIOU(BBox a,BBox b);
float CIOU(BBox a,BBox b);
#endif // IOU_H
