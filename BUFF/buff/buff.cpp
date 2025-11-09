#include"buff.h"
#include"iou.h"


Rect buff::find_R(cv::Mat src,bool color,Point2f & R_center,Ptr<KNearest> model_R){
    bool flag=0;
    vector<Mat> channels;
    split(src,channels);
    Mat gray;
    if(color==0)//红色时处理
    {
        Mat kernel =getStructuringElement(MORPH_RECT, Size(3,3));
        threshold(channels[2]*1.2-channels[0],gray,100,255,THRESH_BINARY);//根据最大的值来动态二值化
        morphologyEx(gray,gray,MORPH_OPEN,kernel,Point(-1,-1));//闭操作去除较小的黑点，使图像连贯
        imshow("t",gray);
        kernel =getStructuringElement(MORPH_RECT, Size(5,5));
        dilate(gray,gray,kernel);

    }
    else{//蓝色部分处理
        Mat kernel =getStructuringElement(MORPH_RECT, Size(3,3));
        threshold(channels[0]-channels[2],gray,100,255,THRESH_BINARY);//根据最大的值来动态二值化
        morphologyEx(gray,gray,MORPH_OPEN,kernel,Point(-1,-1));//闭操作去除较小的黑点，使图像连贯
        kernel =getStructuringElement(MORPH_RECT, Size(5,5));
        dilate(gray,gray,kernel);
    }
#ifdef SHOW_ALL
    imshow("R",gray);
#endif
    vector<vector<Point>> contours;
    vector<Vec4i> hiearachy_test;
    findContours(gray, contours, hiearachy_test, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
     Rect R_rect;
    if(contours.size()>0){
        for(size_t i=0;i<contours.size();i++){
            cout<<"conconcocnocnocnocnon: "<<contourArea(contours[i])<<endl;
            if(contourArea(contours[i])>=100&&contourArea(contours[i])<=650){
                cout<<"conconco: "<<contourArea(contours[i])<<endl;
                Rect rect = boundingRect(contours[i]);
                Mat ROI=Mat(gray,rect);
                resize(ROI,ROI,Size(50,50));

                vector<vector<Point>> contours_;
                vector<Vec4i> hiearachy_test_;
                findContours(ROI, contours_, hiearachy_test_, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//                double max=0;
//                for(size_t j=0;i<contours_.size();j++){
//                    if(contourArea(contours_[j])>max)
//                          max=contourArea(contours_[j]);
//                }

                if(contours_.size()<2){
#ifdef SHOW_ALL
                    imshow("ROI",ROI);
#endif
                    Mat ROI_img_test;
                    Mat tmp3;
                    Mat predict_mat;
                    ROI.convertTo(ROI_img_test, CV_32F);
                    ROI_img_test.copyTo(tmp3);
                    predict_mat=tmp3.reshape(0, 1);

                    Mat predict_simple = predict_mat;
                    float r = model_R->predict(predict_simple);

//                    cout<<"RRRRRRRRRRRRRRRRRRRRRRRRRRRRRR: "<<r<<endl;

                    if(r){
                        R_center=Point2f(rect.x+rect.width/2,rect.y+rect.height/2);
                        flag=1;
                        return rect;
                    }
                    else {
                        flag=1;
                    }
                }

            }
        }
    }

//    return flag;
}

float enclidean_dis(Point2f p1,Point2f p2)
{
    float dis = sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2));
    return dis;
}

Point2f Rotation(float theta, Point2f center) {
    double rotationMatrix[2][2] = {
            {std::cos(theta), std::sin(theta)},
            {-std::sin(theta), std::cos(theta)}
        };
    std::array<double, 2> vector;
    vector[0]=center.x;
    vector[1]=center.y;
        std::array<double, 2> vector_;
        vector_[0] = rotationMatrix[0][0] * vector[0] + rotationMatrix[0][1] * vector[1];
        vector_[1] = rotationMatrix[1][0] * vector[0] + rotationMatrix[1][1] * vector[1];
        return Point2f(vector_[0],vector_[1]);

}

void RotationRectangle::_init(Point2f Points[4], Point2f Rbox_center)
{

 vector<pair<Point2f, float>>p;
 for(int i=0;i<4;i++)
 {
     float dis=enclidean_dis(Points[i],Rbox_center);
     p.push_back({Points[i],dis});
}
 sort(p.begin(), p.end(), [](const std::pair<Point2f, float>& a, const std::pair<Point2f, float>& b) {
         return a.second > b.second;
     });
p1=p[0].first;
p2=p[1].first;//p1,p2是离R最远的亮点
p[2].second=enclidean_dis(p[0].first,p[2].first);
p[3].second=enclidean_dis(p[0].first,p[3].first);
if(p[2].second>p[3].second)
{
    p3=p[2].first;
    p4=p[3].first;//13,24对角线
}
    else
{
    p3=p[3].first;
    p4=p[2].first;
}
points[0]=p1;
points[1]=p2;
points[2]=p3;
points[3]=p4;
    k13=(p3.y-p1.y)/(p3.x-p1.x);
    k24=(p4.y-p2.y)/(p4.x-p2.x);
    b13=p1.y-k13*p1.x;
    b24=p2.y-k24*p2.x;
    center12=_getlinecenter(p1,p2);
    center34=_getlinecenter(p3,p4);
    top=center12;
    botm=center34;
    topdis=enclidean_dis(top,Rbox_center);
    botmdis=enclidean_dis(botm,Rbox_center);
    center=Center();
    center_i=Center_i();
    width=Width();
    height=Height();
    area=Area();
}


Point2f RotationRectangle::_getlinecenter(Point2f p1,Point2f p2)
{
    float x1=p1.x;
    float y1=p1.y;
    float x2=p2.x;
    float y2=p2.y;
         float xmin = min(x1, x2);
         float xmax = max(x1, x2);
         float ymin = min(y1, y2);
         float ymax = max(y1, y2);
         float x = xmin + (xmax - xmin) / 2;
         float y = ymin + (ymax - ymin) / 2;
         return Point2f(x,y);
}


float RotationRectangle::Width()
{
    return enclidean_dis(p1,p2);
}

float RotationRectangle::Height()
{
    return enclidean_dis(p1,p4);
}
float RotationRectangle::Area()
    {
    return width*height;
}

Point2f RotationRectangle::Center()
{

//   Eigen::Matrix2f k;
//   k<<1,-1*k13,
//      1,-1*k24;

//  Eigen::Vector2f b;
//  b<<b13,
//     b24;
//  Eigen::Vector2f result=k.inverse()*b;
// float x=result[0];
// float y=result[1];
// return Point2f(x,y);




float k[2][2] = {
       {1, -1 * k13},
       {1, -1 * k24}
   };
float b[2][1] = {
       {b13},
       {b24}
   };

float inv_k[2][2];
float det = k[0][0] * k[1][1] - k[0][1] * k[1][0];

   inv_k[0][0] = k[1][1] / det;
   inv_k[0][1] = -k[0][1] / det;
   inv_k[1][0] = -k[1][0] / det;
   inv_k[1][1] = k[0][0] / det;

float x = inv_k[0][0] * b[0][0] + inv_k[0][1] * b[1][0];
float y = inv_k[1][0] * b[0][0] + inv_k[1][1] * b[1][0];

   return Point2f(x,y);
}

Point2i RotationRectangle::Center_i()
{
    float x=center.x;
    float y=center.y;
    return Point2i((int)x,(int)y);
}




Point2f BBox::Center()
{
    return Point2f((xmin+xmax)/2,(ymin+ymax)/2);
}
Point2i BBox::Center_i()
{
    return Point2i(int((xmin + xmax) / 2), int((ymin + ymax) / 2));
}
float BBox::Width()
{
    return  xmax-xmin;
}
float BBox::Height()
{
    return  ymax-ymin;
}
float BBox::Area()
{
    return width*height;
}
void BBox::setID(int BBox_id)
{
    id=BBox_id;
}
BBox BBox::create_new_bbox_center(Point2f center)
{
    int xmin=center.x-width/2;
    int ymin=center.y-height/2;
    int xmax=center.x+width/2;
    int ymax=center.y+height/2;
    BBox box;
    box.initbox(xmin,ymin,xmax,ymax,this->id);
    return box;

}
 void  BBox::initbox(float xmin,float ymin,float xmax,float ymax,int BBox_ID)
{
     this->xmin=xmin;
     this->ymin=ymin;
     this->xmax=xmax;
     this->ymax=ymax;
     this->id=BBox_ID;
     width=Width();
     height=Height();
     this->minpt=Point2f(xmin,ymin);
     this->maxpt=Point2f(xmin+width,ymin+height);
     area=Area();
     center=Center();
     center_i=Center_i();
}

float BBox::centerdis(BBox other)
 {
    return enclidean_dis(this->center,other.center);
 }

BBox BBox::boundof(BBox other)
{
    float xmin=min(this->xmin,other.xmin);
    float ymin=min(this->ymin,other.ymin);
    float xmax=max(this->xmax,other.xmax);
    float ymax=max(this->ymax,other.ymax);
    BBox box;
    box.initbox(xmin,ymin,xmax,ymax,-1);
    return box;
}
float BBox::bound_duijiao_dis(BBox other)
{
    BBox bound=this->boundof(other);
    return  enclidean_dis(Point2f(bound.xmin,bound.ymin),Point2f(bound.xmax,bound.ymax));
}



void fanBlade::initfan(BBox rect,RotationRectangle rtn_rect)
{
    bbox=rect;
    rtnrect=rtn_rect;
    state=None;
}



void Targetstruct::_inittarget(BBox box, float iou)
{
    this->box=box;
    this->iou=iou;
}




void BUFFtracker::initbuff(BBox BladeBox,BBox R_box,bool isimshow)
{
this->BladeBOX=BladeBox;
this->R_box=R_box;
//cout<<R_box.center<<endl;
  Point2f points[4];
BBox box;
RotationRectangle RotaRec;
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<float> dis(0.0, 1.0);
for (int i = 0; i < 5; i++)
{
    const int rows = 4;
    const int cols = 2;
    float randomArray[rows][cols];
    for (int j = 0; j < rows; ++j)
    {
       for (int k = 0; k < cols; ++k)
       {
                randomArray[j][k] = dis(gen);
        }
    }

    for (int j = 0; j < rows; ++j) {
            Point2f point;
            point.x = randomArray[j][0];
            point.y = randomArray[j][1];
            points[j]=point;
        }
//      cout<<points[0]<<endl;
    box.initbox(0,0,0,0,-1);
    RotaRec._init(points,this->R_box.center);
    fanBlade blade;
    blade.initfan(box,RotaRec);
    BladeList.push_back(blade);
}
//cout<<BladeList[0].bbox.area<<endl;
//cout<<BladeList[1].bbox.area<<endl;
states[0]=target;
states[1]=unlighted;
states[2]=unlighted;
states[3]=unlighted;
states[4]=unlighted;
radius=this->R_box.centerdis(BladeBox);
center=this->R_box.center;
isImshow=isimshow;
count=0;
fanNUM=0;

}

void BUFFtracker::getthreshold(MayBeTarget &MT)
{

 if(color==1&&buffsize==1)
 {
     lowhsv=Scalar(0,40,255);
     uphsv=Scalar(71,255,255);
     kernel=getStructuringElement(MORPH_RECT,Size(9,9));
     MT.area=0.1;
     MT.width=0.1;
     MT.height=0.1;
     outR=1.28;
     inR=0.74;
 }
 if(color==1&&buffsize==0)
 {
     lowhsv=Scalar(0,40,254);
     uphsv=Scalar(70,255,255);
     kernel=getStructuringElement(MORPH_RECT,Size(7,7));
     MT.area=0.1;
     MT.width=0.1;
     MT.height=0.1;
     outR=1.39;
     inR=0.7;

 }
 if(color==0&&buffsize==1)

 {
     lowhsv=Scalar(0,0,254);
     uphsv=Scalar(90,255,255);
     kernel=getStructuringElement(MORPH_RECT,Size(6,6));
     MT.area=0.1;
     MT.width=0.1;
     MT.height=0.1;
     outR=1.50;
     inR=0.76;
 }
}

Mat BUFFtracker:: HSVThreshold(Mat src)
{
    Mat hsv,mask;
    cvtColor(src,hsv,COLOR_BGR2HSV);
    inRange(hsv,lowhsv,uphsv,mask);
    dilate(mask,mask,kernel);
    return mask;
}

bool BUFFtracker::_MayTargert(float w, float h, bool flag)
{
    if(flag==true)
    {      float intersectionArea = std::min(w * h, this->R_box.area);
           float unionArea = std::max(w * h, this->R_box.area);
           bool condition1 = intersectionArea / unionArea > this->MT.area;
           bool condition2 = std::min(w, this->R_box.width) / std::max(w, this->R_box.width) > this->MT.width;
           bool condition3 = std::min(h, this->R_box.height) / std::max(h, this->R_box.height) > this->MT.height;
           return (condition1 && condition2 && condition3);
    }
    else {
        return true;
   }
}

 vector<Targetstruct> compareByIOU(BBox Rbox,vector<BBox>boxs,IoUType type)
 {
          vector<Targetstruct>ious;
          ious={};
          Targetstruct singleious;
          if(boxs.size()==0)
          {
            return ious;
          }
          else
          {
           for(size_t i=0;i<boxs.size();i++)
           {
               if (type==IoU)
               {
                singleious._inittarget(boxs[i],IOU(Rbox,boxs[i]));
                ious.push_back(singleious);
               }
               else if(type==CIoU)
               {
                   singleious._inittarget(boxs[i],CIOU(Rbox,boxs[i]));
                   ious.push_back(singleious);
               }
               else  if(type==GIoU)
               {
                   singleious._inittarget(boxs[i],GIOU(Rbox,boxs[i]));
                    ious.push_back(singleious);
               }
               else  if(type==DIoU)
               {
                   singleious._inittarget(boxs[i],DIOU(Rbox,boxs[i]));
                    ious.push_back(singleious);
               }
           }
         sort(ious.begin(), ious.end(), [](const Targetstruct& a, const Targetstruct& b) {
                 return a.iou > b.iou;
             });
         }
          return ious;
 }

 BBox BUFFtracker::Points2BBox(Point2f points[4])//获得四点的外接box
 {
     int numPoints = 4 ;// 点的数量
     vector<float> xs{};
     vector<float> ys{};
     for(int i=0;i<numPoints;i++)
     {
         xs.push_back(points[i].x);
         ys.push_back(points[i].y);
     }
     // 计算最小和最大坐标值
     float xmin = *std::min_element(xs.begin(), xs.end());
     float ymin = *std::min_element(ys.begin(), ys.end());
     float xmax = *std::max_element(xs.begin(), xs.end());
     float ymax = *std::max_element(ys.begin(), ys.end());
     BBox box;
     box.initbox(xmin,ymin,xmax,ymax,-1);
     return box;
}

 vector<fanBlade> BUFFtracker::getBlade(Mat mask)
 {
     vector<vector<Point>>contours;
     vector<Vec4i>hierarchy;
     findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
     vector<fanBlade>bladeList={};//扇叶备选
     vector<fanBlade>correctblade={};
     vector<fanBlade>realBladeList={};//真实扇叶子
     Point2f rectpts[4];
     for(size_t i=0;i<contours.size();i++)
     {
         RotatedRect rect=minAreaRect(contours[i]);
         rect.points(rectpts);//顺时针取点
       Point2f rectPtsInt[4];
         for (int j = 0; j < 4; j++)
         {
             rectPtsInt[j]=Point2f((int)rectpts[j].x, (int)rectpts[j].y);
         }
//         cout<<rectPtsInt[0]<<" "<<rectPtsInt[1]<<" "<<rectPtsInt[2]<<" "<<rectPtsInt[3]<<endl;
         RotationRectangle rtnrect;
         rtnrect._init(rectPtsInt,R_box.center);
//         cout<<rtnrect.topdis<<" "<<rtnrect.botmdis<<" "<<rtnrect.area<<" "<<radius<<" "<<R_box.area<<endl;
         if(0.4*radius<rtnrect.botmdis<rtnrect.topdis<1.5*radius && rtnrect.area>2*R_box.area)//备选框条件
         {
             Rect rect=boundingRect(contours[i]);
             float x=rect.tl().x;
             float y=rect.tl().y;
             float w=rect.width;
             float h=rect.height;
             if(isImshow)
             {
                rectangle(mask,Point2f(x,y),Point2f(x+w,y+h),Scalar(255,255,255),3);
             }
             fanBlade blade;
             BBox box;
             box.initbox(x,y,x+w,y+h,-1);
             blade.initfan(box,rtnrect);
             bladeList.push_back(blade);
         }
     }
     if(isImshow)
     {
         imshow("mask_",mask);
     }
//     cout<<BladeList[0].bbox.area<<" "<<bladeList.size()<<endl;
     if(BladeList[0].bbox.area==0 && bladeList.size()==1)//第一帧只有一个扇页，初始化操作
     {
         bladeList[0].bbox.id=0;
         return  bladeList;
     }
     else if(BladeList[0].bbox.area==0 && bladeList.size()!=1)
     {
         bladeList={};
         return bladeList;
     }
     map<int, std::vector<fanBlade>> tempList;
     tempList.clear();
     for(size_t i=0;i<BladeList.size();i++)
     {
         Point2f  tempxy=Point2f(BladeList[i].bbox.center.x-center.x+R_box.center.x,BladeList[i].bbox.center.y-center.y+R_box.center.y);
         BBox tempBox=BladeList[i].bbox.create_new_bbox_center(tempxy);
         Point2f temppoints[4]={};
         for(int j=0;j<4;j++)
         {
             Point2f pt(BladeList[i].rtnrect.points[j].x-center.x+R_box.center.x,BladeList[i].rtnrect.points[j].y-center.y+R_box.center.y);
             temppoints[j]=pt;
         }
         fanBlade blade;
         RotationRectangle rotationrec;
       rotationrec._init(temppoints,R_box.center);
       blade.initfan(tempBox,rotationrec);
       correctblade.push_back(blade);
       if(isImshow)
       {
           rectangle(frame,tempBox.minpt,tempBox.maxpt,Scalar(255,0,0),3 );
          string text="id="+to_string(tempBox.id)+"|lastBlade";
            putText(frame,text,Point2f(tempBox.minpt.x-30,tempBox.minpt.y-30),FONT_HERSHEY_SIMPLEX,0.75,Scalar(255,0,0),2);
       }
     }

     for(size_t i=0;i<correctblade.size();i++)
     {
       BBox box=correctblade[i].bbox;
       tempList[box.id]=vector<fanBlade>();
       for(size_t j=0;j<bladeList.size();j++)
       {
           if(IOU(box,bladeList[j].bbox)>0)
           {
               tempList[box.id].push_back(bladeList[j]);
           }
       }
       if(tempList[box.id].size()==1)
       {
           tempList[box.id][0].bbox.id=box.id;
           realBladeList.push_back(tempList[box.id][0]);
       }
       else if(tempList[box.id].size()>=2)
       {
        std::map<int, std::vector<std::pair<Point2f, float>>> tempP;
        for (int k = 1; k <= 4; ++k) {
              tempP[k] = std::vector<std::pair<Point2f, float>>();
          }

        for(size_t k=0;k<tempList[box.id].size();k++)
        {
        tempP[1].push_back({tempList[box.id][k].rtnrect.p1,enclidean_dis(tempList[box.id][k].rtnrect.p1,R_box.center)});
        tempP[2].push_back({tempList[box.id][k].rtnrect.p2,enclidean_dis(tempList[box.id][k].rtnrect.p2,R_box.center)});
        tempP[3].push_back({tempList[box.id][k].rtnrect.p3,enclidean_dis(tempList[box.id][k].rtnrect.p3,R_box.center)});
        tempP[4].push_back({tempList[box.id][k].rtnrect.p4,enclidean_dis(tempList[box.id][k].rtnrect.p4,R_box.center)});
        }
        auto maxElement1 = std::max_element(tempP[1].begin(), tempP[1].end(), [](const std::pair<Point2f, float>& a, const std::pair<Point2f, float>& b) { return a.second < b.second; });
        auto maxElement2 = std::max_element(tempP[2].begin(), tempP[2].end(), [](const std::pair<Point2f, float>& a, const std::pair<Point2f, float>& b) { return a.second < b.second; });
        auto minElement1 = std::min_element(tempP[3].begin(), tempP[3].end(), [](const std::pair<Point2f, float>& a, const std::pair<Point2f, float>& b) { return a.second > b.second; });
        auto minElement2 = std::min_element(tempP[4].begin(), tempP[4].end(), [](const std::pair<Point2f, float>& a, const std::pair<Point2f, float>& b) { return a.second > b.second; });

        Point2f p1 = maxElement1->first;
        Point2f p2 = maxElement2->first;
        Point2f p3 = minElement1->first;
        Point2f p4 = minElement2->first;
        Point2f points[4]={p1,p2,p3,p4};
        BBox bbox;
        bbox=Points2BBox(points);
        bbox.id=box.id;
        RotationRectangle rtr;
        fanBlade blade1;
        rtr._init(points,R_box.center);
        blade1.initfan(bbox,rtr);
        realBladeList.push_back(blade1);
       }
     }
     //更新扇叶状态（判断是否为待击打或为亮起的
     for(size_t i=0;i<realBladeList.size();i++)
     {
         if(realBladeList.size()==1)
         {
             realBladeList[0].bbox.id=0;
             states[0]=target;
             for(int j=1;j<5;j++)
             {
                 states[j]=unlighted;
             }
         }
         else if(realBladeList.size()>fanNUM)//如果现在的亮起个数大于上一帧亮起的个数，则上一帧未亮起的一定是带击打的，上一阵已亮起的，一定是已击打的
         {
             int id_x=realBladeList[i].bbox.id;
             if(states[id_x]==target)
                states[id_x]=shot;
             else if(states[id_x]==unlighted)
                states[id_x]=target;
         }
     }
    return  realBladeList;
//     return bladeList;
 }


bool BUFFtracker::update(Mat frame, bool isopenMaybetarget)
{

        if(isImshow)
          this->frame=frame;
        vector<int>lightbladeidlist;
        lightbladeidlist={0};//设置0为亮的扇叶
        getthreshold(MT);
//        cout<<MT.area<<" "<<MT.width<<" "<<MT.height<<endl;
        Mat resultframe=HSVThreshold(frame);
        if(isImshow)
            imshow("mask",resultframe);
        vector<BBox>boxs;
        boxs={};
        vector<vector<Point>>contours;
        vector<Vec4i>hierarchy;
        findContours(resultframe,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
        BBox box;
        for(size_t i=0;i<contours.size();i++)//获得 Rbox的备选框
        {
            Rect rect=boundingRect(contours[i]);
            float x=rect.tl().x;
            float y=rect.tl().y;
            float w=rect.width;
            float h=rect.height;
            float dis=enclidean_dis(Point2f(x+w/2,y+h/2),R_box.center);
          if(_MayTargert(w,h,isopenMaybetarget) && dis < radius)
          {
               float xmin=x;
               float ymin=y;
               float xmax=x+w;
               float ymax=y+h;
              box.initbox(xmin,ymin,xmax,ymax,-1);
              boxs.push_back(box);
          }
        }
//        cout<<boxs.size()<<endl;
        vector<Targetstruct>box_and_iou;
        IoUType ioutype=CIoU;
        box_and_iou=compareByIOU(R_box,boxs,ioutype);
//        cout<<box_and_iou.size()<<endl;
//        cout<<box_and_iou[0].iou<<endl;
       if(box_and_iou.size()!=0 && box_and_iou[0].iou>-1)
       {
           R_box=box_and_iou[0].box;
//         cout<<R_box.center<<endl;
       }
       else {
           return false;
       }
//       rectangle(frame,R_box.minpt,R_box.maxpt,Scalar(0,0,255),3);
//       putText(frame,"R",R_box.minpt,FONT_HERSHEY_SIMPLEX,0.75,Scalar(0,0,255),2);
       circle(resultframe,R_box.center_i,(int)radius*inR,Scalar(0,0,0),-1);
       circle(resultframe,R_box.center_i,(int)radius*outR,Scalar(0,0,0),3);
       if(isImshow)
           imshow("result",resultframe);
       vector<fanBlade>bladelist;
       bladelist=getBlade(resultframe);
       waitKey(1);
       center=R_box.center;
       count+=1;
//       cout<<bladelist.size()<<endl;
       if (bladelist.size()==0)
       {
           return false;
       }

       fanNUM=bladelist.size();
       for(size_t i=0;i<bladelist.size();i++)
       {
           BBox box;
           box=bladelist[i].bbox;
//           cout<<box.id<<endl;
           if(box.id!=0)
           {
               lightbladeidlist.push_back(box.id);
           }
           BladeList[box.id].bbox=box;
           if(states[box.id]==target)
           BladeBOX=box;
           if(isImshow)
           {
               State states[] = {target, unlighted, shot, None}; // 假设states数组中包含了相应的枚举成员

               std::string stateText;
               switch (states[box.id]) {
                   case target:
                       stateText = "Target";
                       break;
                   case unlighted:
                       stateText = "Nolight";
                       break;
                   case shot:
                       stateText = "Shot";
                       break;
                   case None:
                       stateText = "None";
                       break;
               }
               rectangle(frame,box.minpt,box.maxpt,Scalar(0,0,255),2);
               string text="id="+to_string(box.id)+" "+stateText;
               putText(frame,text,box.minpt,FONT_HERSHEY_SIMPLEX,0.75,Scalar(0,0,255),2);
               rectangle(frame,R_box.minpt,R_box.maxpt,Scalar(0,0,255),3);
               putText(frame,"R",R_box.minpt,FONT_HERSHEY_SIMPLEX,0.75,Scalar(0,0,255),2);
           }
       }
       //更新未亮起的扇叶顶的信息
       for(int i=0;i<5;i++)
       {
           if (std::find(lightbladeidlist.begin(), lightbladeidlist.end(), i) != lightbladeidlist.end()) {
                    continue;
                }
           float pi=M_PI;
           Point2f Pt=Rotation(pi*2/5*i,Point2f(BladeList[0].bbox.center.x-R_box.center.x,BladeList[0].bbox.center.y-R_box.center.y));
           BladeList[i].bbox=BladeBOX.create_new_bbox_center(Point2f(Pt.x+R_box.center.x,Pt.y+R_box.center.y));
           BladeList[i].bbox.id=i;
       }
       return true;
}
