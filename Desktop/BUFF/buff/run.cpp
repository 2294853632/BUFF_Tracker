#include"run.h"

void run()
{

     VideoCapture video;
     video.open("/home/zhangjiajun/Videos/12mm_blue_dark.MP4");
     int framecount=0;
     BUFFtracker tracker;
     bool isImshow=true;
     BBox R_box;
     BBox BladeBox;
//     enum clockMode clockmode=none;
//     vector<float>angles={};
//     angleObesever Observer;
     Mat frame,ret;
     int start=225;

    while(1)
    {
       video>>frame;
       video>>ret;
//        base base;
       if(ret.empty())
           break; 
       framecount+=1;
       if (framecount>=start)
       {
               if(framecount==start)
               {
                     Rect rect=selectROI("roi1",frame);
//                     buff buff;
//                   Point2f center=Point2f(frame.rows/2,frame.cols/2);
//                   Rect rect=buff.find_R(frame,base.enemy_team, center,base.model_R);
//                   Mat imageroi=frame(rect);
//                   imshow("ROI",imageroi);
                   Rect rect2=selectROI("roi2",frame);
//                   Mat imageroi1=frame(rect2);
//                   imshow("ROI2",imageroi1);
                   R_box.initbox(rect.tl().x,rect.tl().y,rect.tl().x+rect.width,rect.tl().y+rect.height,-1);
                   BladeBox.initbox(rect2.tl().x,rect2.tl().y,rect2.tl().x+rect2.width,rect2.tl().y+rect2.height,-1);
                   tracker.initbuff(BladeBox,R_box,isImshow);
//                   cout<<tracker.radius<<endl;
                   tracker.color=0;
                   tracker.buffsize=BIG;
//                   Observer._init(clockmode);
               }
                 bool flag=tracker.update(frame,true);
//                 rectangle(frame,tracker.R_box.minpt,tracker.R_box.maxpt,Scalar(0,0,255),3);
//               circle(frame,tracker.R_box.center,30,Scalar(0,255,0),3);
                 if(flag==false)
                 cout<<"False"<<endl;
//               float x=tracker.BladeBOX.center.x-tracker.R_box.center.x;
//               float y=tracker.BladeBOX.center.y-tracker.R_box.center.y;
//               float angle=Observer.update(x,y,tracker.radius);
//               angles.push_back(angle);
               rectangle(frame,tracker.BladeBOX.minpt,tracker.BladeBOX.maxpt,Scalar(0,255,0),3);
               imshow("frame",frame);
               int c=waitKey(15);
               if(c=='q'||c=='Q')
                   break;
        }
    }
}
