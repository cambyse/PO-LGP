#ifndef MT_robot_processes_h
#define MT_robot_processes_h

#include <MT/vision.h>
#include <NP/camera.h>

struct LEDtracker:public Process,Variable{
  MyCamera *var;

  //OUTPUT
  floatA center;

  byteA rgbL, rgbR, green;
  floatA theta,tmp;

 LEDtracker():Process("LEDtracker"),Variable("LEDtracker"){}

  void open(){}
  void close(){}
  void step(){
    CvMatDonor cvMatDonor;

    if(!var){ MT_MSG("Variable pointer not set"); return; }
    var->output.readAccess(this);
    rgbL = var->output.rgbL;
    rgbR = var->output.rgbR;
    var->output.deAccess(this);

    green.resize(rgbL.d0,rgbL.d1);
    floatA cen;
    uintA box;
    for(uint c=0;c<2;c++){
      if(c==0) cvSplit(CVMAT(rgbL), NULL, CVMAT(green), NULL, NULL);
      if(c==1) cvSplit(CVMAT(rgbR), NULL, CVMAT(green), NULL, NULL);

      copy(theta,green);
      theta /= 256.f;
      tmp.resizeAs(theta);
      cvSmooth(CVMAT(theta), CVMAT(tmp), CV_BLUR, 5, 5);
      theta=tmp;

      findMaxRegionInEvidence(box, &cen, NULL, theta, .05);

      writeAccess(this);{
        center.resize(2,2);
        center[c] = cen;
      }deAccess(this);
    }
    
    //return;
    cvRectangle(CVMAT(theta), cvPoint(box(0),box(1)), cvPoint(box(2),box(3)), cvScalar(0));
    cvRectangle(CVMAT(theta), cvPoint(cen(0)-1,cen(1)-1), cvPoint(cen(0)+1,cen(1)+1), cvScalar(0.));
    cvShow(theta,"2");
  }
};

#endif
