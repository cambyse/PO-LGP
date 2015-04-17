#define MT_IMPLEMENT_TEMPLATES

#include <signal.h>

#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <MT/BinaryBP.h>
#include <MT/vision.h>

#include <MT/opencv.h>
#include <perception/perception.h>
#include <hardware/uvccamera.h>

ENABLE_CVMAT

Process* camera = NULL;

void TEST(){
  Image imgL("CameraL"), imgR("CameraR");
  camera = newUVCCamera();

  camera->open();

  byteA img,tmp;
  floatA rgb,hsv,phi,last,diff;
  BinaryBPGrid bp;
  for(uint t=0;;t++){
    last = rgb;
    camera->step();
    img.resize(imgL.img.d0/2,imgL.img.d1/2,3);
    cvResize(CVMAT(imgL.img), CVMAT(img));

    cvShow(img,"1");
    
    byte2float(rgb,img);
    rgb2hsv(hsv,rgb);
    uint i,X=hsv.d1,Y=hsv.d0;

    getHsvEvidences(phi,hsv,ARRAY<float>(.0,1.,1.),ARRAY<float>(.2,.7,.7));

    phi.reshape(Y*X);
    //gnuplotHistogram(phi,-10,10);
    phi.reshape(Y,X);
    
    tmp.resize(Y,X);
    for(i=0;i<Y*X;i++) tmp.elem(i) = 128.*(1.+tanh(phi.elem(i)));
    
    if(t){
      diff = rgb-last;
      cvShow(diff,"2");
    }

            
    bp.phi=phi;
    bp.tanh_J = .7;
    bp.step();
    for(i=0;i<Y*X;i++) tmp.elem(i) = 128.*(1.+tanh(bp.b.elem(i)));
    cvShow(tmp,"3");
    
    bp.discount(.8);
  }
}

void shutdown(int){
   camera->close();
   HALT("");
}

int main(int argc,char** argv){
  signal(SIGINT,shutdown);
   
  test();
  
  return 0;
}
