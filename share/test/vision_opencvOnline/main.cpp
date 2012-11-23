#define MT_IMPLEMENT_TEMPLATES

#include <signal.h>

#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/BinaryBP.h>
#include <MT/vision.h>

#include <MT/opencv.h>


#if 0//def MT_BUMBLE
typedef camera::Bumblebee2 Camera;
#else
typedef camera::UVCCamera Camera;
#endif
Camera *cam;
ENABLE_CVMAT


void test(){
  Camera uvc; cam=&uvc;
  uvc.open();

  byteA img,tmp;
  floatA rgb,hsv,phi,last,diff;
  BinaryBPGrid bp;
  for(uint t=0;;t++){
    last = rgb;
    uvc.step();
    img.resize(uvc.output.rgbL.d0/2,uvc.output.rgbL.d1/2,3);
    cvResize(CVMAT(uvc.output.rgbL), CVMAT(img));

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
   cam->close();
   HALT("");
}

int main(int argn,char** argv){
  signal(SIGINT,shutdown);
   
  test();
  
  return 0;
}
