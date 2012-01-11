#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX

#define MT_IMPLEMENT_TEMPLATES

#include <signal.h>

#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/BinaryBP.h>
#include <MT/vision.h>

#include <NP/libcolorseg.h>
//#include <NP/common.h>
#include <NP/uvccamera.h>
#include <NP/uvccamera.cpp>
//#include <NP/camera/bumblebee2.h>

#if 0//def MT_BUMBLE
typedef camera::Bumblebee2 Camera;
#else
typedef camera::UVCCamera Camera;
#endif
Camera *cam;
ENABLE_CVMAT



#if 0
#include <ess.hh>

double findBox(floatA &box,const floatA &phi){
  uint X=phi.d1,Y=phi.d0,N=X*Y,i,x,y;

  arr weights(12),pos(2,N),clst(N);

  weights(11)=-.1;
  for(uint i=0;i<=10;i++) weights(i)=.1*i;
  for(i=0;i<N;i++){
    if(phi.elem(i)<0.f) clst(i)=11;
    else if(phi.elem(i)>1.f) clst(i)=10;
    else clst(i)=floor(10.f*phi.elem(i));
  }
  for(x=0;x<X;x++) for(y=0;y<Y;y++){ i=y*X+x;  pos(0,i)=x;  pos(1,i)=y; }
  
  Box b = pyramid_search(N, X, Y,
                       &pos(0,0), &pos(1,0),
                       clst.p, 2, 3, weights.p);

  box.resize(4);
  box(0)=b.left;
  box(1)=b.top;
  box(2)=b.right;
  box(3)=b.bottom;
  cout <<box <<endl;

  return b.score;
}

void test(){
  Camera uvc; cam=&uvc;
  uvc.open();

  byteA img,tmp;
  floatA rgb,hsv,phi;
  BinaryBPGrid bp;
  for(uint t=0;;t++){
    uvc.step();
    img.resize(uvc.output.rgbL.d0/2,uvc.output.rgbL.d1/2,3);
    cvResize(CVMAT(uvc.output.rgbL), CVMAT(img));

    //cvShow(img,"1");  continue;
    
    byte2float(rgb,img);
    rgb2hsv(hsv,rgb);
    uint i,X=hsv.d1,Y=hsv.d0;

    getHsvEvidences(phi,hsv,ARRAY<float>(.0,1.,1.),ARRAY<float>(.2,.7,.7));

    phi.reshape(Y*X);
    //gnuplotHistogram(phi,-10,10);
    phi.reshape(Y,X);
    
    tmp.resize(Y,X);
    for(i=0;i<Y*X;i++) tmp.elem(i) = 128.*(1.+tanh(phi.elem(i)));
    
    floatA box;
    if(t>10){
      findBox(box,phi);
      cvDrawBox(img,box);
    }
    cvShow(img,"1");
    cvShow(tmp,"2");
            
    bp.phi=phi;
    bp.tanh_J = .7;
    bp.step();
    for(i=0;i<Y*X;i++) tmp.elem(i) = 128.*(1.+tanh(bp.b.elem(i)));
    cvShow(tmp,"3");
    
    bp.discount(.8);
  }
}

void testESS(){
  Camera uvc;
  uvc.open();

  byteA img,tmp;
  floatA rgb,hsv;
  BinaryBPGrid bp;
  for(;;){
    uvc.step();
    img.resize(uvc.output.rgbL.d0/3, uvc.output.rgbL.d1/3,3);
    cvResize(CVMAT(uvc.output.rgbL), CVMAT(img));
    cvShow(img,"1");
  }
}
#endif

void testFelz(){
  Camera uvc; cam = &uvc;
  uvc.open();

  byteA img,tmp;
  uintA pch;
  arr pch_cen;
  uintA pch_edges;
  floatA pch_rgb,pch_hsv;
  floatA pch_evid,pch_grey;
  uint np;
  for(uint t=0;;t++){
    uvc.step();
    img.resize(uvc.output.rgbL.d0/2,uvc.output.rgbL.d1/2,3);
    cvResize(CVMAT(uvc.output.rgbL), CVMAT(img));
    np=get_single_color_segmentation(pch,img,1.25,100,100);
    np=incremental_patch_ids(pch);
                          
    /*random_colorMap(pch_rgb,np);
    pch2img(tmp,pch,pch_rgb);
    cvShow(tmp,"2");*/

    get_patch_colors(pch_rgb,img,pch,np);
    pch2img(tmp,pch,pch_rgb);

    /*get_patch_centroids(pch_cen,img,pch,np);
    if(t>10){
      getDelaunayEdges(pch_edges, pch_cen);
      cvDrawGraph(tmp,pch_cen,pch_edges);
      }*/
    cvShow(tmp,"3");

    pch_rgb/=255.f;
    rgb2hsv(pch_hsv,pch_rgb);
    getHsvEvidences(pch_evid,pch_hsv,ARRAY<float>(.0,1.,1.),ARRAY<float>(.2,.5,.8));

    pch_grey.resizeAs(pch_evid);
    for(uint i=0;i<pch_evid.N;i++) pch_grey.elem(i) = .5*(1.+tanh(pch_evid.elem(i)));
    pch2img(tmp,pch,pch_grey);
    cvShow(tmp,"4");
    cvShow(img,"1");
  }
}


void shutdown(int){
   cam->close();
   exit(0);
}

int main(int argn,char** argv){
   signal(SIGINT,shutdown);
   
  //test();
  testFelz();
  
  return 0;
}
