#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX

#include <MT/vision.h>
#include <MT/optimization.h>
#include <MT/perceptionModule.h>

//declarations from within perception.cpp... actually internals...

void generateShapePoints(arr& points, arr& weights, arr *grad, uint type, uint N, const arr& params);

struct ShapeFitProblem:public OptimizationProblem {
  floatA distImage;
  uint type, N;
  arr x, points;
  bool display;
  
  double f(arr *grad, const arr& x, int i=-1);
};


void testShapes(){
  //test creating shape points
  byteA img(700,700,3);
  arr params;
  for(uint k=0;k<10;k++){
    //params = ARR(400, 400, 100, 30, 0, 100, -100, 30);
    params.setText("[ 0, 0, .5, -.2,  1, 0,  1, 1,  .5, 1.2,  0, 1]");
    params *= 100.;
    params += 400.;
    rndUniform(params,-5.,5.,true);
    arr points,weights;
    generateShapePoints(points,weights,NULL,3,20,params);
    cout <<params <<endl;
    img.setZero(255);
    cvDrawPoints(img,points);
    cvShow(img,"shape",true);
  }
}

void testColorBasedShapeTracking(){
  byteA img;
  CvMatDonor cvMatDonor;

  read_ppm(img,"hsvTheta-smooth7.ppm");
  floatA theta = rgb2evi(img);

  cvShow(img,"org image",true);

#if 1
  //flood fill
  uint W=theta.d1;
  uint i=theta.maxIndex(); float max1 = theta.elem(i); if(max1 < 0.5) return;//assume no detection when unsure....
  CvConnectedComp component;
  byteA mask(theta.d0+2,theta.d1+2); mask.setZero();
  cvFloodFill( CVMAT(theta), cvPoint(i%W,i/W), cvScalar(1.f),
               cvScalar(0.3f), cvScalar(0.3f),
               &component, CV_FLOODFILL_FIXED_RANGE, CVMAT(mask) );//4th and 5th parameter are flood tolerance, 0.3 originally
#else
  //threshold
  uint maxi=theta.maxIndex(); float max1 = theta.elem(maxi); if(max1 < 0.5) return;//assume no detection when unsure....
  //uint W=theta.d1;
  byteA mask(theta.d0,theta.d1);
  cvThreshold(CVMAT(theta), CVMAT(mask), max1-.3f, 1.f, CV_THRESH_BINARY);
#endif
  
  cvShow(byte(255)*mask,"mask",true);
  //cvShow(evi2rgb(theta),"flood theta",true);

  //draw a contour image
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* contour = 0;
  cvFindContours( CVMAT(mask), storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(1, 1));
  byteA contourImage; resizeAs(contourImage,theta); contourImage.setZero(255);
  cvDrawContours( CVMAT(contourImage), contour, cvScalar(0.f), cvScalar(128.f), 0 );
  cvShow(contourImage,"contour",true);

  //distance image
  floatA distImage; distImage.resizeAs(theta); distImage.setZero();
  cvDistTransform( CVMAT(contourImage), CVMAT(distImage), CV_DIST_L2, 5);
  cvShow(.01f*(distImage),"distance",true);

  arr params;
  for(uint k=0;k<10;k++){
    //params = ARR(500, 500, 100, 200, 0,0);
    params = ARR(component.rect.x + 0.5*component.rect.width,
                 component.rect.y + 0.5*component.rect.height,
                 component.rect.width,
                 .7*component.rect.height,
                 .2*component.rect.height);
    params = ARR(component.rect.x + .5*component.rect.width,
                 component.rect.y,
                 .5*component.rect.width, .2*component.rect.height,
                 0, .8*component.rect.height,
                 -.5*component.rect.width, .2*component.rect.height);
    /*
      params.resize(6,2);
    params.setText("[ 0, 0, .5, -.2,  1, 0,  1, 1,  .5, 1.2,  0, 1]");
    for(uint k=0;k<6;k++){
      params(2*k+0) = component.rect.x + component.rect.width *params(2*k+0);
      params(2*k+1) = component.rect.y + component.rect.height*params(2*k+1);
    }
    */
    rndUniform(params,-10.,10.,true);

    ShapeFitProblem problem;
    problem.type=2;
    problem.N=20;
    problem.distImage = pow(distImage,2.f);
    problem.display = true;

    //checkGradient(problem,params,1.);

    MT::timerStart();
    double cost;
    Rprop rprop;
    rprop.dMax = 5.;
    rprop.init(3.);
    rprop.loop(params,problem,&cost,1.e-1,100);
    cout <<"*** cost=" <<cost <<" params=" <<params <<" time=" <<MT::timerRead() <<endl;

    problem.f(NULL,params);
    byteA img; copy(img,10.f*problem.distImage);
    cvDrawPoints(img,problem.points);
    cvShow(img,"shape optimization",false);
  }
}


int main(int argn,char** argv){
  //testShapes();
  testColorBasedShapeTracking();
  
  return 0;
}
