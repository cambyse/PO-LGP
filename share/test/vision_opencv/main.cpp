#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX

#include <MT/vision.h>
#include <MT/util.h>

ENABLE_CVMAT

void testShow(){
  byteA rgbL,rgbR;
  read_ppm(rgbL,"left.ppm");
  read_ppm(rgbR,"right.ppm");

  cvShow(rgbL,"left");
  cvShow(rgbR,"right",true);
}

void testSurf(){
  byteA rgbL,rgbR,rgbDraw;
  read_ppm(rgbL,"left.ppm");
  read_ppm(rgbR,"right.ppm");
  
  cvShow(rgbL,"left");
  cvShow(rgbR,"right",true);

  //-- convert gray
  byteA greyL(rgbL),greyR(rgbR);
  make_grey(greyL);
  make_grey(greyR);
  cvShow(greyL,"left");
  cvShow(greyR,"right",true);

  //-- Canny
  byteA edges(greyL),tmp(greyL);
  float th=MT::getParameter<float>("cannyTh");
  cvCanny(CVMAT(greyL), CVMAT(edges), th, 4.f*th, 3);
  //cvSmooth(CVMAT(edges), CVMAT(tmp), CV_BLUR, 3, 3);
  cvShow(edges,"Canny",true);

  //-- SURF keypoints and features
  CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
  CvMemStorage* storage = cvCreateMemStorage(0);
  MT::timerStart();
  cvExtractSURF(CVMAT(greyL), NULL,
                &imageKeypoints, &imageDescriptors,
                storage, cvSURFParams(500, 1) );
  cout <<"Image Descriptors =" <<imageDescriptors->total
       <<"\nExtraction time = " <<MT::timerRead() <<"sec" <<endl;
  CvSeqReader reader;
  cvStartReadSeq( imageKeypoints, &reader, 0 );
  rgbDraw=rgbL;
  for(int i=0; i<imageKeypoints->total; i++){
    const CvSURFPoint* kp = (const CvSURFPoint*)reader.ptr;
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    cvCircle(CVMAT(rgbDraw), cvPointFrom32f(kp->pt), 3, cvScalar(255,0,0));
    /*cout <<i
    <<" pos=" <<kp->pt.x <<',' <<kp->pt.y
    <<" laplacian=" <<kp->laplacian
    <<" size=" <<kp->size
    <<" dir=" <<kp->dir
    <<" hessian=" <<kp->hessian <<endl;
    */

  }
  cvShow(rgbDraw,"left-keypoints",true);

  //-- good corner points
  floatA tmp1,tmp2;
  resizeAs(tmp1,greyL);
  resizeAs(tmp2,greyL);
  CvPoint2D32f corners[100];
  int cornerCount;
  MT::timerStart();
  cvGoodFeaturesToTrack(CVMAT(greyL), CVMAT(tmp1), CVMAT(tmp2),
                        corners, &cornerCount, .1f, 5, NULL, 5, 0);
  cout <<"Good Corners #=" <<cornerCount <<" time=" <<MT::timerRead() <<endl;
  rgbDraw = rgbL;
  for(int i=0; i<cornerCount; i++){
    cvCircle(CVMAT(rgbDraw), cvPointFrom32f(corners[i]), 3, cvScalar(0,155,0));
  }
  cvShow(rgbDraw,"left-keypoints",true);
 
  //-- Star features
  storage = cvCreateMemStorage(0);
  CvSeq* keypoints = 0;
  keypoints = cvGetStarKeypoints( CVMAT(greyL), storage, cvStarDetectorParams(45) );
  rgbDraw=rgbL;
  for(int i=0; i<(keypoints?keypoints->total:0);i++){
    CvStarKeypoint kpt = *(CvStarKeypoint*)cvGetSeqElem(keypoints, i);
    int r = kpt.size/2;
    cvCircle( CVMAT(rgbDraw), kpt.pt, r, CV_RGB(0,255,0));
    cvLine(  CVMAT(rgbDraw), cvPoint(kpt.pt.x + r, kpt.pt.y + r),
             cvPoint(kpt.pt.x - r, kpt.pt.y - r), CV_RGB(0,255,0));
    cvLine(  CVMAT(rgbDraw), cvPoint(kpt.pt.x - r, kpt.pt.y + r),
             cvPoint(kpt.pt.x + r, kpt.pt.y - r), CV_RGB(0,255,0));
  }
  cvShow(rgbDraw,"Star keypoints",true);

  //-- Canny - Hough
  storage = cvCreateMemStorage(0);
  CvSeq* lines = 0;
  cvShow(edges, "hough input");
  MT::timerStart();
  lines = cvHoughLines2( CVMAT(edges), storage,
                         CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 50, 30, 10 );
  cout <<"Hough #=" <<lines->total <<" time=" <<MT::timerRead() <<endl;
  rgbDraw = rgbL;
  for(int i = 0; i < MT::MIN(lines->total,100); i++ ){
    CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
    cvLine( CVMAT(rgbDraw), line[0], line[1], CV_RGB(255,0,0), 2.);
  }
  cvShow(rgbDraw, "Canny -- Hough", true);

  //-- Displarity BM & GC
  floatA dispL,dispR;
  resizeAs(dispL,greyL);
  resizeAs(dispR,greyR);
  int maxDisp=16;

  CvStereoBMState* statebm = cvCreateStereoBMState(CV_STEREO_BM_BASIC, maxDisp);
  MT::timerStart();
  cvFindStereoCorrespondenceBM(CVMAT(greyL), CVMAT(greyR),
                               CVMAT(dispL), statebm);
  dispL /= float(maxDisp);
  cvReleaseStereoBMState( &statebm );
  cout <<"stereo BM, time = " <<MT::timerRead()
       <<" maxL=" <<dispL.max() <<endl;
  cvShow(dispL, "dispL BM", true);


  CvStereoGCState* stategc = cvCreateStereoGCState( maxDisp, 3 );
  MT::timerStart();
  cvFindStereoCorrespondenceGC(CVMAT(greyL), CVMAT(greyR),
                               CVMAT(dispL), CVMAT(dispR),
                               stategc);
  dispL /= -float(maxDisp);
  dispR /=  float(maxDisp);
  cvReleaseStereoGCState( &stategc );
  cout <<"stereo GC, time = " <<MT::timerRead()
       <<" maxL=" <<dispL.max() <<" maxR=" <<dispR.max() <<endl;
  cvShow(dispL, "dispL");
  cvShow(dispR, "dispR", true);
}


void testShiftAnalysis(){
  byteA rgbL,rgbR,rgbDraw;
  read_ppm(rgbL,"left.ppm");
  read_ppm(rgbR,"right.ppm");

  cvShow(rgbL,"left");
  cvShow(rgbR,"right",true);

  byteA shiftR,disp;
  floatA diff,max;
  disp.resize(rgbL.d0,rgbL.d1);
  disp.setZero();
  MT::timerStart();
  uint maxD = 31;
  for(uint d=0;d<maxD;d++){
    shiftR=rgbR;
    shiftR.shift(3*d,true);
    getDiffProb(diff,shiftR,rgbL,10.,5);
    cvShow(shiftR, "shift");
    cvShow(diff, "diff");
    if(!d) max=diff; 
    else{
      for(uint i=0;i<diff.N;i++){
        if(diff.elem(i)>2.*max.elem(i)){ //HACK!
          max.elem(i)=diff.elem(i);
          disp.elem(i)=d;
        }
      }
    }
  }
  disp *= byte(255/maxD);
  cout <<"time = " <<MT::timerRead() <<endl;
  byteA tmp(disp);
  cvSmooth(CVMAT(disp), CVMAT(tmp), CV_BLUR, 5, 5);
  cvSmooth(CVMAT(tmp), CVMAT(disp), CV_BLUR, 5, 5);
  cvShow(disp, "disparity", true);
}


int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  switch(MT::getParameter<int>("mode")){
  case 1:  testShow();  break;
  case 2:  testSurf();  break;
  case 3:  testShiftAnalysis();  break;
  default: HALT("");
  }

  return 0;
}
