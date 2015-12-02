#include <Core/util.h>

#ifdef MLR_OPENCV

#include <opencv2/opencv.hpp>
#undef MIN
#undef MAX

#include <Perception/opencv.h>

void TEST(Show){
  byteA rgbL,rgbR;
  read_ppm(rgbL,"left.ppm");
  read_ppm(rgbR,"right.ppm");

  cvShow(rgbL,"left");
  cvShow(rgbR,"right",true);
}

void TEST(Surf){
  byteA rgbL,rgbR,rgbDraw;
  read_ppm(rgbL,"left.ppm");
  read_ppm(rgbR,"right.ppm");
  
  cvShow(rgbL,"left");
  cvShow(rgbR,"right",true);
  CvMemStorage* storage;

  //-- convert gray
  byteA greyL(rgbL),greyR(rgbR);
  make_grey(greyL);
  make_grey(greyR);
  cvShow(greyL,"left");
  cvShow(greyR,"right",true);

  //-- Canny
  byteA edges(greyL),tmp(greyL);
  float th=mlr::getParameter<float>("cannyTh");
  cv::Canny(cvMAT(greyL), cvMAT(edges), th, 4.f*th, 3);
  //cvSmooth(cvMAT(edges), cvMAT(tmp), CV_BLUR, 3, 3);
  cvShow(edges,"Canny",true);

  //-- SURF keypoints and features
  // CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
  // CvMemStorage* storage = cvCreateMemStorage(0);
  // mlr::timerStart();
  // cvExtractSURF(cvMAT(greyL), NULL,
  //               &imageKeypoints, &imageDescriptors,
  //               storage, cvSURFParams(500, 1) );
  // cout <<"Image Descriptors =" <<imageDescriptors->total
  //      <<"\nExtraction time = " <<mlr::timerRead() <<"sec" <<endl;
  // CvSeqReader reader;
  // cvStartReadSeq( imageKeypoints, &reader, 0 );
  // rgbDraw=rgbL;
  // for(int i=0; i<imageKeypoints->total; i++){
  //   const CvSURFPoint* kp = (const CvSURFPoint*)reader.ptr;
  //   CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
  //   cvCircle(cvMAT(rgbDraw), cvPointFrom32f(kp->pt), 3, cvScalar(255,0,0));
  //   /*cout <<i
  //   <<" pos=" <<kp->pt.x <<',' <<kp->pt.y
  //   <<" laplacian=" <<kp->laplacian
  //   <<" size=" <<kp->size
  //   <<" dir=" <<kp->dir
  //   <<" hessian=" <<kp->hessian <<endl;
  //   */

  // }
  // cvShow(rgbDraw,"left-keypoints",true);

  // //-- good corner points
  // floatA tmp1,tmp2;
  // resizeAs(tmp1,greyL);
  // resizeAs(tmp2,greyL);
  // CvPoint2D32f corners[100];
  // int cornerCount;
  // mlr::timerStart();
  // cvGoodFeaturesToTrack(cvMAT(greyL), cvMAT(tmp1), cvMAT(tmp2),
  // 			corners, &cornerCount, .1f, 5, NULL, 5, 0);
  // cout <<"Good Corners #=" <<cornerCount <<" time=" <<mlr::timerRead() <<endl;
  // rgbDraw = rgbL;
  // for(int i=0; i<cornerCount; i++){
  //   cvCircle(cvMAT(rgbDraw), cvPointFrom32f(corners[i]), 3, cvScalar(0,155,0));
  // }
  // cvShow(rgbDraw,"left-keypoints",true);
 
  // //-- Star features
  // storage = cvCreateMemStorage(0);
  // CvSeq* keypoints = 0;
  // keypoints = cvGetStarKeypoints( cvMAT(greyL), storage, cvStarDetectorParams(45) );
  // rgbDraw=rgbL;
  // for(int i=0; i<(keypoints?keypoints->total:0);i++){
  //   CvStarKeypoint kpt = *(CvStarKeypoint*)cvGetSeqElem(keypoints, i);
  //   int r = kpt.size/2;
  //   cvCircle( cvMAT(rgbDraw), kpt.pt, r, CV_RGB(0,255,0));
  //   cvLine(  cvMAT(rgbDraw), cvPoint(kpt.pt.x + r, kpt.pt.y + r),
  //            cvPoint(kpt.pt.x - r, kpt.pt.y - r), CV_RGB(0,255,0));
  //   cvLine(  cvMAT(rgbDraw), cvPoint(kpt.pt.x - r, kpt.pt.y + r),
  //            cvPoint(kpt.pt.x + r, kpt.pt.y - r), CV_RGB(0,255,0));
  // }
  // cvShow(rgbDraw,"Star keypoints",true);

  // //-- Canny - Hough
  // storage = cvCreateMemStorage(0);
  // CvSeq* lines = 0;
  // cvShow(edges, "hough input");
  // mlr::timerStart();
  // lines = cv::HoughLinesP( cvMAT(edges), storage,
  // 			   1, CV_PI/180, 50, 30, 10 );
  // cout <<"Hough #=" <<lines->total <<" time=" <<mlr::timerRead() <<endl;
  // rgbDraw = rgbL;
  // for(int i = 0; i < mlr::MIN(lines->total,100); i++ ){
  //   CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
  //   cvLine( cvMAT(rgbDraw), line[0], line[1], CV_RGB(255,0,0), 2.);
  // }
  // cvShow(rgbDraw, "Canny -- Hough", true);

  // //-- Displarity BM & GC
  // floatA dispL,dispR;
  // resizeAs(dispL,greyL);
  // resizeAs(dispR,greyR);
  // int maxDisp=16;

  // CvStereoBMState* statebm = cvCreateStereoBMState(CV_STEREO_BM_BASIC, maxDisp);
  // mlr::timerStart();
  // cvFindStereoCorrespondenceBM(cvMAT(greyL), cvMAT(greyR),
  //                              cvMAT(dispL), statebm);
  // dispL /= float(maxDisp);
  // cvReleaseStereoBMState( &statebm );
  // cout <<"stereo BM, time = " <<mlr::timerRead()
  //      <<" maxL=" <<dispL.max() <<endl;
  // cvShow(dispL, "dispL BM", true);


  // CvStereoGCState* stategc = cvCreateStereoGCState( maxDisp, 3 );
  // mlr::timerStart();
  // cvFindStereoCorrespondenceGC(cvMAT(greyL), cvMAT(greyR),
  //                              cvMAT(dispL), cvMAT(dispR),
  //                              stategc);
  // dispL /= -float(maxDisp);
  // dispR /=  float(maxDisp);
  // cvReleaseStereoGCState( &stategc );
  // cout <<"stereo GC, time = " <<mlr::timerRead()
  //      <<" maxL=" <<dispL.max() <<" maxR=" <<dispR.max() <<endl;
  // cvShow(dispL, "dispL");
  // cvShow(dispR, "dispR", true);
}


void TEST(ShiftAnalysis){
  byteA rgbL,rgbR,rgbDraw;
  read_ppm(rgbL,"left.ppm");
  read_ppm(rgbR,"right.ppm");

  cvShow(rgbL,"left");
  cvShow(rgbR,"right",true);

  byteA shiftR,disp;
  floatA diff,max;
  disp.resize(rgbL.d0,rgbL.d1);
  disp.setZero();
  mlr::timerStart();
  uint maxD = 31;
  for(uint d=0;d<maxD;d++){
    shiftR=rgbR;
    shiftR.shift(3*d,true);
    getDiffProb(diff, shiftR, rgbL, 10., 5);
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
  cout <<"time = " <<mlr::timerRead() <<endl;
  // byteA tmp(disp);
  // cvSmooth(cvMAT(disp), cvMAT(tmp), CV_BLUR, 5, 5);
  // cvSmooth(cvMAT(tmp), cvMAT(disp), CV_BLUR, 5, 5);
  cvShow(disp, "disparity", true);
}


int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  switch(mlr::getParameter<int>("mode")){
  case 1:  testShow();  break;
  case 2:  testSurf();  break;
  case 3:  testShiftAnalysis();  break;
  default: HALT("");
  }

  return 0;
}

#else //MLR_OPENCV

int main(int argc,char** argv){
  NICO;
  return 0;
}

#endif
