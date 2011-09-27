#include <MT/util.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
  VideoCapture capture;
  capture.open(0);

  if(!capture.isOpened()){
    cerr <<"Could not initialize video capture" <<endl;
    return -1;
  }

  //namedWindow("Image View");

  MT::timerStart();
  Mat img,last;
  uint t;
  for(t=0;;t++){
    capture >>img;
    imshow("original image", img);

    //-- blurrrr
    Mat blur;
    GaussianBlur(img, blur, Size(5,5), 1., 1.);
    imshow("blurred image", blur);

    //-- Canny
    Mat canny,imgGray;
    float th=50.;
    cvtColor(img, imgGray, CV_BGR2GRAY);
    Canny(imgGray, canny,  th, 4.f*th, 3);
    imshow("Canny image", canny);

    //-- Hough
    Mat hough;
    vector<Vec4i> lines;
    HoughLinesP( canny, lines, 1, CV_PI/180, 50, 30, 10 );
    img.copyTo(hough);
    for(uint i=0; i<lines.size(); i++){
      line( hough, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 2, 8 );
    }
    imshow("Hough lines", hough);

    //-- difference
    Mat diff;
    if(t){
      diff = img-last;
      imshow("difference image",diff);
    }
    img.copyTo(last);

    //-- SURF keypoints and features
    Mat surfImg;
    vector<KeyPoint> keypoints;
    SurfFeatureDetector	surf(400);
    surf.detect(imgGray, keypoints);
    drawKeypoints(imgGray,keypoints,surfImg, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("SURF keypoints", surfImg);

    if((waitKey(2)&0xff)==27)  break;
  }
  cout <<"fps=" <<(double)t/MT::timerRead() <<endl;
  return 0;
}
