
#include "cvutils.h"

template<class T>
void np::draw_circles(byteA& image, MT::Array<T>& centers, MT::Array<T>& rad)
{
  CvMat image_cv = cvMat(image.d0, image.d1, CV_8UC3, image.p);
  for (uint i = 0; i < centers.d0; i++)
  {
    cvCircle(&image_cv, cvPoint(cvRound(centers(i,0)),cvRound(centers(i,1))),
               3, CV_RGB(200,200,200), -1, 8, 0 );
    cvCircle(&image_cv, cvPoint(cvRound(centers(i,0)),cvRound(centers(i,1))),
             cvRound(rad(i)), CV_RGB(128,128,128), 3, 8, 0 );
  }
}

