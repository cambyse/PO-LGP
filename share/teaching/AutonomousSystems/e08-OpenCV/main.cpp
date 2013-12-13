#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifdef HAVE_OPENCV_NONFREE
#include <opencv2/nonfree/features2d.hpp>
#endif

using namespace cv;
using namespace std;

int main(int argc, char** argv){
    VideoCapture capture;
    capture.open(0);
    if(!capture.isOpened()){
        cerr <<"Could not initialize video capture" <<endl;
        return -1;
    }
    Mat img, kpImg, imgGray, last, blur, diff, canny, dst, mask, hough;
    vector<Vec2f> lines; // for hough
    vector<KeyPoint> keypoints;
#ifdef HAVE_OPENCV_NONFREE
    SURF surf;
#endif

    uint t;
    for(t=0;;t++){
        capture >>img;

        // part 1, blur
        GaussianBlur(img, blur, Size(5,5), 1., 1.);
        imshow("blurred image", blur);

        // part 2, difference image
        diff = img - last;
        img.copyTo(last);
        imshow("difference image", diff);

        // part 3, canny
        cvtColor(img, imgGray, CV_BGR2GRAY);
        Canny(imgGray, canny, 50, 200.f, 3);
        imshow("canny edge image", canny);

        // part 4, hough transform
        HoughLines(canny, lines, 1, CV_PI/180, 100, 0, 0 );
        img.copyTo(hough);
        for( size_t i = 0; i < lines.size(); i++ )
        {
          float rho = lines[i][0], theta = lines[i][1];
          Point pt1, pt2;
          double a = cos(theta), b = sin(theta);
          double x0 = a*rho, y0 = b*rho;
          pt1.x = cvRound(x0 + 1000*(-b));
          pt1.y = cvRound(y0 + 1000*(a));
          pt2.x = cvRound(x0 - 1000*(-b));
          pt2.y = cvRound(y0 - 1000*(a));
          line( hough, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
        }

        imshow("hough lines ", hough);

#ifdef HAVE_OPENCV_NONFREE
        // part 5, surf features
        surf(imgGray, mask, keypoints);
        drawKeypoints(img, keypoints, kpImg, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        imshow("SURF keypoints", kpImg);
#endif

        if((waitKey(2)&0xff)==27)
            break;
    }
    return 0;
}
