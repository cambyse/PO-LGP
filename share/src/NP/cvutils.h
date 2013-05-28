/*  Copyright 2009 Nils Plath
    email: nilsp@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/> */

/** @file cvutils.h
    @brief Routines to ease use of MT::Array with OpenCV */

#ifndef _NP_CVUTILS_H
#define _NP_CVUTILS_H

#ifdef MT_OPENCV

#include <typeinfo>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <MT/array.h>
class CvBoost;                                         // defined in opencv/ml.h

#include "nputils.h"
namespace np {
void map_CvMat2byteA(CvMat& cvmat, byteA& bytea);
void copy_iplimage2byteA(byteA& img, IplImage& img_cv);
void copy_byteA2iplimage(IplImage& img_cv, byteA& img);
char display(const char* window, byteA& img, int cvwaitkey = 0, bool rgb2bgr = true);
void save_image(const byteA& img, const char* filename);
void load_image(byteA& img, const char* filename, int is_color = -1);
void convert(byteA& out, byteA& in, int mode);

template<class T>
void draw_circles(byteA& image, MT::Array<T>& centers, MT::Array<T>& rad);

template<class T>
void map_cvmat2mtarray(CvMat& cvmat, MT::Array<T>& mtarray)
{
  if (mtarray.nd > 3)
    msg_error(HERE, "number of dimensions of input must be <= 3");
  if (mtarray.N == 0)
    msg_error(HERE, "input must be non-empty");

  int cvtype=0;
  uint d0=1,d1=1,d2=1;
  d0 = mtarray.d0;
  if (mtarray.nd >= 2) d1 = mtarray.d1;
  if (mtarray.nd == 3) d2 = mtarray.d2;

  switch ((int)(*typeid(T).name()))
  {
    case 'c': // char
      cvtype = (mtarray.nd <= 2 ? CV_8S : CV_8SC3);
      break;
    case 'h': // uchare
      cvtype = (mtarray.nd <= 2 ? CV_8U : CV_8UC3);
      break;
    case 'i': // int
      cvtype = (mtarray.nd <= 2 ? CV_32S : CV_32SC3);
      break;
//    case 'j': // uint
//      cvtype = (mtarray.nd > 2 ? CV_32UC1 : CV_32UC3);
//      break;
    case 'f': // float
      cvtype = (mtarray.nd <= 2 ? CV_32F : CV_32FC3);
      break;
    case 'd': // double
      cvtype = (mtarray.nd <= 2 ? CV_64F : CV_64FC3);
      break;
    default:
      msg_error(HERE, "unknown typeid");
  };

  if (mtarray.nd == 1)
    cvmat = cvMat(d0, 1, cvtype, mtarray.p);
  else
    cvmat = cvMat(d0, d1, cvtype, mtarray.p);
};

/** @brief KMeans clustering algorithm
 *
 *  Wrapper for the OpenCV implementation of KMeans.
 *
 *  @note this algorithm uses floatA AND NOT doubleA (b/c of OpenCV)
 *
 *  @param clusters      cluster centers, size: (N,2), N>0
 *  @param labels        denotes to which cluster center each sample belongs to
 *  @param compactness   sum(sqrdist(sample_i-center_{labels_i})), the lower the better
 *  @param samples       data matrix, size: (M,D), M>0, D>0
 *  @param num_clusters  number of clusters, >0
 *  @param termcrit_d    termination threshold
 *  @param termcrit_i    max number of iterations
 *  @param attempts      number of restarts, final result has lowest compactness [1]
 *  @param use_labels    use param labels for initial cluster distribution [false]
 *
 *  @return cf. parameter points
 *
 */
int kmeans
(
  floatA& clusters,
  intA& labels,
  double& compactness,
  const floatA& samples,
  uint num_clusters,
  double termcrit_d = 1.0,
  uint termcrit_i = 10,
  int attempts = 1,
  bool use_labels = false
);

/** @brief Trains a Boosted Decision Forest
 *
 *  For details search OpenCV docs for "CvDTrees::train".
 *
 *  @param data       M-by-D matrix, row-wise samples
 *  @param responses  M-by-1 vector with discrete (cls.) or cont. (reg.) values
 *  @param boost_type CvBoost::{REAL,LOGIT,...} (real AdaBoost, LogitBoost, ...)
 *  @param weak_count number of weak classifiers
 *  @param weight_trim_rate range: [0.-1.], ignore samples small weights
 *  @param max_depth  max. no. tree layers
 *  @param use_surrogates true, if working with missing data, else false
 *  @param priors     “cost” of false positives
 *
 *  @return pointer to trained instance of a Boosted Decision Forest
 */
CvBoost* boostDTrees_train
(
 const floatA& data,
 const floatA& responses,
 int boost_type, /*          = CvBoost::REAL,*/
 uint num_classes        = 2,
 int weak_count          = 100,
 double weight_trim_rate = 0.95,
 int max_depth           = 5,
 bool use_surrogates     = false,
 const float* priors     = 0
);

/** @brief Prediction with Boosted Decision Forest
 *
 *  For details search OpenCV docs for "CvDTrees::predict".
 *
 *  @param responses    predicted class labels, size: (N)
 *  @param sum          summed votes of weak classifiers
 *  @param boost        trained Boosted Decision Forest
 *  @param data         data matrix, size: (N,D)
 *  @param num_classes  number of classes or categories of data
 *
 *  @return 0 on OK, cf. params. responses and sum
 */
int boostDTrees_predict
(
 floatA& responses,
 floatA& sums,
 CvBoost* boost,
 floatA& data,
 uint num_classes
);

/** @brief Remaps image pixels and interpolates
 *
 *  For details search OpenCV docs for "cvRemap()".
 *
 *  @param rimg  remapped image, size: (h,w[,3])
 *  @param img   image, size: (h,w[,3])
 *  @param mapx  distortion map for x-coordinates, size: (h,w)
 *  @param mapy  distortion map for y-coordinates, size: (h,w)
 *
 *  @return 0 on OK, cf. param. rimg
 */
int remap(byteA& rimg, const byteA& img, const floatA& mapx, const floatA& mapy);

/** @brief Tracks colored ball
 *
 *  Tracks colored ball by thresholding HSV space of input image between 
 *  min_{h,s,v} and max_{h,s,v}, followed by a Hough Transform to detect 
 *  circles. Multiple balls can be found.
 *
 *  @param centers  image coordinates of ball center(s), size: (N,2), N - no. balls
 *  @param r        radiae of found circles, size: (N)
 *  @param image    RGB input image, size: (y,x,3)
 *  @param min_h    mininum H value for thresholding
 *  @param min_s    mininum S value for thresholding
 *  @param min_v    mininum V value for thresholding
 *  @param min_h    maximum H value for thresholding
 *  @param min_s    maximum S value for thresholding
 *  @param min_v    maximum V value for thresholding
 *  @param draw     draw centers and circle into input image
 *
 *  @return cf. params centers, r, image
 */
void track_ball
(
 floatA& centers,
 floatA& r,
 byteA& image,
 uint min_h,
 uint min_s,
 uint min_v,
 uint max_h,
 uint max_s,
 uint max_v,
 bool is_hsv = false,
 bool draw = false
);

/** @brief Tracks {red,green,blue} ball
 *
 *  Tracks any reasonably {red,green,blue} ball by thresholding HSV space.
 *
 *  @see track_ball()
 *  @param centers  image coordinates of ball center(s), size: (N,2), N - no. balls
 *  @param r        radiae of found circles, size: (N)
 *  @param image    RGB input image, size: (y,x,3)
 *  @param draw     draw centers and circle into input image
 *
 *  @return cf. params centers, r, image
 */
void track_red_ball(floatA& centers, floatA& r, byteA& image, bool is_hsv = false, bool draw = false);
void track_green_ball(floatA& centers, floatA& r, byteA& image, bool is_hsv = false, bool draw = false);
void track_blue_ball(floatA& centers, floatA& r, byteA& image, bool is_hsv = false, bool draw = false);

/** @brief Tracks a red, a green, and a blue ball
 *
 *  Tracks one instance of each a red, a green, and a blue ball.
 *
 *  @see track_ball()
 *  @param left           left image
 *  @param right          right image
 *  @param centers_left   image coordinates of ball center(s), size: (N,2), N - no. balls
 *  @param centers_right  image coordinates of ball center(s), size: (N,2), N - no. balls
 *  @param rad_left       radiae of found circles, size: (N)
 *  @param rad_right      radiae of found circles, size: (N)
 *
 *  @return true if 6 instance found, else false; also cf. params {centers,rad}_{left,right}
 */
bool track_RGB_balls
(
  byteA& left,
  byteA& right,
  floatA& centers_left,
  floatA& centers_right,
  floatA& rad_left,
  floatA& rad_right
);

/** @brief Detects the chessboard corners
 *
 *  Detects the inner (num_x-1)x(num_y-1) corners of a chessboard pattern
 *  with subpixel accuracy.
 *
 *  @see OpenCV::cvFindChessboardCorners()
 *  @param corners  detected corners
 *  @param image    right image
 *  @param num_x    number of square on x-axis, num_x >= 2
 *  @param num_y    number of square on y-axis, num_y >= 2
 *
 *  @return 0 on OK, cf. params corners
 */
int find_chessboard_corners(floatA& corners, byteA& image, uint num_x, uint num_y, bool draw = false);

void stereo_calibration
(
  doubleA& Kl, doubleA& Kr,      /* left/right cmaera intrinsics, size: (3,3) */
  doubleA& dl, doubleA& dr,  /* left/right distortion parameters, size: (5,1) */
  doubleA& R, doubleA& T,    /* right to left rot./transl., size: (3,3),(3,1) */
  doubleA& E, doubleA& F,    /* essential and fundamental matrix, size: (3,3) */
  doubleA& Pl, doubleA& Pr,    /* left/right projection matrices, size: (3,4) */
  doubleA& Rl, doubleA& Rr,      /* left/right rotation matrices, size: (3,3) */
  doubleA& Q,                 /* 2D-to-3D back-projection matrix, size: (4,4) */
  floatA& cleft,                                           /* left 2D corners */
  floatA& cright,                                         /* right 2D corners */
  floatA& cworld,                                         /* world 3D corners */
  uint num_views,
  uint nx,
  uint ny,
  uint image_width,
  uint image_height,
  bool use_guess = false
);

void stereo_calibration_save
(
  doubleA& Kl, doubleA& Kr,      /* left/right cmaera intrinsics, size: (3,3) */
  doubleA& dl, doubleA& dr,  /* left/right distortion parameters, size: (5,1) */
  doubleA& R, doubleA& T,    /* right to left rot./transl., size: (3,3),(3,1) */
  doubleA& E, doubleA& F,    /* essential and fundamental matrix, size: (3,3) */
  doubleA& Pl, doubleA& Pr,    /* left/right projection matrices, size: (3,4) */
  doubleA& Rl, doubleA& Rr,      /* left/right rotation matrices, size: (3,3) */
  doubleA& Q,                 /* 2D-to-3D back-projection matrix, size: (4,4) */
  floatA& cleft,                                           /* left 2D corners */
  floatA& cright,                                         /* right 2D corners */
  floatA& cworld,                                         /* world 3D corners */
  const char* filename
);

void stereo_calibration_load
(
  doubleA& Kl, doubleA& Kr,      /* left/right cmaera intrinsics, size: (3,3) */
  doubleA& dl, doubleA& dr,  /* left/right distortion parameters, size: (5,1) */
  doubleA& R, doubleA& T,    /* right to left rot./transl., size: (3,3),(3,1) */
  doubleA& E, doubleA& F,    /* essential and fundamental matrix, size: (3,3) */
  doubleA& Pl, doubleA& Pr,    /* left/right projection matrices, size: (3,4) */
  doubleA& Rl, doubleA& Rr,      /* left/right rotation matrices, size: (3,3) */
  doubleA& Q,                 /* 2D-to-3D back-projection matrix, size: (4,4) */
  floatA& cleft,                                           /* left 2D corners */
  floatA& cright,                                         /* right 2D corners */
  floatA& cworld,                                         /* world 3D corners */
  const char* filename
);

inline void triangulate(floatA& p3D, const floatA& p2D, const floatA& d, float f, float b)
{
  double bf = b*f, finv=1./f;
  uint num_points = p2D.d0;
  p3D.resize(num_points,3);
  for (uint i=0; i<num_points; i++)
  {
    if (d(i) == 0.) 
    {
//      std::cout << "nannery" << std::endl; 
      continue;
    }
    p3D(i,2) = bf/d(i);
    p3D(i,0) = p2D(i,0)*p3D(i,2)*finv;
    p3D(i,1) = p2D(i,1)*p3D(i,2)*finv;
  }
}

inline void circle_center_3D(floatA& cX, const floatA& X, const floatA& c0, float radius)
{
  floatA v = X-c0;
  float normv_inv = 1./norm(v);
  for (uint i=0;i<v.N;i++)
    v(i)*=normv_inv*radius;
  cX = X+v;
};

// "glue" imgL and imgR together
void merge(byteA& imgLR, const byteA& imgL, const byteA& imgR);

} // namespace np

#if defined NP_IMPLEMENT_TEMPLATES
#  include "cvutils_t.cpp"
#endif

#endif

#endif
