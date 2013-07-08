/*  Copyright 2010 Nils Plath
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

/** @file opencv_helper.h
    @brief Wrappers around OpenCV routines. */

#ifndef NP_OPENCV_HELPER_H
#define NP_OPENCV_HELPER_H

#include <opencv/cv.h>

#include "nputils.h"
#include "cvutils.h"

inline void cvPolyLine(byteA& img, const doubleA& pts, int is_closed=0, CvScalar color=cvScalar(0,255,255), int thickness=1)
{
  if (pts.N==0 || pts.d1 != 2)
    return;
  CvMat cvimg = cvMat(img.d0,img.d1,CV_8UC3,img.p);
  int num_pts = pts.d0;
  int num_curves=1;
  MT::Array<CvPoint> cv_pts;
  for (int j=0; j<num_pts;j++)
    cv_pts.append(cvPoint((int)pts(j,0),(int)pts(j,1))); //
  CvPoint* curveArr[1]={cv_pts.p};
  cvPolyLine(&cvimg,curveArr,&num_pts,num_curves,is_closed,color,thickness);
};

inline void cvPolyLine(byteA& img, const MT::Array<doubleA>& lines, int is_closed=0, CvScalar color=cvScalar(0,255,255), int thickness=1)
{
  if (lines.N==0)
    return;
  for (uint i=0; i<lines.N; i++)
    cvPolyLine(img, lines(i), is_closed, color, thickness);
};

inline void cvShowImage(const char* name, byteA& img)
{
  CvMat cvimg = cvMat(img.d0,img.d1,(img.nd==2?CV_8U:CV_8UC3),img.p);
  cvShowImage(name, &cvimg);
};

inline void cvRectangle(byteA& img, CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int lineType=8, int shift=0)
{
  CvMat cvimg = cvMat(img.d0,img.d1,(img.nd==2?CV_8U:CV_8UC3),img.p);
  cvRectangle(&cvimg, pt1, pt2, color, thickness, lineType, shift);
};

inline void cvCvtColor(byteA& dst, const byteA& src, int code)
{
  if (dst.N == 0) np::msg_error(HERE, "did you allocate memory?!");
  CvMat cvdst = cvMat(dst.d0,dst.d1,(dst.nd==2?CV_8U:CV_8UC3),dst.p);
  CvMat cvsrc = cvMat(src.d0,src.d1,(src.nd==2?CV_8U:CV_8UC3),src.p);
  cvCvtColor(&cvsrc, &cvdst, code);
};

inline void cvPutText(byteA& img, const char* text, CvPoint org, const CvFont* font, CvScalar color)
{
  CvMat cvimg = cvMat(img.d0,img.d1,(img.nd==2?CV_8U:CV_8UC3),img.p);
  cvPutText(&cvimg, text, org, font, color);
};

inline void cvResize(byteA& dst, const byteA& src, int interpolation=CV_INTER_LINEAR)
{
  CvMat cvdst = cvMat(dst.d0,dst.d1,(dst.nd==2?CV_8U:CV_8UC3),dst.p);
  CvMat cvsrc = cvMat(src.d0,src.d1,(src.nd==2?CV_8U:CV_8UC3),src.p);
  cvResize(&cvsrc, &cvdst, interpolation);
};

inline int cvFindChessboardCorners(byteA& img, CvSize patternSize, CvPoint2D32f* corners, int* cornerCount=NULL, int flags=CV_CALIB_CB_ADAPTIVE_THRESH)
{
  CvMat cvimg = cvMat(img.d0,img.d1,(img.nd==2?CV_8U:CV_8UC3),img.p);
  return cvFindChessboardCorners(&cvimg, patternSize, corners, cornerCount, flags);
};

inline void cvDrawChessboardCorners(byteA& img, CvSize patternSize, CvPoint2D32f* corners, int count, int patternWasFound)
{
  CvMat cvimg = cvMat(img.d0,img.d1,(img.nd==2?CV_8U:CV_8UC3),img.p);
  cvDrawChessboardCorners(&cvimg, patternSize, corners, count, patternWasFound);
};

inline void cvFindCornerSubPix(byteA& img, CvPoint2D32f* corners, int count, CvSize win, CvSize zero_zone, CvTermCriteria criteria)
{
  CvMat cvimg = cvMat(img.d0,img.d1,(img.nd==2?CV_8U:CV_8UC3),img.p);
  cvFindCornerSubPix(&cvimg, corners, count, win, zero_zone, criteria);
};

inline double cvStereoCalibrate
 (
  MT::Array<CvPoint3D32f> &objectPoints,
  MT::Array<CvPoint2D32f> &imagePoints1,
  MT::Array<CvPoint2D32f> &imagePoints2,
  intA& pointCounts,
  doubleA &cameraMatrix1,
  doubleA &distCoeffs1,
  doubleA &cameraMatrix2,
  doubleA &distCoeffs2,
  CvSize imageSize,
  doubleA &R,
  doubleA &T,
  doubleA &E,
  doubleA &F,
  CvTermCriteria term_crit=cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5),
  int flags=CV_CALIB_FIX_ASPECT_RATIO+CV_CALIB_ZERO_TANGENT_DIST
           +CV_CALIB_SAME_FOCAL_LENGTH
//   int flags=CV_CALIB_ZERO_TANGENT_DIST+CV_CALIB_FIX_K3
 )
{
  std::cout << "bin hier!" << std::endl;
  uint N=pointCounts(0)*pointCounts.d0;
  floatA pts2dL(N,2), pts2dR(N,2), pts3d(N,3);
  for (uint i=0;i<N;i++)
  {
    pts2dL(i,0) = imagePoints1(i).x; pts2dL(i,1) = imagePoints1(i).y;
    pts2dR(i,0) = imagePoints2(i).x; pts2dR(i,1) = imagePoints2(i).y;
    pts3d(i,0) = objectPoints(i).x; pts3d(i,1) = objectPoints(i).y; pts3d(i,2) = objectPoints(i).z;
  }

  CvMat cvobjectPoints  = cvMat(1, N, CV_32FC3, pts3d.p);
  CvMat cvimagePoints1  = cvMat(1, N, CV_32FC2, pts2dL.p);
  CvMat cvimagePoints2  = cvMat(1, N, CV_32FC2, pts2dR.p);

//   CvMat cvobjectPoints  = cvMat(1, N, CV_32FC3, objectPoints.p);
//   CvMat cvimagePoints1  = cvMat(1, N, CV_32FC2, imagePoints1.p);
//   CvMat cvimagePoints2  = cvMat(1, N, CV_32FC2, imagePoints2.p);
  CvMat cvpointCounts   = cvMat(1, pointCounts.d0, CV_32S, pointCounts.p);

  if (cameraMatrix1.N==0) {cameraMatrix1.resize(3,3); cameraMatrix1=0.; cameraMatrix1.setDiag(1.);}
  if (cameraMatrix2.N==0) {cameraMatrix2.resize(3,3); cameraMatrix2=0.; cameraMatrix2.setDiag(1.);}
  if (distCoeffs1.N==0) {distCoeffs1.resize(1,5); distCoeffs1=0.;}
  if (distCoeffs2.N==0) {distCoeffs2.resize(1,5); distCoeffs2=0.;}
  if (R.N==0) {R.resize(3,3); R=0.;}
  if (T.N==0) {T.resize(3,1); T=0.;}
  if (E.N==0) {E.resize(3,3); E=0.;}
  if (F.N==0) {F.resize(3,3); F=0.;}

  CvMat cvcameraMatrix1 = cvMat(3, 3, CV_64F, cameraMatrix1.p);
  CvMat cvcameraMatrix2 = cvMat(3, 3, CV_64F, cameraMatrix2.p);
  CvMat cvdistCoeffs1   = cvMat(1, 5, CV_64F, distCoeffs1.p);
  CvMat cvdistCoeffs2 = cvMat(1, 5, CV_64F, distCoeffs2.p);
  CvMat cvR  = cvMat(3, 3, CV_64F, R.p);
  CvMat cvT  = cvMat(3, 1, CV_64F, T.p);
  CvMat cvE  = cvMat(3, 3, CV_64F, E.p);
  CvMat cvF  = cvMat(3, 3, CV_64F, F.p);

  return ::cvStereoCalibrate
        (
          &cvobjectPoints,
          &cvimagePoints1,
          &cvimagePoints2,
          &cvpointCounts,
          &cvcameraMatrix1,
          &cvdistCoeffs1,
          &cvcameraMatrix2,
          &cvdistCoeffs2,
          imageSize,
          &cvR, &cvT, &cvE, &cvF, term_crit, flags
        );
};

inline void cvUndistortPoints(
   doubleA& dst,
   doubleA& src,
   doubleA& intrinsic_matrix,
   doubleA& distortion_coeffs,
   doubleA *R = 0,
   const doubleA* P = 0
)
{
  if (src.d0 == 0) return;
  if (src.d1 != 2) return;
  if (dst.N != src.N) dst.resize(src.d0, src.d1);
  if (intrinsic_matrix.N != 9) return;
  if (distortion_coeffs.N != 5) return;
  if (R!=NULL)
    if (R->N!=9)
      return;
  if (P!=NULL)
    if (P->N!=12 && P->N!=9)
      return;
  CvMat cvsrc = cvMat(src.d0, src.d1, CV_64FC1, src.p);
  CvMat cvdst = cvMat(dst.d0, dst.d1, CV_64FC1, dst.p);
  CvMat cvintrinsic_matrix = cvMat(intrinsic_matrix.d0, intrinsic_matrix.d1, CV_64FC1, intrinsic_matrix.p);
  CvMat cvdistortion_coeffs = cvMat(distortion_coeffs.d0, distortion_coeffs.d1, CV_64FC1, distortion_coeffs.p);
  CvMat *cvR=NULL;
  CvMat *cvP=NULL;
  if (R!=NULL) cvInitMatHeader(cvR, R->d0, R->d1, CV_64FC1, R->p);
  if (P!=NULL) cvInitMatHeader(cvP, P->d0, P->d1, CV_64FC1, P->p);

  cvUndistortPoints(&cvsrc, &cvdst, &cvintrinsic_matrix, &cvdistortion_coeffs, cvR, cvP);
};

inline void cvProjectPoints2(const CvMat* objectPoints, const CvMat* rvec, const CvMat* tvec, const CvMat* cameraMatrix, const CvMat* distCoeffs, CvMat* imagePoints, CvMat* dpdrot=NULL, CvMat* dpdt=NULL, CvMat* dpdf=NULL, CvMat* dpdc=NULL, CvMat* dpddist=NULL)
{std::cout << "TODO" << std::endl;}

inline void cvStereoRectify
(
  const doubleA& cameraMatrix1,
  const doubleA& cameraMatrix2,
  const doubleA& distCoeffs1,
  const doubleA& distCoeffs2,
  CvSize imageSize,
  const doubleA& R, const doubleA& T,
  doubleA& R1, doubleA& R2, doubleA& P1, doubleA& P2, doubleA& Q,
  int flags=CV_CALIB_ZERO_DISPARITY,
  double alpha=0,
  CvSize newImageSize=cvSize(0, 0),
  CvRect* roi1=0,
  CvRect* roi2=0
)
{
  if (cameraMatrix1.N!=9) {np::msg_error(HERE, "cameraMatrix1 is not right");}
  if (cameraMatrix2.N!=9) {np::msg_error(HERE, "cameraMatrix2 is not right");}
  if (distCoeffs1.N!=5) {np::msg_error(HERE, "distCoeff1 is not right");}
  if (distCoeffs2.N!=5) {np::msg_error(HERE, "distCoeff2 is not right");}
  if (R.N!=9) {np::msg_error(HERE, "R is not right");}
  if (T.N!=3) {np::msg_error(HERE, "T is not right");}

  if (R1.N==0) {R1.resize(3,3); R1=0.;}
  if (R2.N==0) {R2.resize(3,3); R2=0.;}
  if (P1.N==0) {P1.resize(3,4); P1=0.;}
  if (P2.N==0) {P2.resize(3,4); P2=0.;}
  if (Q.N==0)  {Q.resize(4,4);  Q =0.;}

  CvMat cvcameraMatrix1 = cvMat(3, 3, CV_64F, cameraMatrix1.p);
  CvMat cvcameraMatrix2 = cvMat(3, 3, CV_64F, cameraMatrix2.p);
  CvMat cvdistCoeffs1   = cvMat(1, 5, CV_64F, distCoeffs1.p);
  CvMat cvdistCoeffs2   = cvMat(1, 5, CV_64F, distCoeffs2.p);
  CvMat cvR  = cvMat(3, 3, CV_64F, R.p);
  CvMat cvT  = cvMat(3, 1, CV_64F, T.p);
  CvMat cvR1 = cvMat(3, 3, CV_64F, R1.p);
  CvMat cvR2 = cvMat(3, 3, CV_64F, R2.p);
  CvMat cvP1 = cvMat(3, 4, CV_64F, P1.p);
  CvMat cvP2 = cvMat(3, 4, CV_64F, P2.p);
  CvMat cvQ  = cvMat(4, 4, CV_64F, Q.p);

  cvStereoRectify(
                  &cvcameraMatrix1, &cvcameraMatrix2,
                  &cvdistCoeffs1, &cvdistCoeffs2,
                  imageSize,
                  &cvR, &cvT, &cvR1, &cvR2, &cvP1, &cvP2, &cvQ,
                  flags, alpha, newImageSize, roi1, roi2
                 );
};

inline void cvInitUndistortRectifyMap
(
  const doubleA& cameraMatrix,
  const doubleA& distCoeffs,
  const doubleA& R,
  const doubleA& newCameraMatrix,
  floatA& map1,
  floatA& map2
)
{
  if (cameraMatrix.N!=9) {np::msg_error(HERE, "cameraMatrix is not right");}
  if (distCoeffs.N!=5) {np::msg_error(HERE, "distCoeff is not right");}
  if (R.N!=9) {np::msg_error(HERE, "R is not right");}
  if (newCameraMatrix.N!=12) {np::msg_error(HERE, "newCameraMatrix is not right");}
  if (map1.N==0) {np::msg_error(HERE, "you need to allocate memory for map1");}
  if (map2.N==0) {np::msg_error(HERE, "you need to allocate memory for map2");}

  CvMat cvcameraMatrix    = cvMat(3, 3, CV_64F, cameraMatrix.p);
  CvMat cvdistCoeffs      = cvMat(1, 5, CV_64F, distCoeffs.p);
  CvMat cvR               = cvMat(3, 3, CV_64F, R.p);
  CvMat cvnewCameraMatrix = cvMat(3, 4, CV_64F, newCameraMatrix.p);
  CvMat cvmap1            = cvMat(map1.d0, map1.d1, CV_32FC1, map1.p);
  CvMat cvmap2            = cvMat(map2.d0, map2.d1, CV_32FC1, map2.p);

  cvInitUndistortRectifyMap(
                            &cvcameraMatrix,
                            &cvdistCoeffs,
                            &cvR,
                            &cvnewCameraMatrix,
                            &cvmap1,
                            &cvmap2
                           );
};

inline void cvRemap
(
  const byteA& src,
  byteA& dst,
  const floatA& mapx,
  const floatA& mapy,
  int flags=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,
  CvScalar fillval=cvScalarAll(0)
)
{
  if (src.d0 != mapx.d0 || src.d1 != mapx.d1)
  {
    std::cout << "src.d*  = " << src.d0 << " " << src.d1 << std::endl;
    std::cout << "mapx.d* = " << mapx.d0 << " " << mapx.d1 << std::endl;
    np::msg_error(HERE, "src image does not have the same size as mapx");
  }
  if (mapx.N != mapy.N)
    np::msg_error(HERE, "mapx and mapy are not of the same size");
  if (src.N != dst.N)
  {
    if (src.nd == 2)
      dst.resize(src.d0, src.d1);
    else
      dst.resize(src.d0, src.d1, src.d2);
  }
  CvMat cvsrc = cvMat(src.d0, src.d1, (src.nd==2?CV_8U:CV_8UC3), src.p);
  CvMat cvdst = cvMat(dst.d0, dst.d1, (dst.nd==2?CV_8U:CV_8UC3), dst.p);
  CvMat cvmapx = cvMat(mapx.d0, mapx.d1, CV_32F, mapx.p);
  CvMat cvmapy = cvMat(mapy.d0, mapy.d1, CV_32F, mapy.p);

  cvRemap(&cvsrc, &cvdst, &cvmapx, &cvmapy, flags, fillval);
};

inline void cvLine(byteA& img, CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int lineType=8, int shift=0)
{
  CvMat cvimg = cvMat(img.d0, img.d1, (img.nd==2?CV_8U:CV_8UC3), img.p);
  cvLine(&cvimg, pt1, pt2, color, thickness, lineType, shift);
};

inline void cvExtractSURF(const byteA& image, floatA& keypoints, floatA& descriptors,  CvSURFParams params=cvSURFParams(300, 0), const byteA* mask=NULL, bool useProvidedKeyPts=false)
{
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq *kp=NULL, *ds=NULL;
  CvMat cvimage= cvMat(image.d0, image.d1, (image.nd==2?CV_8U:CV_8UC3), image.p);
  
  cvExtractSURF(&cvimage, 0, &kp, &ds, storage, params, useProvidedKeyPts);

  uint length_kp= (uint)(kp->elem_size/sizeof(float));
  uint length_ds= (uint)(ds->elem_size/sizeof(float));
  keypoints.resize(kp->total, length_kp);
  descriptors.resize(kp->total, length_ds);

  cv::Mat cvkeypoints(keypoints.d0, length_kp, CV_32F, keypoints.p);
  cv::Mat cvdescriptors(descriptors.d0, length_ds, CV_32F, descriptors.p);

  CvSeqReader seq_reader_kp, seq_reader_ds;
  float *kp_ptr = keypoints.p, *ds_ptr=descriptors.p;
  cvStartReadSeq(kp, &seq_reader_kp);
  cvStartReadSeq(ds, &seq_reader_ds);
  for(int i = 0; i < kp->total; i++ )
  {
      const float* kp_ptr2 = (const float*) seq_reader_kp.ptr;
      CV_NEXT_SEQ_ELEM(seq_reader_kp.seq->elem_size, seq_reader_kp);
      memcpy(kp_ptr, kp_ptr2, length_kp*sizeof(float));
      kp_ptr += length_kp;

      const float* ds_ptr2 = (const float*) seq_reader_ds.ptr;
      CV_NEXT_SEQ_ELEM(seq_reader_ds.seq->elem_size, seq_reader_ds);
      memcpy(ds_ptr, ds_ptr2, length_ds*sizeof(float));
      ds_ptr += length_ds;
  }
//   cvReleaseStorage(&storage);
};

inline void cvCanny(const byteA& image, byteA& edges, double threshold1, double threshold2, int aperture_size=3)
{
  if (image.nd==3)
   np::msg_error(HERE, "input image has to be a grayscale image");

  CvMat cvimage = cvMat(image.d0, image.d1, CV_8U, image.p);
  CvMat cvedges = cvMat(edges.d0, edges.d1, CV_8U, edges.p);
  cvCanny(&cvimage, &cvedges, threshold1, threshold2, aperture_size);
};

inline void cvCircle(byteA& img, const floatA& centers, int radius, CvScalar color, int thickness=1, int lineType=8, int shift=0)
{
  CvMat cvimg = cvMat(img.d0, img.d1, (img.nd==2?CV_8U:CV_8UC3), img.p);
  for (uint i=0; i<centers.d0; i++)
    cvCircle(&cvimg, cvPoint(centers(i,0),centers(i,1)), radius, color, thickness, lineType, shift);
}

namespace np {
class SURF {
public:
  SURF() : surf_(1, 4, 2, false) {};
  void operator()(const byteA& img,
                    std::vector<cv::KeyPoint>& keypoints,
                    std::vector<float>& descriptors,
                    bool useProvidedKeypoints=false)
  {
    cv::Mat cvimg(img.d0, img.d1, (img.nd==2?CV_8U:CV_8UC3), img.p);
    byteA mask(img.d0,img.d1); mask=1;
    cv::Mat cvmask(mask.d0, mask.d1, CV_8U, mask.p);
    surf_(cvimg, cvimg, keypoints, descriptors, useProvidedKeypoints);
  };
  void extract(const byteA& img,
               std::vector<cv::KeyPoint>& keypoints,
               std::vector<float>& descriptors,
               bool useProvidedKeypoints=false)
  {
    cv::Mat cvimg(img.d0, img.d1, (img.nd==2?CV_8U:CV_8UC3), img.p);
    byteA mask(img.d0,img.d1); mask=1;
    cv::Mat cvmask(mask.d0, mask.d1, CV_8U, mask.p);
    surf_(cvimg, cvimg, keypoints, descriptors, useProvidedKeypoints);
  };

  cv::SURF surf_;
};

template <class S, class T>
inline void cvExtractSURF(MT::Array<S>& de, const MT::Array<T>& kp, const byteA& image, np::SURF& surf, uint scale=2)
{
  uint num_kp=kp.d0;
//   uint scale=1;                               // NOTE segfaults if 0 (in cvsurf.cpp:cvExtractSURF())
  std::vector<cv::KeyPoint> kp_temp;
  std::vector<float> de_temp;
  for (uint i=0; i<num_kp; i++)
    kp_temp.push_back(cv::KeyPoint(cv::Point2f(kp(i,0),kp(i,1)),scale, -1, 0, scale));

  np::SURF surf_temp;
  // compute features and copy (Grrrr!) them back
  surf_temp.extract(image, kp_temp, de_temp, true);
  de.resize(de_temp.size()/surf.surf_.descriptorSize(), surf.surf_.descriptorSize());
  memcpy(de.p, &de_temp[0], de_temp.size()*sizeof(float));
};

class flann {
public:
  flann() : index_(NULL) {};
  flann(floatA& val, uint num_trees=4) : index_(NULL) {
    init(val, num_trees);
  };
  flann(doubleA& val, uint num_trees=4) : index_(NULL) {
    np::array2array(data, val);
    init(data, num_trees);
  };
  ~flann() {delete index_;};
  void init(floatA& val, uint num_trees=4) {
    num_trees_=num_trees;
    if (index_!=NULL) reset();
    cv::Mat cvval(val.d0, val.d1, CV_32FC1, val.p);
    index_ = new cv::flann::Index(cvval, cv::flann::KDTreeIndexParams(num_trees_));
  };
  void reset() {delete index_; index_=NULL;};
  void knnSearch(const doubleA& queries, intA& indices, floatA& dists, int knn, cv::flann::SearchParams params=cv::flann::SearchParams(64))
  {
    floatA queries_temp;
    np::array2array(queries_temp, queries);
    knnSearch(queries_temp, indices, dists, knn, params);
  };
  void knnSearch(const floatA& queries, intA& indices, floatA& dists, int knn, cv::flann::SearchParams params=cv::flann::SearchParams(64))
  {
    if (knn <= 0) knn=1;
    if (index_ == NULL) msg_error(HERE, "FLANN has not been initialized");
    if (queries.d0==0) return;
    if (indices.d0!=queries.d0) indices.resize(queries.d0, knn);
    if (dists.d0!=queries.d0) dists.resize(queries.d0, knn);
    cv::Mat cvqueries(queries.d0, queries.d1, CV_32FC1, queries.p);
    cv::Mat cvindices(indices.d0, indices.d1, CV_32S, indices.p);
    cv::Mat cvdists(dists.d0, dists.d1, CV_32F, dists.p);
    index_->knnSearch(cvqueries, cvindices, cvdists, knn, params);
  };
  cv::flann::Index *index_;
  floatA            data;
  uint              num_trees_;
};
};
#endif
