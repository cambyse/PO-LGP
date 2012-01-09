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

#ifdef MT_OPENCV

#include <MT/array.h>
#include <MT/util.h>

#include "nputils.h"
#include "cvutils.h"

#include <MT/vision.h>

#include <opencv/highgui.h>
#include <opencv/ml.h>

//#define USE_MARCS_TRACKER

void np::map_CvMat2byteA(CvMat& cvmat, byteA& bytea)
{
  int N = bytea.N;
  int nd = bytea.nd;
  int d2 = bytea.d2;
  int w = bytea.d1, h = bytea.d0;

  if (N == 0)
    msg_error(HERE, "cannot map CvMat to byteA, since latter one is empty");
  if (nd != 2 && nd != 3)
    msg_error(HERE, "only 2D and 3D arrays, please");

  // check number of channels
  int type = CV_8UC1;
  if (nd == 3 && d2 == 3)
    type = CV_8UC3;

  cvmat = cvMat(h, w, type, bytea.p);
}

void np::copy_iplimage2byteA(byteA& img, IplImage& img_cv)
{
  int num_channels = img_cv.nChannels;
  int height = img_cv.height;
  int width = img_cv.width;
  int counter = 0;

  // padding for openCV regulation, that images need to be multiples of 4 ...
  int padding = img_cv.widthStep - (num_channels * width);
  if (num_channels == 1)
  {
    img.resize(height, width);
    int counter = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
          img(y,x) = img_cv.imageData[counter++];
        counter += padding;
    }
  }
  else
  {
    img.resize(height, width, num_channels);
    if (img_cv.dataOrder == 0)
    {
        counter = 0;
        // determine channel order (RGB or BGR)
        bool is_RGB = (strcmp("RGB", img_cv.channelSeq) == 0);
        if(is_RGB)
        {
          int counter = 0;
          for (int y = 0; y < height; y++)
          {
              for (int x = 0; x < width; x++)
              {
                img(y,x,0) = img_cv.imageData[counter++];
                img(y,x,1) = img_cv.imageData[counter++];
                img(y,x,2) = img_cv.imageData[counter++];
              }
              counter += padding;
          }
        }
        else // "is_BGR"
        {
          int counter = 0;
          for (int y = 0; y < height; y++)
          {
              for (int x = 0; x < width; x++)
              {
                img(y,x,0) = img_cv.imageData[counter+2];
                img(y,x,1) = img_cv.imageData[counter+1];
                img(y,x,2) = img_cv.imageData[counter];
                counter += 3;
              }
              counter += padding;
          }
        }
    }
    else
        msg_error(HERE, "TBD - conversion in case of separate color channels");
  }
}

char np::display(const char* window, byteA& img, int cvwaitkey, bool rgb2bgr)
{
  CvMat img_cv;
  byteA helper;
  int num_channels = img.d2;
  if (rgb2bgr && (num_channels == 3))
  {
    helper.resize(img.d0, img.d1, img.d2);
    for (uint i = 0; i < img.N; i+=3)
    {
      helper.p[i] = img.p[i+2];
      helper.p[i+1] = img.p[i+1];
      helper.p[i+2] = img.p[i];
    }
    map_CvMat2byteA(img_cv, helper);
  }
  else
    map_CvMat2byteA(img_cv, img);

  cvShowImage(window, &img_cv);
  return cvWaitKey(cvwaitkey);
}

void np::save_image(const byteA& img, const char* filename)
{
  CvMat img_cv = cvMat(
                       img.d0, img.d1,
                       (img.d2 == 3 ? CV_8UC3 : CV_8UC1),
                       img.p
                      );
   cvSaveImage(filename, &img_cv);
}

void np::load_image(byteA& img, const char* filename, int is_color)
{
  IplImage* img_cv = cvLoadImage(filename, is_color);
  if (img_cv == NULL)
  {
    std::ostringstream oss;
    oss << "Cannot load image [" << filename << "]";
    msg_error(HERE, oss.str().c_str());
  }
  copy_iplimage2byteA(img, *img_cv);
  cvReleaseImage(&img_cv);
}

void np::convert(byteA& out, byteA& in, int mode)
{
 int num_channels = (in.nd == 3 ? CV_8UC3 : CV_8UC1);
 out.resize(in.N);
 if (in.nd == 3)
   out.reshape(in.d0, in.d1, in.d2);
 else
   out.reshape(in.d0, in.d1);

 CvMat incv = cvMat(in.d0, in.d1, num_channels, in.p);
 CvMat outcv = cvMat(out.d0, out.d1, (out.nd == 3 ? CV_8UC3 : CV_8UC1), out.p);
 cvCvtColor(&incv, &outcv, mode);
}

/*! \brief KMeans clustering algorithm
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
int np::kmeans
(
  floatA& clusters,
  intA& labels,
  double& compactness,
  const floatA& samples,
  uint num_clusters,
  double termcrit_d,
  uint termcrit_i,
  int attempts,
  bool use_labels
)
{
  uint num_samples_dim = (samples.d1 > 0 ? samples.d1 : 1);
  uint num_samples = samples.d0;
  if (num_clusters < 1)
    np::msg_error(HERE, "Set num_clusters to a value >= 1");
  if (samples.N == 0)
    np::msg_error(HERE, "Data matrix <samples> must not be empty.");
  if (use_labels && labels.N != samples.d0)
    np::msg_error(HERE, "Number of labels does not equal number of data samples.");

  // map samples to openCV data structure
  clusters.resize(num_clusters, num_samples_dim);
  CvMat clusters_cv = cvMat(clusters.d0, clusters.d1, CV_32FC1, clusters.p);

  // map clusters to openCV data structure
  CvMat samples_cv = cvMat(samples.d0, samples.d1, CV_32FC1, samples.p);

  // use initially randomized labels for kmeans
  MT::rnd.clockSeed();
  labels.resize(num_samples);
  rndUniform(labels,0,num_clusters,false);
  CvMat labels_cv = cvMat(labels.d0, 1, CV_32SC1, labels.p);
  CvTermCriteria tc;
  tc = cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, termcrit_i, termcrit_d);

  CvRNG rng = cvRNG(0xffffffff);
  int status = cvKMeans2(
                         &samples_cv,
                         (int) num_clusters,
                         &labels_cv,
                         tc,
                         attempts,
                         &rng,
                         CV_KMEANS_USE_INITIAL_LABELS,
                         &clusters_cv,
                         &compactness
                        );

  return status;
}

/*! \brief Trains a Boosted Decision Forest
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
CvBoost* np::boostDTrees_train
(
 const floatA& data,
 const floatA& responses,
 int boost_type,
 uint num_classes,
 int weak_count,
 double weight_trim_rate,
 int max_depth,
 bool use_surrogates,
 const float* priors
)
{
  uint num_samples = data.d0;
  uint num_dim = data.d1;
  uint var_count = num_dim; // TODO remove this
  CvMat *var_type = NULL;
  CvBoost *boost = new CvBoost();

  if (num_classes < 2)
    np::msg_error("only two- or multi-class problems allowed.");

//   if (num_classes == 2)
//   {
//     intA responses_temp;
//     array2array(responses_temp, responses);
//     CvMat d = cvMat(data.d0, data.d1, CV_32FC1, data.p);
//     CvMat r = cvMat(responses_temp.d0, 1, CV_32SC1, responses_temp.p);
// 
//     // 2. create type mask
//     var_type = cvCreateMat(num_dim + 1, 1, CV_8U);
//     cvSet(var_type, cvScalarAll(CV_VAR_ORDERED));
//     // the last indicator variable, as well
//     // as the new (binary) response are categorical
//     cvSetReal1D(var_type, var_count, CV_VAR_CATEGORICAL);
// 
//     // debug
//     floatA dd; dd.referTo(d.data.fl, d.rows*d.cols); dd.reshape(d.rows, d.cols);
//     intA rr; rr.referTo(r.data.i, r.rows*r.cols); rr.reshape(r.rows*r.cols);
//     for (uint x = 0; x < dd.d0; x++)
//       std::cout << dd[x] << " - " << (int) rr(x) << std::endl;
// 
//     // 3. train classifier
//     std::cout << "Training the classifier (may take a few minutes)..." << std::flush;
//     boost->train(&d, CV_ROW_SAMPLE, &r, 0, 0, var_type, 0,
//         CvBoostParams(CvBoost::REAL, 100, 0.95, 5, false, 0 ));
// 
//     std::cout << "Number of trees: " << boost->get_weak_predictors()->total << std::endl;
//   }
//   else
//   {
    // multi-class case
    //  "unrolling" the data, cf. letter-recognition.cpp in opencv-sample-folder.
    CvMat *d = NULL;
    CvMat *r = NULL;
    d = cvCreateMat(num_samples*num_classes, num_dim+1, CV_32FC1);
    r = cvCreateMat(num_samples*num_classes, 1, CV_32SC1);
    for(uint s = 0; s < num_samples; s++)
    {
//       float* data_row = (float*)(data.p + data.d1*s);
      for (uint c = 0; c < num_classes; c++)
      {
        float* d_row = (float*)(d->data.ptr + d->step*(s*num_classes+c));
        for(uint k = 0; k < num_dim; k++ )
            d_row[k] = data(s,k);
        d_row[num_dim] = (float) c;
        r->data.i[s*num_classes+c] = (responses(s) == c);
      }
    }

//     // debug
//     floatA dd; dd.referTo(d->data.fl, d->rows*d->cols); dd.reshape(d->rows, d->cols);
//     intA rr; rr.referTo(r->data.i, r->rows*r->cols); rr.reshape(r->rows*r->cols);
//     for (uint x = 0; x < dd.d0; x++)
//       std::cout << dd[x] << " - " << (int) rr(x) << std::endl;

    // 2. create type mask
    var_type = cvCreateMat(num_dim + 2, 1, CV_8U);
    cvSet(var_type, cvScalarAll(CV_VAR_ORDERED));
    // the last indicator variable, as well
    // as the new (binary) response are categorical
    cvSetReal1D(var_type, var_count, CV_VAR_CATEGORICAL);
    cvSetReal1D(var_type, var_count+1, CV_VAR_CATEGORICAL);

    // 3. train classifier
    std::cout << "Training the classifier (may take a moment or two) ..." << std::flush;
    boost->train(d, CV_ROW_SAMPLE, r, 0, 0, var_type, 0,
        CvBoostParams(boost_type, weak_count, weight_trim_rate, max_depth, use_surrogates, priors),
        false);
    std::cout << " done!" << std::endl;

    cvReleaseMat(&d);
    cvReleaseMat(&r);
//   }

  cvReleaseMat(&var_type);

  return boost;
};

/*! \brief Prediction with Boosted Decision Forest
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
int np::boostDTrees_predict
(
 floatA& responses,
 floatA& sums,
 CvBoost* boost,
 floatA& data,
 uint num_classes
)
{
  uint num_dim = data.d1;
  uint num_samples = data.d0;
//   double reco = 0;

  // TODO replace these
  uint var_count = num_dim;
  uint nsamples_all = num_samples;
  uint class_count = num_classes;

  CvMat *temp_sample = cvCreateMat( 1, var_count + 1, CV_32F );
  CvMat *weak_responses = cvCreateMat(1, boost->get_weak_predictors()->total, CV_32F);
  CvMat data_cv = cvMat(data.d0, data.d1, CV_32FC1, data.p);
  responses.resize(num_samples);
  sums.resize(num_samples);
  sums = 0;

  // compute prediction error on train and test data
  for(uint i = 0; i < nsamples_all; i++)
  {
    int best_class = 0;
    double max_sum = -DBL_MAX;
//     double r;
    CvMat sample;
    cvGetRow(&data_cv, &sample, i);
    for(uint k = 0; k < var_count; k++ )
      temp_sample->data.fl[k] = sample.data.fl[k];

    for(uint j = 0; j < class_count; j++)
    {
      temp_sample->data.fl[var_count] = (float)j;
      boost->predict(temp_sample, 0, weak_responses);
      double sum = cvSum(weak_responses).val[0];
      if( max_sum < sum )
      {
        max_sum = sum;
        sums(i) = sum;
        best_class = j;
      }
    }

//     if (true_responses != NULL)
//     {
//        r = fabs(best_class - (*true_responses)(i)) < FLT_EPSILON ? 1 : 0;
// //        std::cout << "best_class = " << best_class << " true = " << (*true_responses)(i) << std::endl;
//        reco += r;
//     }
    responses(i) = best_class;
  }

//   if (true_responses != NULL)
//   {
//     std::cout << "number of samples: " << data.d0 << std::endl;
//     std::cout << "number of recognitions: " << reco << std::endl;
//     reco *= (100/num_samples);
//     std::cout << "Recognition rate: " << reco << "% ..." << std::endl;
//   }
//   std::cout << "Number of trees: " << boost->get_weak_predictors()->total << std::endl;

  cvReleaseMat(&temp_sample);
  cvReleaseMat(&weak_responses);

  return 0;
}

/*! \brief Remaps image pixels and interpolates
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
int np::remap(byteA& rimg, const byteA& img, const floatA& mapx, const floatA& mapy)
{
  if (img.nd == 2)
    rimg.resize(img.d0, img.d1);
  else
    rimg.resize(img.d0, img.d1, img.d2);

  CvMat cvimg  = cvMat(img.d0, img.d1, (img.nd == 2? CV_8U : CV_8UC3), img.p);
  CvMat cvrimg = cvMat(img.d0, img.d1, (img.nd == 2? CV_8U : CV_8UC3), rimg.p);
  CvMat cvmapx = cvMat(mapx.d0, mapx.d1, CV_32F, mapx.p);
  CvMat cvmapy = cvMat(mapy.d0, mapy.d1, CV_32F, mapy.p);

  cvRemap(&cvimg,&cvrimg,&cvmapx,&cvmapy);

  return 0;
}

/*! \brief Tracks colored ball
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
void np::track_ball
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
 bool is_hsv,
 bool draw
)
{
  if (image.N == 0 || image.d2 != 3)
    np::msg_error("image is empty or not an color image");

  // 
//  CvMemStorage *storage = cvCreateMemStorage(0); //needed for Hough circles
  CvSize size = cvSize(image.d1,image.d0);
  IplImage *hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3);
  IplImage *thresholded  = cvCreateImage(size, IPL_DEPTH_8U, 1);
  CvMat image_cv = cvMat(image.d0, image.d1, CV_8UC3, image.p);
  if (!is_hsv)
  {
//    hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
    cvCvtColor(&image_cv, hsv_frame, CV_RGB2HSV);
    image_cv = cvMat(hsv_frame->height, hsv_frame->width, CV_8UC3, hsv_frame->imageData);
  }

  // apply thresholding on HSV values
  CvScalar hsv_min = cvScalar(min_h, min_s, min_v, 0);
  CvScalar hsv_max = cvScalar(max_h, max_s, max_v, 0);
//  cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
  cvInRangeS(&image_cv, hsv_min, hsv_max, thresholded);

  // apply hough detector (works better with some smoothing of the image)
  cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 3, 3);

#if defined(DEBUG)
   cvShowImage("asasd", thresholded);
   cvWaitKey(10);
#endif
////  CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 100, 60, 25, 200);
////  CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 80, 20, 25, 50);
////   CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 1, thresholded->height/4, 10, 10, 1, 100);
//  CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height, 50, 50, 0, 100);

  double sumx=0., sumy=0.;
  uint p=0, count=0;
  for (int y=0;y<thresholded->height;y++)
    for ( int x=0;x<thresholded->width;x++)
    {
      p=y*thresholded->width+x;
      if (thresholded->imageData[p]>0)
      {
        sumx+=x;
        sumy+=y;
        count++;
      }
    }
  if (count>0)
  {
    sumx/=(double)count;
    sumy/=(double)count;
    // create a bounding box around preliminarily found center
    // and recompute the center
    int minsum= (sumx < sumy ? sumx : sumy);
    int offset =  (minsum > 100 ? 100 : (minsum-1));
    double sumx2=0., sumy2=0.;
    int count2=0;
    if (sumy-offset < 0 || sumx-offset < 0)
      msg_error(HERE, "points outside image canvas - not good at all!!");
    for (int y=sumy-offset; y<sumy+offset && y<thresholded->height;y++)
      for (int x=sumx-offset; x<sumx+offset && x<thresholded->width;x++)
      {
        p=y*thresholded->width+x;
        if (thresholded->imageData[p]>0)
        {
          sumx2+=x;
          sumy2+=y;
          count2++;
        }
      }
    if (count2>0)
    {
      sumx=sumx2/(double)count2;
      sumy=sumy2/(double)count2;
    }

    centers.resize(1, 2);
    r.resize(1);
    centers(0,1) = sumy; centers(0,0) = sumx;
    r=offset;
  }
  else
  {
    std::cout << "found nothing" << std::endl;
  }

#if defined(DEBUG)
  std::cout << "sumy="<<sumy<<", sumx="<<sumx<<std::endl;
  cvCircle(&image_cv, cvPoint(sumx,sumy),  3, CV_RGB(200,200,200), -1, 8, 0 );
  cvCircle(&image_cv, cvPoint(sumx,sumy), 100, CV_RGB(128,128,128), 3, 8, 0 );
  cvShowImage("circle", &image_cv);
  cvWaitKey(10);
#endif

  // return output
//  centers.resize(circles->total, 2);
//  r.resize(circles->total);
//  for (int i = 0; i < circles->total; i++)
//  {
//    float* p = (float*) cvGetSeqElem(circles, i);
//    centers(i,0) = p[0];
//    centers(i,1) = p[1];
//    r(i) = p[2];
//    if (draw)
//    {
//      cvCircle(&image_cv, cvPoint(cvRound(p[0]),cvRound(p[1])),
//                 3, CV_RGB(200,200,200), -1, 8, 0 );
//      cvCircle(&image_cv, cvPoint(cvRound(p[0]),cvRound(p[1])),
//               cvRound(p[2]), CV_RGB(128,128,128), 3, 8, 0 );
//    }
//  }

//  cvClearSeq(circles);
  cvReleaseImage(&hsv_frame);
  cvReleaseImage(&thresholded);
//  cvReleaseMemStorage(&storage);
};

/*! \brief Tracks red ball
 *
 *  Tracks any reasonably red ball by thresholding HSV space.
 *
 *  @see track_ball()
 *  @param centers  image coordinates of ball center(s), size: (N,2), N - no. balls
 *  @param r        radiae of found circles, size: (N)
 *  @param image    RGB input image, size: (y,x,3)
 *  @param draw     draw centers and circle into input image
 *
 *  @return cf. params centers, r, image
 */
void np::track_red_ball(floatA& centers, floatA& r, byteA& image, bool is_hsv, bool draw)
{
//  track_ball(centers, r, image, 0, 120, 120, 10, 256, 256);
//  track_ball(centers, r, image, 0, 150, 150, 20, 256, 256, draw);
  track_ball(centers, r, image, 0, 120, 120, 30, 256, 256, is_hsv, false);
}

/*! \brief Tracks green ball
 *
 *  Tracks any reasonably green ball by thresholding HSV space.
 *
 *  @see track_ball()
 *  @param centers  image coordinates of ball center(s), size: (N,2), N - no. balls
 *  @param r        radiae of found circles, size: (N)
 *  @param image    RGB input image, size: (y,x,3)
 *  @param draw     draw centers and circle into input image
 *
 *  @return cf. params centers, r, image
 */
void np::track_green_ball(floatA& centers, floatA& r, byteA& image, bool is_hsv, bool draw)
{track_ball(centers, r, image, 60, 0, 0, 90, 256, 256, is_hsv, false);}

/*! \brief Tracks blue ball
 *
 *  Tracks any reasonably blue ball by thresholding HSV space.
 *
 *  @see track_ball()
 *  @param centers  image coordinates of ball center(s), size: (N,2), N - no. balls
 *  @param r        radiae of found circles, size: (N)
 *  @param image    RGB input image, size: (y,x,3)
 *  @param draw     draw centers and circle into input image
 *
 *  @return cf. params centers, r, image
 */
void np::track_blue_ball(floatA& centers, floatA& r, byteA& image, bool is_hsv, bool draw)
{track_ball(centers, r, image, 120, 120, 120, 150, 256, 256, is_hsv, false);}

/*! \brief Tracks a red, a green, and a blue ball
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
bool np::track_RGB_balls
(
  byteA& left,
  byteA& right,
  floatA& centers_left,
  floatA& centers_right,
  floatA& rad_left,
  floatA& rad_right
)
{
    byteA left_hsv, right_hsv;
    floatA centers_temp, rad_temp;
    uint num_balls_found = 0;
    centers_left.clear();
    centers_left.reshape(0,2);
    centers_right.clear();
    centers_right.reshape(0,2);
    rad_left.clear();
    rad_right.clear();

    // determine left/right image coordinates of calibration object (II)
    np::convert(left_hsv, left, CV_RGB2HSV);
    np::convert(right_hsv, right, CV_RGB2HSV);

#ifdef USE_MARCS_TRACKER
    doubleA cT;
    getHsvCenter(cT,left,2,ARRAY<float>(.0,1.,1.),ARRAY<float>(.2,.5,.5));
    cT.reshape(cT.N/2,2);
    array2array(centers_temp, cT);
    rad_temp.resize(cT.d0);
    rad_temp=10;
#else
    np::track_red_ball(centers_temp, rad_temp, left_hsv, true, true);
#endif
    if (centers_temp.d0 > 0)
    {
      np::draw_circles(left, centers_temp, rad_temp);
      centers_left.append(centers_temp);
      rad_left.append(rad_temp);
      num_balls_found+=centers_temp.d0;
    }

#ifdef USE_MARCS_TRACKER
    getHsvCenter(cT,left,2,ARRAY<float>(1./3.,1.,1.),ARRAY<float>(.2,.5,.5));
    cT.reshape(cT.N/2,2);
    array2array(centers_temp, cT);
    rad_temp.resize(cT.d0);
    rad_temp=10;
#else
    np::track_green_ball(centers_temp, rad_temp, left_hsv, true, true);
#endif
    if (centers_temp.d0 > 0)
    {
      np::draw_circles(left, centers_temp, rad_temp);
      centers_left.append(centers_temp);
      rad_left.append(rad_temp);
      num_balls_found+=centers_temp.d0;
    }

#ifdef USE_MARCS_TRACKER
    getHsvCenter(cT,left,2,ARRAY<float>(2./3.,1.,1.),ARRAY<float>(.2,.5,.5));
    cT.reshape(cT.N/2,2);
    array2array(centers_temp, cT);
    rad_temp.resize(cT.d0);
    rad_temp=10;
#else
    np::track_blue_ball(centers_temp, rad_temp, left_hsv, true, true);
#endif
    if (centers_temp.d0 > 0)
    {
      np::draw_circles(left, centers_temp, rad_temp);
      centers_left.append(centers_temp);
      rad_left.append(rad_temp);
      num_balls_found+=centers_temp.d0;
    }

    // right side
#ifdef USE_MARCS_TRACKER
    getHsvCenter(cT,right,2,ARRAY<float>(.0,1.,1.),ARRAY<float>(.2,.5,.5));
    cT.reshape(cT.N/2,2);
    array2array(centers_temp, cT);
    rad_temp.resize(cT.d0);
    rad_temp=10;
#else
    np::track_red_ball(centers_temp, rad_temp, right_hsv, true, true);
#endif
    if (centers_temp.d0 > 0)
    {
      np::draw_circles(right, centers_temp, rad_temp);
      centers_right.append(centers_temp);
      rad_right.append(rad_temp);
      num_balls_found+=centers_temp.d0;
    }

#ifdef USE_MARCS_TRACKER
    getHsvCenter(cT,right,2,ARRAY<float>(1./3.,1.,1.),ARRAY<float>(.2,.5,.5));
    cT.reshape(cT.N/2,2);
    array2array(centers_temp, cT);
    rad_temp.resize(cT.d0);
    rad_temp=10;
#else
    np::track_green_ball(centers_temp, rad_temp, right_hsv, true, true);
#endif
    if (centers_temp.d0 > 0)
    {
      np::draw_circles(right, centers_temp, rad_temp);
      centers_right.append(centers_temp);
      rad_right.append(rad_temp);
      num_balls_found+=centers_temp.d0;
    }
#ifdef USE_MARCS_TRACKER
    getHsvCenter(cT,right,2,ARRAY<float>(2./3.,1.,1.),ARRAY<float>(.2,.5,.5));
    cT.reshape(cT.N/2,2);
    array2array(centers_temp, cT);
    rad_temp.resize(cT.d0);
    rad_temp=10;
#else
    np::track_blue_ball(centers_temp, rad_temp, right_hsv, true, true);
#endif
    if (centers_temp.d0 > 0)
    {
      np::draw_circles(right, centers_temp, rad_temp);
      centers_right.append(centers_temp);
      rad_right.append(rad_temp);
      num_balls_found+=centers_temp.d0;
    }

    std::cout << "num_balls_found = " << num_balls_found << std::endl;
    return (num_balls_found == 6);
};


/*! \brief Detects the chessboard corners
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
int np::find_chessboard_corners(floatA& corners, byteA& image, uint num_x, uint num_y, bool draw)
{
  if (num_x < 2 || num_y < 2)
    np::msg_error(HERE, "np::find_chessboard_corners: make sure that num_x >= 2 and num_y >= 2");

  int num_corners, found = 0;
  std::vector<CvPoint2D32f> cv_corners(num_x*num_y);
  CvMat cv_image;
  map_CvMat2byteA(cv_image, image);

  found = cvFindChessboardCorners(
                  &cv_image,
                  cvSize(num_x, num_y),
                  &cv_corners[0],
                  &num_corners,
                  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE
                );
//  std::cout << "num_x*num_y = " << num_x*num_y << std::endl;
//  std::cout << "num_corners = " << num_corners << std::endl;
//  std::cout << "found       = " << found << std::endl;

  // if chessboard pattern has been found, improve positions to subpixel accuracy
  if (found == 1)
  {
    CvMat* temp = cvCreateMat(image.d0, image.d1, CV_8U);
    cvCvtColor(&cv_image, temp, CV_RGB2GRAY);
    cvFindCornerSubPix(
                temp,
                &cv_corners[0],
                num_corners,
                cvSize(5, 5),                /* half of size of search window */
                cvSize(-1,-1),   /* half of size of the dead region (not used)*/
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01)
              );
    cvReleaseMat(&temp);
  }

  // draw chessboard pattern into input image
  if (draw && found == 1)
    cvDrawChessboardCorners(
                &cv_image,
                cvSize(num_x, num_y),
                &cv_corners[0],
                num_corners,
                CV_CALIB_CB_ADAPTIVE_THRESH
              );

  // return coordinates of found corners
  if (found == 1)
  {
    corners.resize(num_corners, 2);
    for (int i = 0; i < num_corners; i++)
    {
      corners(i,0) = cv_corners[i].x;
      corners(i,1) = cv_corners[i].y;
    }
  }
  else
    corners.clear();

  return (found == 1);
};

void np::stereo_calibration
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
  bool use_guess
)
{
  // resize output and wrap it for OpenCV
  if (Kl.N != 9) Kl.resize(3,3); 
  if (Kr.N != 9) Kr.resize(3,3);
  if (dl.N != 5) dl.resize(5); 
  if (dr.N != 5) dr.resize(5);
  R.resize(3,3); T.resize(3);
  E.resize(3,3), F.resize(3,3);
  Pl.resize(3,4); Pr.resize(3,4);
  Rl.resize(3,3); Rr.resize(3,3);
  Q.resize(4,4);
  if (!use_guess)
  {
    dl = 0; dr = 0;
    Kl.setDiag(1); Kr.setDiag(1);
  }

  CvMat cvKl; np::map_cvmat2mtarray(cvKl, Kl);
  CvMat cvKr; np::map_cvmat2mtarray(cvKr, Kr);
  CvMat cvdl; np::map_cvmat2mtarray(cvdl, dl);
  CvMat cvdr; np::map_cvmat2mtarray(cvdr, dr);
  CvMat cvR;  np::map_cvmat2mtarray(cvR,  R);
  CvMat cvT;  np::map_cvmat2mtarray(cvT,  T);
  CvMat cvE;  np::map_cvmat2mtarray(cvE,  E);
  CvMat cvF;  np::map_cvmat2mtarray(cvF,  F);
  CvMat cvPl; np::map_cvmat2mtarray(cvPl, Pl);
  CvMat cvPr; np::map_cvmat2mtarray(cvPr, Pr);
  CvMat cvRl; np::map_cvmat2mtarray(cvRl, Rl);
  CvMat cvRr; np::map_cvmat2mtarray(cvRr, Rr);
  CvMat cvQ;  np::map_cvmat2mtarray(cvQ,  Q);
//  cvSetIdentity(&cvKl); cvSetIdentity(&cvKr);
//  cvZero(&cvdl); cvZero(&cvdr);

  CvMat cvcleft;  np::map_cvmat2mtarray(cvcleft,  cleft);
  CvMat cvcright; np::map_cvmat2mtarray(cvcright, cright);
  CvMat cvcworld; np::map_cvmat2mtarray(cvcworld, cworld);

  // number of corners per view (needed by cvStereoCalibrate())
  CvMat *pointCounts = cvCreateMat(num_views, 1, CV_32SC1);
  cvSet(pointCounts, cvRealScalar(nx*ny), NULL);

  std::cout << "Kl == " << std::endl << Kl << std::endl;
  std::cout << "Kr == " << std::endl << Kr << std::endl;
  std::cout << "dl == " << dl << std::endl;
  std::cout << "dr == " << dr << std::endl;
  // actual stereo calibration
  cvStereoCalibrate(
                    &cvcworld,
                    &cvcleft,
                    &cvcright,
                    pointCounts,
                    &cvKl, &cvdl, &cvKr, &cvdr,
                    cvSize(image_width, image_height),
                    &cvR, &cvT, &cvE, &cvF,
                    cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-50),
                    CV_CALIB_SAME_FOCAL_LENGTH+CV_CALIB_FIX_K3+(use_guess ? CV_CALIB_USE_INTRINSIC_GUESS : 0)
                   );
  cvReleaseMat(&pointCounts);

  // stereo rectification
  cvStereoRectify(
                  &cvKl, &cvKr,
                  &cvdl, &cvdr,
                  cvSize(image_width, image_height),
                  &cvR, &cvT,
                  &cvRl, &cvRr,
                  &cvPl, &cvPr,
                  &cvQ,
                  0 /* CV_CALIB_ZERO_DISPARITY */
                 );
};

void np::stereo_calibration_save
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
)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  Kl.writeTagged(os, "doubleA_Kl", false);
  Kr.writeTagged(os, "doubleA_Kr", false);
  dl.writeTagged(os, "doubleA_dl", false);
  dr.writeTagged(os, "doubleA_dr", false);
  R.writeTagged(os,  "doubleA_R",  false);
  T.writeTagged(os,  "doubleA_T",  false);
  E.writeTagged(os,  "doubleA_E",  false);
  F.writeTagged(os,  "doubleA_F",  false);
  Pl.writeTagged(os, "doubleA_Pl", false);
  Pr.writeTagged(os, "doubleA_Pr", false);
  Rl.writeTagged(os, "doubleA_Rl", false);
  Rr.writeTagged(os, "doubleA_Rr", false);
  Q.writeTagged(os,  "doubleA_Q",  false);
  cleft.writeTagged(os,  "floatA_cleft",  false);
  cright.writeTagged(os, "floatA_cright", false);
  cworld.writeTagged(os, "floatA_cworld", false);

  os.close();
};


// "glue" imgL and imgR together
void np::merge(byteA& imgLR, const byteA& imgL, const byteA& imgR)
{
  if (imgL.nd != imgR.nd)
    np::msg_error(HERE, "images have to be both color or both gray, no mixing please!");
  if (imgL.d0 != imgR.d0)
    np::msg_error(HERE, "images have to have the same height");
  bool is_color = (imgL.nd==3);

  if (imgLR.d0 != imgL.d0 || imgLR.d1 != imgL.d1)
    imgLR.resize(imgL.d0, imgL.d1+imgR.d1, 3);

  CvMat cvtemp;
  CvMat cvimgL = cvMat(imgL.d0, imgL.d1, (is_color?CV_8UC3:CV_8U), imgL.p);
  CvMat cvimgR = cvMat(imgR.d0, imgR.d1, (is_color?CV_8UC3:CV_8U), imgR.p);
  CvMat cvimgLR = cvMat(imgLR.d0, imgLR.d1, CV_8UC3, imgLR.p);
  cvGetCols(&cvimgLR, &cvtemp, 0, imgL.d1);
  if (is_color)
    cvCvtColor(&cvimgL, &cvtemp, CV_RGB2BGR);
  else
    cvCvtColor(&cvimgL, &cvtemp, CV_GRAY2BGR);

  cvGetCols(&cvimgLR, &cvtemp, imgL.d1, imgL.d1+imgR.d1);
  if (is_color)
    cvCvtColor(&cvimgR, &cvtemp, CV_RGB2BGR);
  else
    cvCvtColor(&cvimgR, &cvtemp, CV_GRAY2BGR);
};

#endif
