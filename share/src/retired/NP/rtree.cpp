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

#include <fstream>
#include "rtree.h"
#include "nputils.h"
#include <opencv/cv.h>
#include <opencv/ml.h>

/** @brief Trains a Random Forest
 *
 *  For details search OpenCV docs for "CvRTrees::train".
 *
 *  @param data       M-by-D matrix, row-wise samples
 *  @param responses  M-by-1 vector with discrete (cls.) or cont. (reg.) values
 *  @param fraction   percentage of total samples used to train a single RT, range: (0,1]
 *  @param num_trees  number of trees to generate, > 0
 *  @param is_regression    if true train regression, else train classification
 *
 *  @return trained instance of an Random Forest
 */
CvRTrees* np::rtree_train
(
 const floatA& data,
 const floatA& responses,
 float fraction,
 uint  num_trees,
 bool  is_regression
)
{
  CvRTrees* forest       = new CvRTrees();
  CvMat data_cv;
  CvMat responses_cv;
  CvMat* var_type_cv     = 0;
  CvMat* sample_idx_cv   = 0;
  uint num_samples       = data.d0;
  uint num_train_samples = (int)((fraction > 0 ? fraction : 0.8)*num_samples);

  if (num_samples <= 0)
    np::msg_error(HERE, "data set must not be empty");
  if (responses.d0 != num_samples)
    np::msg_error(HERE, "number of samples and responses has to be equal");
  if (responses.d0 != responses.N)
    np::msg_error(HERE, "parameter <responses> has to be a M-by-1 matrix/vector");

  // wrap input data
  data_cv = cvMat(data.d0, data.d1, CV_32FC1, data.p);
  responses_cv = cvMat(responses.d0, 1, CV_32FC1, responses.p);

  // 1. create type mask
  var_type_cv = cvCreateMat(data_cv.cols + 1, 1, CV_8U);
  cvSet(var_type_cv, cvScalarAll((is_regression ? CV_VAR_NUMERICAL : CV_VAR_CATEGORICAL)));
//  cvSetReal1D(var_type_cv, data_cv->cols, CV_VAR_CATEGORICAL); // TODO muss das hier wirklich sein?

  // 2. create sample idx (select first 80% samples for training, and deselect 
  //    remaining 20% [cf. OpenCV letter_recog example])
  sample_idx_cv = cvCreateMat(1, num_samples, CV_8UC1 );
  {
    CvMat mat;
    cvGetCols(sample_idx_cv, &mat, 0, num_train_samples);
    cvSet(&mat, cvRealScalar(1));

    cvGetCols(sample_idx_cv, &mat, num_train_samples, num_samples);
    cvSetZero(&mat);
  }

  // 3. train Random Trees
  forest->train(
                &data_cv, 
                CV_ROW_SAMPLE, 
                &responses_cv,
                0, 
                sample_idx_cv, 
                var_type_cv, 
                0, 
//                CvRTParams(10,10,0,false,15,0,true,4,100,0.01f,CV_TERMCRIT_ITER) // TODO let the user choose RT params
                CvRTParams(10,10,0,false,15,0,false,0,num_trees,0.01f,CV_TERMCRIT_ITER) // TODO let the user choose RT params
               );

//  CvRTParams( int _max_depth, int _min_sample_count,
//                float _regression_accuracy, bool _use_surrogates,
//                int _max_categories, const float* _priors,
//                bool _calc_var_importance,
//                int _nactive_vars, int max_tree_count,
//                float forest_accuracy, int termcrit_type );

  return forest;
}

/** @brief Predicts the output for the input samples
 *
 *  This method returns the cumulative result from all the trees in the forest 
 *  (the class that receives the majority of voices, or the mean of the 
 *  regression function estimates).
 *
 *  For further details search OpenCV docs for "CvRTrees::predict".
 *
 *  @see rtree_train()
 *  @param p        predictions, size: (N,1)
 *  @param forest   pointer to a trained random forest
 *  @param samples  data matrix, size: (N, M)
 *
 *  @return cf. parameter p
 */
void np::rtree_predict(floatA& p, const CvRTrees* forest, const floatA& samples)
{
  uint num_samples = samples.d0;
  doubleA p_t;
  CvMat samples_cv = cvMat(samples.d0, samples.d1, CV_32FC1, samples.p);
  CvMat row;

  if (num_samples <= 0)
    np::msg_error(HERE, "data set must not be empty");

  p_t.resize(num_samples);
  p_t = 0;

  // 
  samples_cv = cvMat(samples.d0, samples.d1, CV_32FC1, samples.p);
  for (uint i = 0; i < num_samples; i++)
  {
    cvGetRow(&samples_cv, &row, i);
    p_t(i) = forest->predict(&row);
  }
  
  array2array(p,p_t);
}

/** @brief Saves a Random Forest from file
 *
 *  Wraps around CvStatModel::save().
 *  "The method save stores the complete model state to the specified XML or 
 *  YAML file with the specified name or default name (that depends on the 
 *  particular class)."
 *
 *  @param filename XML- or YAML-file name
 *  @param forest   Random Trees instance
 */
void np::rtree_save(const char* filename, const CvRTrees* forest)
{
  CvFileStorage* fs = cvOpenFileStorage(filename,0,CV_STORAGE_WRITE);
  char name[] = "blablabla";
  forest->write(fs, name);
  cvReleaseFileStorage(&fs);
}

/** @brief Loads a Random Forest from file
 *
 *  Wraps around CvStatModel::load().
 *  "The method loads the complete model state with the specified name 
 *  (or default model-dependent name) from the specified XML or YAML file."
 *
 *  @param filename XML- or YAML-file name
 *
 *  @return instance of an Random Forest or NULL if fail

 */
CvRTrees* np::rtree_load(const char* filename)
{
  CvRTrees *ret = new CvRTrees();
  CvFileStorage* fs = cvOpenFileStorage(filename,0,CV_STORAGE_READ);
  CvFileNode* node = cvGetFileNodeByName(fs,0,"blablabla");
  ret->read(fs,node);
  return ret;
}

/** @brief Saves prediction results
 *
 *  @param filename any file name, e.g. train.rtree.prediction.array
 *  @param p        prediction output
 *  @param err      prediction errors (pass empty array, if no errors computed)
 *
 *  @return 0 on ok
 */
int np::rtree_predict_save(const char* filename, const floatA& p, const floatA& err)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
  {
    np::msg_error(HERE, "Could not open file", false);
    return -1;
  }

  p.writeTagged(os, "rtree_predict_floatA", false); // TODO debug
  err.writeTagged(os, "rtree_error_floatA", false);
  os.close();
  return 0;
}

/** @brief Load prediction results
 *
 *  @param filename any file name, e.g. train.rtree.prediction.array
 *  @param p        prediction output
 *  @param err      prediction errors (pass empty array, if no errors computed)
 *
 *  @return 0 on ok, cf. params p, err
 */
int np::rtree_predict_load(const char* filename, floatA& p, floatA& err)
{
  std::ifstream is;
  is.open(filename);

  if(!is)
  {
    np::msg_error(HERE, "Could not open file", false);
    return -1;
  }

  p.readTagged(is, "rtree_predict_floatA");
  err.readTagged(is, "rtree_error_floatA");
  is.close();

  return 0;
}
