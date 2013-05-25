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

/** \file rtree.h
    \brief OpenCV Random Trees */

#ifndef _RTREE_H
#define _RTREE_H

#include <MT/array.h>

// TODO mv all OpenCV-wrapping methods to directory wrappers/opencv{.h/.cpp}

class CvRTrees;                                        // defined in opencv/ml.h

namespace np {

/** \brief Trains a Random Forest
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
CvRTrees* rtree_train
(
 const floatA& data,
 const floatA& responses,
 float fraction,
 uint  num_trees,
 bool  is_regression = true
);

/** \brief Loads a Random Forest from file
 *
 *  Wraps around CvStatModel::load().
 *  "The method loads the complete model state with the specified name 
 *  (or default model-dependent name) from the specified XML or YAML file."
 *
 *  @param filename XML- or YAML-file name
 *
 *  @return instance of an Random Forest or NULL if fail

 */
CvRTrees* rtree_load(const char* filename);

/** \brief Saves a Random Forest from file
 *
 *  Wraps around CvStatModel::save().
 *  "The method save stores the complete model state to the specified XML or 
 *  YAML file with the specified name or default name (that depends on the 
 *  particular class)."
 *
 *  @param filename XML- or YAML-file name
 *  @param forest   Random Trees instance
 */
void rtree_save(const char* filename, const CvRTrees* forest);

/** \brief Predicts the output for the input samples
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
void rtree_predict(floatA& p, const CvRTrees* forest, const floatA& samples);

/** \brief Saves prediction results
 *
 *  @param filename any file name, e.g. train.rtree.prediction.array
 *  @param p        prediction output
 *  @param err      prediction errors (pass empty array, if no errors computed)
 *
 *  @return 0 on ok
 */
int rtree_predict_save(const char* filename, const floatA& p, const floatA& err);

/** \brief Load prediction results
 *
 *  @param filename any file name, e.g. train.rtree.prediction.array
 *  @param p        prediction output
 *  @param err      prediction errors (pass empty array, if no errors computed)
 *
 *  @return 0 on ok, cf. params p, err
 */
int rtree_predict_load(const char* filename, floatA& p, floatA& err);

} // namespace np

#endif
