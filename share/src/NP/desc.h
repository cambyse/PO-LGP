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

/** @file desc.h
    @brief Descriptors and utils */

#ifndef _NP_DESC_H
#define _NP_DESC_H

#include <Core/array.h>
#include "nputils.h"
#include "ucm.h"


namespace np {
static const char* label_types[] ={"exact", "contains", "part-of", "has-overlap", NULL};

/** @brief Shape descriptors for an UCM tree
 *
 *  Computes for all N patches of an UCM segmentation tree the following
 *  descriptors:
 *    (1) Elliptic Fourier Descriptors, size: N-by-num_coef
 *    (2) Centers of each patch, normalized to [0,1], size: N-by-2
 *    (3) Size of each patch, normalized to [0,1], size: N-by-1
 *
 *  @see seg::ucm_gen_tree(), efd_contour()
 *  @param efd       Elliptic Fourier descriptors for all patches, size: N-by-num_coef
 *  @param centers   center coord. of each patch normalized to [0,1], size: N-by-2
 *  @param sizes     size normalized wrt total no pix in image [0,1], size: N-by-1
 *  @param tree      UCM segmentation tree
 *  @param num_coef  number of Fourier coefficients to be used (default: 20)
 *
 *  @return cf. parameters efd, centers, sizes
 */
void ucm_shapedesc
(
 floatA& efd,
 floatA& centers,
 floatA& sizes,
 const seg::UcmTree& tree,
 unsigned int num_coef = 20
);

/** @brief Load the shape descriptors of an UCM tree
 *
 *  Load the file that store the Elliptic Fourier descriptors (EFD), sizes, and
 *  centers of each patch in an UCM segmentation tree. Sizes and centers are
 *  normalized to the value range [0,1]. The EFD matrix has the size N-by-c,
 *  where N is the number of patches, and c the number of Fourier coefficients.
 *
 *  @see seg::ucm_gen_tree()
 *  @param efd       Elliptic Fourier descriptors for all patches, size: N-by-c
 *  @param centers   center coord. of each patch normalized to [0,1], size: N-by-2
 *  @param sizes     size normalized wrt total no pix in image [0,1], size: N-by-1
 *  @param filename  file name where the arrays are stored
 *
 *  @return  0 on ok and cf. parameters efd, centers, sizes
 */
int load_ucm_shapedesc
(
 floatA& efd,
 floatA& centers,
 floatA& sizes,
 const char *filename
);

/** @brief Save the shape descriptors of an UCM tree
 *
 *  Save the Elliptic Fourier descriptors (EFD), sizes, and centers of each
 *  patch in an UCM segmentation tree to a file. Sizes and centers are assumed
 *  to be normalized to the value range [0,1]. The EFD matrix has the size
 *  N-by-c, where N is the number of patches, and c the number of Fourier
 *  coefficients.
 *
 *  @see seg::ucm_gen_tree()
 *  @param efd       Elliptic Fourier descriptors for all patches, size: N-by-c
 *  @param centers   center coord. of each patch normalized to [0,1], size: N-by-2
 *  @param sizes     size normalized wrt total no pix in image [0,1], size: N-by-1
 *  @param filename  file name where the arrays are stored
 *
 *  @return 0 on ok
 */
int save_ucm_shapedesc
(
 const floatA& efd,
 const floatA& centers,
 const floatA& sizes,
 const char *filename
);

/** @brief Color descriptors for patches in an UCM tree
 *
 *  Computes for all N patches of an UCM segmentation tree the following
 *  color descriptors:
 *    cf. Plath2009, ICML paper adapted for UCM trees
 *
 *  @param desc_HS  binned H+S values, size: (N, num_bH*num_bS), N>0, num_bH/S>0
 *  @param desc_V   binned V values, size: (N, num_bV), N>0, num_bV>0
 *  @param image    color image, size: (y,x,3)
 *  @param tree     UCM segmentation tree
 *  @param num_bH   number of bins for Hue, 0 < num_bH <=256
 *  @param num_bS   number of bins for Saturation, 0 < num_bS <=256
 *  @param num_bV   number of bins for Value, 0 < num_bV <=256
 *
 *  @return 0 on OK, cf. parameters desc_HS, desc_V
 */
int ucm_colordesc_hsv
(
  floatA& desc_HS,
  floatA& desc_V,
  const seg::UcmTree& tree,
  const byteA& image,
  uint num_bH = 10,
  uint num_bS = 10,
  uint num_bV = 10
);

/** @brief Load HSV color descriptors
 *
 *  @param desc_HS   binned H+S values, size: (N, num_bH*num_bS), N>0, num_bH/S>0
 *  @param num_bH    number of bins for Hue
 *  @param num_bS    number of bins for Saturation
 *  @param desc_V    binned V values, size: (N, num_bV), N>0, num_bV>0
 *  @param filename  path and filename
 *
 *  @return 0 on OK, cf. parameters, desc_HS, num_bH, num_bS, desc_V
 */
int ucm_colordesc_hsv_load
(
 floatA& desc_HS,
 uint& num_bH,
 uint& num_bS,
 floatA& desc_V,
 const char* filename
);

/** @brief Save HSV color descriptors
 *
 *  @param desc_HS   binned H+S values, size: (N, num_bH*num_bS), N>0, num_bH/S>0
 *  @param num_bH    number of bins for Hue
 *  @param num_bS    number of bins for Saturation
 *  @param desc_V    binned V values, size: (N, num_bV), N>0, num_bV>0
 *  @param filename  path and filename
 *
 *  @return 0 on OK
 */
int ucm_colordesc_hsv_save
(
 const floatA& desc_HS,
 uint num_bH,
 uint num_bS,
 const floatA& desc_V,
 const char* filename
);

/** @brief SURF texture descriptors for vertices on regular grid
 *
 *  Lays out a pyramid of regular grids over an image and computes SURF
 *  descriptors on vertices at a specified SURF scale. The SURF scales are
 *  chosen according to the grid width: s(w) = 0.3*w - 0.6, w - grid width.
 *
 *  @see SURF
 *  @param desc         SURF descriptors, size: N-by-64
 *  @param grid         all M points of all regular grids, size: M-by-1
 *  @param scales       SURF scale of each grid point, size: M-by-1
 *  @param image        an image
 *  @param grid_widths  L-by-1 array specifying grid with of each pyramid layer
 *  @param upright      rotation invariance
 *
 *  @return 0 on OK, cf. parameters desc, grid, scales
 */
int img_texturedesc
(
  floatA& desc,
  intA& grid,
  floatA& scales,
  const byteA& image,
  const uintA& grid_widths,
  bool upright = true
);

/** @brief Load a set of texture descriptors of an image
 *
 *  @param desc      descriptors matrix, row-wise, size: (N,D), N>0, D>0
 *  @param grid      grid points, size: (N,2), N>0
 *  @param scales    SURF scale for each grid point, size: (N)
 *  @param filename  path and filename
 *
 *  @return 0 on ok, cf. parameters desc, grid, scales
 */
int img_texturedesc_load(floatA& desc, intA& grid, floatA& scales, const char* filename);

/** @brief Save a set of texture descriptors of an image
 *
 *  @param desc      descriptors matrix, row-wise, size: (N,D), N>0, D>0
 *  @param grid      grid points, size: (N,2), N>0
 *  @param scales    SURF scale for each grid point, size: (N)
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int img_texturedesc_save
(
 const floatA& desc,
 const intA& grid,
 const floatA& scales,
 const char* filename
);

/** @brief Write all features into one matrix
 *
 *  @param desc      descriptors matrix, row-wise, size: (N,D), N>0, D>0
 *  @param grid      grid points, size: (N,2), N>0
 *  @param scales    SURF scale for each grid point, size: (N)
 *  @param list      a list of filenames (e.g., *.texturedesc.array), size: (M), M>0
 *
 *  @return 0 on ok, cf. parameters desc, grid, scales
 */
int img_texturedesc2block
(
 floatA& desc,
 intA& grid,
 floatA& scales,
 const stringA& list
);

/** @brief Write all features into one matrix
 *
 *  @param desc      descriptors matrix, row-wise, size: (N,D), N>0, D>0
 *  @param list      a list of filenames (e.g., *.texturedesc.array), size: (M), M>0
 *
 *  @return 0 on ok, cf. parameters desc, grid, scales
 */
int final_features2block
(
 floatA& desc,
 const stringA& list
);

/** @brief Generate an unique name for a codebook
 *
 *  @see np::kmeans()
 *  @param name          name suggestion
 *  @param num_clusters  number of clusters, i.e. number of visual words
 *  @param compactness   compactness as in np::kmeans()
 *  @param path          path to folder where codebook will be stored [NULL]
 *
 *  @return cf. parameters name
 */
void codebook_name
(
 charA& name,
 uint num_clusters,
 double compactness,
 const char* path = NULL
);

/** @brief Save a visual codebook
 *
 *  @param codebook  matrix with visual words, size: (N,D), N>0, D>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int codebook_save(const floatA& codebook, const char* filename);

/** @brief Load a visual codebook
 *
 *  @param codebook  matrix with visual words, size: (N,D), N>0, D>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int codebook_load(floatA& codebook, const char* filename);

/** @brief Generate an unique name for a codebook
 *
 *  Quantization of a set of descriptors given a visual codebook. The result
 *  is a (N-by-4) float-array. Colums 1 and 2 are the x- and y- position of the
 *  descriptor in the image, colum 3 is the SURF scale, and column 4 is the
 *  index of the closest visual words (cluster center, 0-(N-1)).
 *
 *  @note uses ANN
 *  @param xysc      quantized features, size: (N,4), N>0
 *  @param codebook  visual codebook (e.g. thru kmeans), size: (M,D), M>0, D>0
 *  @param desc      set of descriptors, size: (N,D), N>0, D>0
 *  @param grid      image grid points of descriptors, size: (N,2), N>0
 *  @param scales    SURF scales of descriptors, size: (N), N>0
 *
 *  @return 0 on OK, cf. parameters xysc
 */
int quantization
(
 floatA& xysc,
 const floatA& codebook,
 const floatA& desc,
 const intA& grid,
 const floatA& scales
);

/** @brief Load a set of quantized descriptors
 *
 *  @param xysc      quantized features, size: (N,4), N>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int quantization_load
(
 floatA& xysc,
 uint& num_codewords,
 const char* filename
);

/** @brief Save a set of quantized descriptors
 *
 *  @param xysc      quantized features, size: (N,4), N>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int quantization_save
(
 const floatA& xysc,
 uint num_codewords,
 const char* filename
);

enum LabelType {
  EQUAL            = 0,
  CONTAINS         = 1,
  PARTOF           = 2,
  CONTAINSORPARTOF = 3
};

/** @brief Generate labels for patches in an UCM tree
 *
 *  Generate labels for patches in an UCM tree. Each patch is assigned a value o
 *  from the interval [0,1] according to the type label:
 *
 *   TYPE                  key                  the patch A ...
 *    np::EQUAL             |A\cap C|/|A\cup C|  IS the object C
 *    np::CONTAINS          |A\cap C|/|C|        CONTAINS the object C
 *    np::PARTOF            |A\cap C|/|A|        PART OF the object C
 *    np::CONTAINSORPARTOF  |A\cap C|            CONTAINS OR IS PART OF the object C
 *
 *  A and C are both regions of the images, A is a patch in the UCM tree, and
 *  C is a patch of the given class-/instance-wise groundtruth mask.
 *
 *  The labels are a N-by-(number-of-classes+1) array. Column 0 corresponds to
 *  the background, the others columns denote the corresponding class indices.
 *  N is the total number of patches in the UCM tree.
 *
 *  @param labels       labels as real numbers, interval [0,1], size: (N,num_classes+1)
 *  @param tree         an UCM segmentation tree
 *  @param cls          class-wise segmentation groundtruth, size: (y,x)
 *  @param obj          instance-wise seg. groundtruth, size: (y,x)
 *  @param lt           type of labels: equals, contains, ... cf. description
 *  @param num_classes  number of classes of this learning problem, e.g. VOC: 20
 *
 *  @return 0 on OK, cf. parameter labels
 */
int labels
(
 floatA& labels,
 const seg::UcmTree& tree,
 const intA& cls,
 const intA& obj,
 uint num_classes
);

/** @brief Load labels
 *
 *  @param labels    the label array, size: (N), N>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int labels_load(floatA& labels, const char* filename);

/** @brief Save labels
 *
 *  @param labels    the label array, size: (N), N>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int labels_save(const floatA& labels, const char* filename);

/** @brief Write all label vectors into one matrix
 *
 *  @param labels  labels matrix, row-wise, size: (N,4), N>0
 *  @param list    a list of filenames (e.g., *.labels.array), size: (M), M>0
 *
 *  @return 0 on ok, cf. parameter labels
 */
int labels2block
(
 floatA& labels,
 const stringA& list
);

/** @brief Compute texture histograms
 *
 *  @see quantization(), ucm_gen_tree(), img_texturedesc()
 *  @param hist  histogram of visual codewords, size: (N,nc), N>0, nc>0
 *  @param xysc  quantized texture features, size: (M, 4), M>0
 *  @param tree  UCM segmentation tree with N>0 patches
 *
 *  @return 0 on OK
 */
int ucm_texturehist
(
 floatA& hist,
 const floatA& xysc,
 uint num_codewords,
 const seg::UcmTree& tree
);

/** @brief Load texture histograms
 *
 *  @param hist      histogram of visual codewords, size: (N,nc), N>0, nc>0
 *  @param filename  path and filename
 *
 *  @return 0 on OK, cf. parameter hist
 */
int ucm_texturehist_load(floatA& hist, const char* filename);

/** @brief Save texture histograms
 *
 *  @param hist      histogram of visual codewords, size: (N,nc), N>0, nc>0
 *  @param filename  path and filename
 *
 *  @return 0 on OK
 */
int ucm_texturehist_save(const floatA& hist, const char* filename);

/** @brief Save final feature of an UCM tree
 *
 *  @param ff        concatenated, normalized features
 *  @param tree      UCM segmentation tree
 *  @param texture   texture features of the tree above
 *  @param color_HS  Hue+Saturation bins
 *  @param color_V   Value bins
 *  @param shape     shape descriptors of the UCM tree
 *  @param centers   patch centers
 *  @param sizes     patch sizes
 *
 *  @return 0 on OK, cf. parameter ff
 */
int ucm_final_features
(
 floatA& ff,
 const seg::UcmTree& tree,
 const floatA& texture,
 const floatA& color_HS,
 const floatA& color_V,
 const floatA& shape,
 const floatA& centers,
 const floatA& sizes
);

/** @brief Load final features of an UCM tree
 *
 *  @param ff        concatenated, normalized features
 *  @param filename  path and filename
 *
 *  @return 0 on OK, cf. parameter ff
 */
int ucm_final_features_load(floatA& ff, const char* filename);

/** @brief Save final features of an UCM tree
 *
 *  @param ff        concatenated, normalized features
 *  @param filename  path and filename
 *
 *  @return 0 on OK
 */
int ucm_final_features_save(floatA& ff, const char* filename);
} // namespace np

#endif
