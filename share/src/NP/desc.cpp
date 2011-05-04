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

#include <MT/algos.h>
#include <fstream>
#include <limits>
#include "desc.h"
#include "efd.h"
#include "imgproc.h"
#include "nputils.h"
#include "cvutils.h"
#include "wrappers/surf.h"

namespace np { // hidden declaration
inline void init_hsv_lut(intA& lut, uint num_bins);
inline void normalize_histogram(floatA& hist, float s = 1.0);
inline void unique_indices(intA& indices, const intA& data);
}

/*! \brief Shape descriptors for an UCM tree
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
void np::ucm_shapedesc
(
 floatA& efd,
 floatA& centers,
 floatA& sizes,
 const seg::UcmTree& tree,
 unsigned int num_coef
)
{
    int num_patches = tree.nodes.N;

  // determine the edge thresholds, which occur in the image
  intA tree_levels;
  int num_tree_levels;
  seg::ucm_tree_levels(tree_levels, tree);
  num_tree_levels = tree_levels.N;

  MT::Array<intA> contours(num_patches);   // contours of each patch in the tree
  MT::Array<intA> contours2D(num_patches);    // same as above but 2D img coord.
  MT::Array<intA> patchmap_pyramid(num_tree_levels);   // patchmap at each level

  if (num_coef == 0)
    num_coef = 20;

  // resize to output to fit the data
  efd.resize(num_patches, num_coef);
  centers.resize(num_patches, 2);
  centers = 0;
  sizes.resize(num_patches);
  sizes = 0;

  // for each patch at each level of the tree, that has not been processed yet,
  // (1) determine the contour, (2) translate contour to 2D coordinates,
  // (3) compute Elliptic Fourier Descriptors (EFD), (4) determine size, and
  // (5) determine its center.
  intA done(num_patches); done = 0;
  intA leaves;
  intA centers_temp, sizes_temp;
  doubleA efd_temp;
  uint num_leaves, leaf_cur, contour_start, p, v;
//   double inv;
  for (int li = 0; li < num_tree_levels; li++)
  {
    // determine the current patchmap at level tree_levels(li)
    seg::ucm_patchmap(patchmap_pyramid(li), leaves, tree, tree_levels(li));
    num_leaves = leaves.N;
    intA& patchmap_cur = patchmap_pyramid(li);

    // determine centers and sizes of the current leaf patches
    centers_temp.resize(num_patches, 2);
    centers_temp = 0;
    sizes_temp.resize(num_patches);
    sizes_temp = 0;
    for (uint yi = 0; yi < patchmap_cur.d0; yi++)
    {
      for (uint xi = 0; xi < patchmap_cur.d1; xi++)
      {
        p = patchmap_cur.d1 * yi + xi;
        v = patchmap_cur.p[p];
        centers_temp(v-1, 1) += xi;
        centers_temp(v-1, 0) += yi;
        sizes_temp(v-1) += 1;
      }
    }

    for (uint pi = 0; pi < centers_temp.d0; pi++)
    {
      if (sizes_temp(pi) == 0)
        centers_temp[pi] = -1;
      else
      {
        centers_temp(pi,1) /= sizes_temp(pi);
        centers_temp(pi,0) /= sizes_temp(pi);
      }
    }

    // process all the leaf patches at current level
    for (uint pi = 0; pi < num_leaves; pi++)
    {
      leaf_cur = leaves(pi) - 1;                // memory index of current patch

      // (0) has the current patch been processed already?
      if (done(leaf_cur))
        continue;

      intA& contour_cur = contours(leaf_cur);
      intA& contour2D_cur = contours2D(leaf_cur);

      // (1) determine the contour of the current patch
      contour_start = patchmap_cur.findValue(leaves(pi));
      np::ctrace(contours(leaf_cur), patchmap_cur, contour_start);

      // (2) translate 1D contour to 2D coordinates
      contour2D_cur.resize(contour_cur.N,2);
      for (uint i = 0; i < contour_cur.N; i++)
      {
        contour2D_cur(i,1) = contour_cur(i)/patchmap_cur.d1;
        contour2D_cur(i,0) = contour_cur(i) -
                             (contour2D_cur(i,1)*patchmap_cur.d1);
      }

      // (3) compute Elliptic Fourier Descriptors (EFD)
      np::efd_contour(efd_temp, contour2D_cur, num_coef);
      for (uint ci = 0; ci < num_coef; ci++)
        efd(leaf_cur, ci) = (float) efd_temp(ci);

      // (4),(5) determine patch size and center, and normalize to interval [0,1]
      sizes(leaf_cur) = (float) sizes_temp(leaf_cur) / (float) patchmap_cur.N;
      centers(leaf_cur, 1) = (float) centers_temp(leaf_cur,1)
                             / (float) patchmap_cur.d1;
      centers(leaf_cur, 0) = (float) centers_temp(leaf_cur,0)
                             / (float) patchmap_cur.d0;

      done(leaf_cur) = 1;
    }
  }
}

/*! \brief Load the shape descriptors of an UCM tree
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
int np::load_ucm_shapedesc
(
 floatA& efd,
 floatA& centers,
 floatA& sizes,
 const char *filename
)
{
  std::ifstream is;
  is.open(filename);
  if (!is)
    np::msg_error(HERE, "cannot open file");

  efd.readTagged(is, "EFD_floatA");
  centers.readTagged(is, "centers_floatA");
  sizes.readTagged(is, "sizes_floatA");

  is.close();
  return 0;
}

/*! \brief Save the shape descriptors of an UCM tree
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
int np::save_ucm_shapedesc
(
 const floatA& efd,
 const floatA& centers,
 const floatA& sizes,
 const char *filename
)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  efd.writeTagged(os, "EFD_floatA", true);
  centers.writeTagged(os, "centers_floatA", true);
  sizes.writeTagged(os, "sizes_floatA", true);

  os.close();
  return 0;
}

/*! \brief Color descriptors for patches in an UCM tree
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
int np::ucm_colordesc_hsv
(
  floatA& desc_HS,
  floatA& desc_V,
  const seg::UcmTree& tree,
  const byteA& image,
  uint num_bH,
  uint num_bS,
  uint num_bV
)
{
  // look-up tables (LUT) to match pixel value to according bin
  intA bins_HS, bins_V;
  uint num_patches = tree.nodes.N, num_levels;
  int* patch_idx;
  intA levels, patchmap, leaves;

  if (!image.nd == 3)
    np::msg_error(HERE, "make sure the input is a color image");

  // all num_b* have to be 0 < num_b* <= 256
  if ((num_bH == 0) || (num_bH >= 256))
    np::msg_error(HERE, "make sure that: 0 < num_bH <= 256");
  if ((num_bS == 0) || (num_bS >= 256))
    np::msg_error(HERE, "make sure that: 0 < num_bS <= 256");
  if ((num_bV == 0) || (num_bV >= 256))
    np::msg_error(HERE, "make sure that: 0 < num_bV <= 256");

  // init LUTs
  bins_HS.resize(256*256);
  init_hsv_lut(bins_HS, num_bH*num_bS);
  bins_V.resize(256);
  init_hsv_lut(bins_V, num_bV);

  // iterate thru the different levels of the tree: at each level etract the
  // corresponding patchmap and determine the HSV histogram for the patches
  // at that particular level.
  desc_HS.resize(num_patches, num_bH*num_bS);
  desc_HS = 0;
  desc_V.resize(num_patches, num_bV);
  desc_V = 0;
  seg::ucm_tree_levels(levels, tree);// number of levels and corresp. thresholds
  num_levels = levels.N;
  boolA patch_done(num_patches); patch_done = false;
  boolA patch_done_new(num_patches); patch_done_new = false;
  for (uint level = 0; level < num_levels; level++)
  {
    // patchmap at current level
    seg::ucm_patchmap(patchmap, leaves, tree, levels(level));

    // determine HSV histograms at current tree level
    patch_idx = patchmap.p;
    for (uint pix = 0; pix < image.N; pix+=3)
    {
      // check if the current patch has been processed at an earlier level
      // already, if not process it now
      if (!patch_done(*patch_idx-1))
      {
        desc_HS(*patch_idx-1, bins_HS(image.p[pix]*image.p[pix+1])) += 1;
        desc_V(*patch_idx-1,  bins_V(image.p[pix+2]))               += 1;
        patch_done_new(*patch_idx-1) |= true;
      }

      patch_idx++;
    }

    // mark the patches that have been processed in this level in order to
    // avoid processing them again at the next level
    for (uint i = 0; i < num_patches; i++)
      patch_done(i) = patch_done_new(i);
  }

  // normalize the rows of the histograms to sum up to 1.0
  normalize_histogram(desc_HS, 1.0);
  normalize_histogram(desc_V, 1.0);

  return 0;
}

/*! \brief Load HSV color descriptors
 *
 *  @param desc_HS   binned H+S values, size: (N, num_bH*num_bS), N>0, num_bH/S>0
 *  @param num_bH    number of bins for Hue
 *  @param num_bS    number of bins for Saturation
 *  @param desc_V    binned V values, size: (N, num_bV), N>0, num_bV>0
 *  @param filename  path and filename
 *
 *  @return 0 on OK, cf. parameters, desc_HS, num_bH, num_bS, desc_V
 */
int np::ucm_colordesc_hsv_load
(
 floatA& desc_HS,
 uint& num_bH,
 uint& num_bS,
 floatA& desc_V,
 const char* filename
)
{
  std::ifstream is;
  is.open(filename);

  if (!is)
    np::msg_error(HERE, "Could not open file");

  if (num_bH*num_bS != desc_HS.d1)
    np::msg_error(HERE, "num_bH*num_bS != desc_HS.d1, check input arguments");

  uintA bins(3);
  bins.readTagged(is, "num_bin_uintA");
  num_bH = bins(0);
  num_bS = bins(1);

  desc_HS.readTagged(is, "desc_HS_floatA");
  desc_V.readTagged(is, "desc_V_floatA");

  is.close();
  return 0;
}

/*! \brief Save HSV color descriptors
 *
 *  @param desc_HS   binned H+S values, size: (N, num_bH*num_bS), N>0, num_bH/S>0
 *  @param num_bH    number of bins for Hue
 *  @param num_bS    number of bins for Saturation
 *  @param desc_V    binned V values, size: (N, num_bV), N>0, num_bV>0
 *  @param filename  path and filename
 *
 *  @return 0 on OK
 */
int np::ucm_colordesc_hsv_save
(
 const floatA& desc_HS,
 uint num_bH,
 uint num_bS,
 const floatA& desc_V,
 const char* filename
)
{
  std::ofstream os;
  os.open(filename);

  if (!os)
    np::msg_error(HERE, "Could not open file");

  if (num_bH*num_bS != desc_HS.d1)
    np::msg_error(HERE, "num_bH*num_bS != desc_HS.d1, check input arguments");

  uintA bins(3);
  bins(0) = num_bH;
  bins(1) = num_bS;
  bins(2) = desc_V.d1;

  bins.writeTagged(os, "num_bin_uintA", true);
  desc_HS.writeTagged(os, "desc_HS_floatA", true);
  desc_V.writeTagged(os, "desc_V_floatA", true);

  os.close();
  return 0;
}

/*! \brief SURF texture descriptors for vertices on regular grid
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
int np::img_texturedesc
(
  floatA& desc,
  intA& grid,
  floatA& scales,
  const byteA& image,
  const uintA& grid_widths,
  bool upright
)
{
  floatA desc_t, scales_t;
  intA grid_t;
  uint num_widths;
  bool is_color = (image.nd == 3);

  if (grid_widths.N == 0)
    np::msg_error(HERE, "No grid widths! You need to specify at least one width.");

  // prepare the output
  desc.clear();
  grid.clear();
  scales.clear();
  grid.reshape(0,2);
  if (is_color) desc.reshape(0, 192);
  else          desc.reshape(0, 64);

  // work thru the pyramid of regular grids
  num_widths = grid_widths.N;
  for (uint li = 0; li < num_widths; li++)
  {
    desc_t.clear();
    grid_t.clear();
    scales_t.clear();

    // generate grid points
    np::regular_grid(grid_t, image.d1, image.d0, grid_widths(li));
    grid.append(grid_t);

    // determine scales for points at current layer of the grid pyramid
    scales_t.resize(grid_t.d0);
    scales_t = np::wrappers::surf_scale(grid_widths(li));     // TODO debug
    scales.append(scales_t);

    // compute texture descriptors of current layer
    np::wrappers::surf_descriptors(desc_t, image, grid_t, scales, upright);
    desc.append(desc_t);

  }

  return 0;
}

/*! \brief Load a set of texture descriptors of an image
 *
 *  @param desc      descriptors matrix, row-wise, size: (N,D), N>0, D>0
 *  @param grid      grid points, size: (N,2), N>0
 *  @param scales    SURF scale for each grid point, size: (N)
 *  @param filename  path and filename
 *
 *  @return 0 on ok, cf. parameters desc, grid, scales
 */
int np::img_texturedesc_load(floatA& desc, intA& grid, floatA& scales, const char* filename)
{
  std::ifstream is;
  is.open(filename);
  if (!is)
    np::msg_error(HERE, "cannot open file");

  desc.readTagged(is, "texturedesc_floatA");
  grid.readTagged(is, "texturegrid_floatA");
  scales.readTagged(is, "texturescales_floatA");

  is.close();
  return 0;
};

/*! \brief Save a set of texture descriptors of an image
 *
 *  @param desc      descriptors matrix, row-wise, size: (N,D), N>0, D>0
 *  @param grid      grid points, size: (N,2), N>0
 *  @param scales    SURF scale for each grid point, size: (N)
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int np::img_texturedesc_save
(
 const floatA& desc,
 const intA& grid,
 const floatA& scales,
 const char* filename
)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  desc.writeTagged(os, "texturedesc_floatA", true);
  grid.writeTagged(os, "texturegrid_floatA", true);
  scales.writeTagged(os, "texturescales_floatA", true);

  os.close();
  return 0;
}

/*! \brief Write all features into one matrix
 *
 *  @param desc      descriptors matrix, row-wise, size: (N,D), N>0, D>0
 *  @param grid      grid points, size: (N,2), N>0
 *  @param scales    SURF scale for each grid point, size: (N)
 *  @param list      a list of filenames (e.g., *.texturedesc.array), size: (M), M>0
 *
 *  @return 0 on ok, cf. parameters desc, grid, scales
 */
int np::img_texturedesc2block
(
 floatA& desc,
 intA& grid,
 floatA& scales,
 const stringA& list
)
{
  floatA desc_t, scales_t;
  intA grid_t;
  uint num_list = list.N;

  if (num_list == 0)
    np::msg_error(HERE, "File list is empty");

  desc.clear();
  grid.clear();
  scales.clear();
  grid.reshape(0,2);
  // NOTE desc.reshape(0, 64 or 192) --> reshape is done inside the for-loop

  for (uint fi = 0; fi < num_list; fi++)
  {
    img_texturedesc_load(desc_t, grid_t, scales_t, list(fi).c_str());

// NOTE to make sure that the data is not NAN
    for (uint i = 0; i < desc_t.N; i++)
      if (isnan(desc_t.p[i]))
        desc_t.p[i] = 0.0;

    if (fi == 0) // NOTE make sure descriptors can be easily appended, cf. above
      desc.reshape(0, desc_t.d1);
    else
      if (desc.d1 != desc_t.d1) // append color to bw descriptor or vice versa
        np::msg_error(HERE, "Cannot append current descriptors to block, b/c of diff in d1");

    desc.append(desc_t);
    grid.append(grid_t);
    scales.append(scales_t);
  }

  return 0;
}

/*! \brief Write all features into one matrix
 *
 *  @param desc      descriptors matrix, row-wise, size: (N,D), N>0, D>0
 *  @param list      a list of filenames (e.g., *.texturedesc.array), size: (M), M>0
 *
 *  @return 0 on ok, cf. parameters desc, grid, scales
 */
int np::final_features2block
(
 floatA& desc,
 const stringA& list
)
{
  floatA desc_t;
  uint num_list = list.N;

  if (num_list == 0)
    np::msg_error(HERE, "File list is empty");

  desc.clear();
  for (uint fi = 0; fi < num_list; fi++)
  {
    ucm_final_features_load(desc_t, list(fi).c_str());

    // NOTE to make sure that the data is not NAN
    for (uint i = 0; i < desc_t.N; i++)
      if (isnan(desc_t.p[i]))
        std::cout << "NAN-Warning: " << list(fi).c_str() << std::endl;

    if (fi == 0) // NOTE make sure descriptors can be easily appended, cf. above
      desc.reshape(0, desc_t.d1);
    else
      if (desc.d1 != desc_t.d1) // append color to bw descriptor or vice versa
        np::msg_error(HERE, "Cannot append current descriptors to block, b/c of diff in d1");

    desc.append(desc_t);
  }

  return 0;
}

/*! \brief Generate an unique name for a codebook
 *
 *  @see np::kmeans()
 *  @param name          name suggestion
 *  @param num_clusters  number of clusters, i.e. number of visual words
 *  @param compactness   compactness as in np::kmeans()
 *  @param path          path to folder where codebook will be stored [NULL]
 *
 *  @return cf. parameters name
 */
void np::codebook_name(charA& name, uint num_clusters, double compactness, const char* path)
{
  uint len;
  std::ostringstream oss, oss2;
  oss2 <<  std::setw(5) << std::setfill('0') << ceil(compactness);
  oss.str("");
  oss << (path != NULL ? path : ".") << "/"
      << std::setw(4) << std::setfill('0') << num_clusters
      << "_" << (compactness >= 0 ? oss2.str().c_str() : "")
      << (compactness >= 0 ? "_" : "")
      << datetime(false).p
      << ".codebook.array";
  len = oss.str().length();
  name.resize(len+1);
  name = '\0';
  memcpy(name.p, oss.str().c_str(), len);
}

/*! \brief Save a visual codebook
 *
 *  @param codebook  matrix with visual words, size: (N,D), N>0, D>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int np::codebook_save(const floatA& codebook, const char* filename)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  codebook.writeTagged(os, "codebook_floatA", true);

  os.close();
  return 0;
}

/*! \brief Load a visual codebook
 *
 *  @param codebook  matrix with visual words, size: (N,D), N>0, D>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int np::codebook_load(floatA& codebook, const char* filename)
{
  std::ifstream is;
  is.open(filename);
  if (!is)
    np::msg_error(HERE, "cannot open file");

  codebook.readTagged(is, "codebook_floatA");

  is.close();
  return 0;
}

/*! \brief Generate an unique name for a codebook
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
int np::quantization
(
 floatA& xysc,
 const floatA& codebook,
 const floatA& desc,
 const intA& grid,
 const floatA& scales
)
{
#if defined MT_ANN | defined NP_ANN
  intA nn;
  ANN codebook_tree;
  doubleA codebook_t;
  array2array(codebook_t, codebook);
  codebook_tree.setX(codebook_t);
#endif

  int num_codewords = codebook.d0;
  int num_descriptors = desc.d0;
  doubleA dist(num_codewords);

  // resize quantization output (XYSC-format)
  xysc.resize(num_descriptors, 4);

  // for all features on each level
  doubleA row;
  for (uint fei = 0; fei < desc.d0; fei++)
  {
#if defined MT_ANN | defined NP_ANN
    array2array(row, desc[fei]);
    codebook_tree.getNN(row, 1, nn);
#else
    dist = std::numeric_limits<double>::max();
    for (int cwi = 0; cwi < num_codewords; cwi++)
      dist(cwi) = sqrDistance(desc[fei], codebook[cwi]);
#endif

    xysc(fei, 0) = grid(fei,0);                     // x
    xysc(fei, 1) = grid(fei,1);                     // y
    xysc(fei, 2) = scales(fei);                     // scale
#if defined MT_ANN | defined NP_ANN
    xysc(fei, 3) = nn(0);
#else
    xysc(fei, 3) = dist.minIndex();                 // nearest neighbor: index
                                                    // of visual codewords
#endif
  }

  return 0;
}

/*! \brief Load a set of quantized descriptors
 *
 *  @param xysc      quantized features, size: (N,4), N>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int np::quantization_load
(
 floatA& xysc,
 uint& num_codewords,
 const char* filename
)
{
  std::ifstream is;
  is.open(filename);
  if (!is)
    np::msg_error(HERE, "cannot open file");

  uintA nc(1);
  nc.readTagged(is, "num_codewords_uintA");
  num_codewords = nc(0) ;
  xysc.readTagged(is, "xysc_floatA");

  is.close();
  return 0;
}

/*! \brief Save a set of quantized descriptors
 *
 *  @param xysc      quantized features, size: (N,4), N>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int np::quantization_save
(
 const floatA& xysc,
 uint num_codewords,
 const char* filename
)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  uintA nc(1);
  nc(0) = num_codewords;
  nc.writeTagged(os, "num_codewords_uintA", true);
  xysc.writeTagged(os, "xysc_floatA", true);

  os.close();
  return 0;
}

/*! \brief Generate labels for patches in an UCM tree
 *
 *  Generate labels for patches in an UCM tree. Each patch is assigned a value o
 *  from the interval [0,1] according to the type label:
 *
 *   TYPE                  key                  the patch A ...
 *    np::EQUAL             |A\cap C|/|A\cup C|  IS the object C
 *    np::CONTAINS          |A\cap C|/|C|        CONTAINS the object C
 *    np::EQUAL             |A\cap C|/|A|        PART OF the object C
 *    np::EQUAL             |A\cap C|            CONTAINS OR IS PART OF the object C
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
int np::labels
(
 floatA& labels,
 const seg::UcmTree& tree,
 const intA& cls,
 const intA& obj,
 uint num_classes
)
{
  uint num_patches = tree.nodes.N, num_dim = num_classes+1, num_types = 4;
  uint num_obj, num_cls, num_pix = tree.patchmap.N;
  uint cur_patch, cur_cls, cur_obj;
  floatA sizes_obj, sizes_patch, intersection_AO;
  uintA O_cls;
  uint num_levels;
  intA patchmap, levels, leaves, indices;
  boolA patch_done;

  if (num_classes < 1)
    np::msg_error(HERE, "value of parameter num_classes must be larger than 0");

  if (num_pix != cls.N)
    np::msg_error(HERE, "num_pix is not equal to number of pixels of arg. cls");
  if (num_pix != obj.N)
    np::msg_error(HERE, "num_pix is not equal to number of pixels of arg. obj");

  // resize the label matrix to fit labels for each type of each patch for each
  // class + background
  labels.resize(num_types, num_patches, num_dim);
  labels = -1;
  // labels[0]: tree patch equals groundtruth patch
  // labels[1]: tree patch contains groundtruth patch
  // labels[2]: tree patch is part of groundtruth patch
  // labels[3]: tree patch contains or is part of groundtruth patch

  // 0. determine the size and class of each of the groundtruth object patches
  unique_indices(indices, obj);
  num_obj = indices.N;
  unique_indices(indices, cls);
  num_cls = indices.N;
  sizes_obj.resize(num_obj+1);              sizes_obj = 0;
  O_cls.resize(num_obj+1);                  O_cls = 0;
  for (uint pix = 0; pix < num_pix; pix++)
  {
    cur_obj = obj.p[pix];
    cur_cls = cls.p[pix];

    // ignore pixels with value 255, b/c that's a VOC-specific slack area (not
    // counted)
    if (cur_obj == 255 || cur_cls == 255)
      continue;

    sizes_obj(cur_obj) += 1;
//     O_cls(cur_obj)   += 1;
    O_cls(cur_obj)   = cur_cls;

  }

  // 1. determine the size of each patch in the tree and its overlap with
  //    the groundtruth object patches: |A_i \cap O_j|
  seg::ucm_tree_levels(levels, tree);
  num_levels = levels.N;
  sizes_patch.resize(num_patches);      sizes_patch = 0;
  intersection_AO.resize(num_patches, num_obj+1); intersection_AO = 0;
  patch_done.resize(num_patches);       patch_done = false;
  boolA patch_done_cur(num_patches);    patch_done_cur = false;;
  for (uint level = 0; level < num_levels; level++)
  {
    // get patchmap of current level in tree
    seg::ucm_patchmap(patchmap, leaves, tree, levels(level));

    // determine patch sizes and overlaps at current level
    for (uint pix = 0; pix < num_pix; pix++)
    {
      cur_patch = patchmap.p[pix]-1; // -1 b/c map starts counting at 1, C++ at 0
      cur_obj   = obj.p[pix];
      cur_cls   = cls.p[pix];

      if(patch_done(cur_patch))
        continue;
      else
        patch_done_cur(cur_patch) = true;

      // ignore pixels with value 255, b/c that's a VOC-specific slack area (not
      // counted)
      if (cur_obj == 255 || cur_cls == 255)
        continue;

      sizes_patch(cur_patch) += 1;
      intersection_AO(cur_patch, cur_obj) += 1;
    }

    // Mark patch i as done, if its size has been measured at one level already.
    for (uint i = 0; i < num_patches; i++)
      patch_done(i) |= patch_done_cur(i);
  }

  // 2. compute the different labels using the information from 0. and 1.
  for (uint pi = 0; pi < num_patches; pi++)
  {
    for (uint oi = 0; oi <= num_obj; oi++)
    {
      cur_cls = O_cls(oi); // class index (0...20) of current object

      labels(0, pi, cur_cls) = intersection_AO(pi,oi) / \
        (sizes_patch(pi) + sizes_obj(oi) - intersection_AO(pi,oi));       // equals
      labels(1, pi, cur_cls) = intersection_AO(pi,oi) / sizes_obj(oi);    // contains
      labels(2, pi, cur_cls) = intersection_AO(pi,oi) / sizes_patch(pi);  // part of
      labels(3, pi, cur_cls) = (intersection_AO(pi,oi) > 0);              // has overlap

      for (uint z = 0; z < 4; z++)
        if (isnan(labels(z,pi,cur_cls)))
        {
          std::cout << "type = " << z << " patch: " << pi << " class: " << cur_cls << " patch size:" << sizes_patch(pi) << std::endl;
          labels(z,pi,cur_cls) = 0;
        }
    }
  }
  std::cout << labels.d0 << " " << labels.d1 << " " << labels.d2 << " " << std::endl;
  for (uint i = 0; i < labels.N; i++)
    if (isnan(labels.p[i]))
      std::cout << "nan " << std::endl;
  return 0;
}

/*! \brief Load labels
 *
 *  @param labels    the label array, size: (N), N>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int np::labels_load(floatA& labels, const char* filename)
{
  std::ifstream is;
  is.open(filename);
  if (!is)
    np::msg_error(HERE, "cannot open file");

  labels.readTagged(is, "labels_floatA");

  is.close();
  return 0;
}

/*! \brief Save labels
 *
 *  @param labels    the label array, size: (N), N>0
 *  @param filename  path and filename
 *
 *  @return 0 on ok
 */
int np::labels_save(const floatA& labels, const char* filename)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  labels.writeTagged(os, "labels_floatA", true);

  os.close();
  return 0;
}

/*! \brief Write all label vectors into one matrix
 *
 *  @param labels  labels matrix, row-wise, size: (N,4), N>0
 *  @param list    a list of filenames (e.g., *.labels.array), size: (M), M>0
 *
 *  @return 0 on ok, cf. parameter labels
 */
int np::labels2block
(
 floatA& labels,
 const stringA& list
)
{
  floatA labels_t, labels_t0, labels_t1, labels_t2, labels_t3;
  uint num_list = list.N;

  if (num_list == 0)
    np::msg_error(HERE, "File list is empty");

  labels.clear();
  for (uint fi = 0; fi < num_list; fi++)
  {
    labels_load(labels_t, list(fi).c_str());

    // NOTE to make sure that the data is not NAN
    for (uint i = 0; i < labels_t.N; i++)
      if (isnan(labels_t.p[i]))
      {
        std::cout << "NAN-Warning: " << list(fi).c_str() << std::endl;
        labels_t.p[i] = 0;
      }

    if (fi == 0) // NOTE make sure labels can be easily appended, cf. above
    {
      labels_t0.resize(0, labels_t.d2);
      labels_t1.resize(0, labels_t.d2);
      labels_t2.resize(0, labels_t.d2);
      labels_t3.resize(0, labels_t.d2);
    }
    else
      if (labels_t0.d1 != labels_t.d2) // append color to bw labels or vice versa
        np::msg_error(HERE, "Cannot append current labels to block, b/c of diff in d1");

    labels_t0.append(labels_t[0]);
    labels_t1.append(labels_t[1]);
    labels_t2.append(labels_t[2]);
    labels_t3.append(labels_t[3]);
  }


  labels.resize(4,labels_t0.d0,labels_t0.d1);
  uint p = 0;
  for (uint i = 0; i < labels_t0.N; i++)
    labels.p[p++] = labels_t0.p[i];
  for (uint i = 0; i < labels_t0.N; i++)
    labels.p[p++] = labels_t1.p[i];
  for (uint i = 0; i < labels_t0.N; i++)
    labels.p[p++] = labels_t2.p[i];
  for (uint i = 0; i < labels_t0.N; i++)
    labels.p[p++] = labels_t3.p[i];

  std::cout << "labels = " << std::endl << labels << std::endl;

  return 0;

}


/*! \brief Compute texture histograms
 *
 *  @see quantization(), ucm_gen_tree(), img_texturedesc()
 *  @param hist  histogram of visual codewords, size: (N,nc), N>0, nc>0
 *  @param xysc  quantized texture features, size: (M, 4), M>0
 *  @param tree  UCM segmentation tree with N>0 patches
 *
 *  @return 0 on OK
 */
int np::ucm_texturehist
(
 floatA& hist,
 const floatA& xysc,
 uint num_codewords,
 const seg::UcmTree& tree
)
{
  intA levels, patchmap, leaves;
  uint num_levels, num_patches = tree.nodes.N, num_xysc = xysc.d0;
  uint patch_idx;
  float row_sum;

  if (num_xysc <= 0)
    np::msg_error(HERE, "no quantized texture features given");
  if (xysc.d1 != 4)
    np::msg_error(HERE, "quantized texture features must have 4 dim., but they don't");

  seg::ucm_tree_levels(levels, tree);
//   std::cout << levels << std::endl;
  num_levels = levels.N;

  // get patchmap at each tree level and determine the patch-wise grid point
  // distribution
  hist.resize(num_patches, num_codewords);
  hist = 0;
  for (uint level = 0; level < num_levels; level++)
  {
    // patchmap at current level
    seg::ucm_patchmap(patchmap, leaves, tree, levels(level));

    // find grid point distribution
    for (uint xyi = 0; xyi < num_xysc; xyi++)
    {
      patch_idx = patchmap(xysc(xyi,1), xysc(xyi,0));
      hist(patch_idx-1, xysc(xyi,3)) += 1.0;
    }
  }

  // normalize each row to a sum of 1, if at least one entry is > 0
  for (uint row = 0; row < num_patches; row++)
  {
    row_sum = sum(hist[row]);
    if (row_sum > 0)
      for (uint col = 0; col < hist.d1; col++)
        hist(row,col) = hist(row,col) / row_sum;
  }

  return 0;
}

/*! \brief Load texture histograms
 *
 *  @param hist      histogram of visual codewords, size: (N,nc), N>0, nc>0
 *  @param filename  path and filename
 *
 *  @return 0 on OK, cf. parameter hist
 */
int np::ucm_texturehist_load(floatA& hist, const char* filename)
{
  std::ifstream is;
  is.open(filename);
  if (!is)
    np::msg_error(HERE, "cannot open file");

  hist.readTagged(is, "texturehist_floatA");

  is.close();
  return 0;
}

/*! \brief Save texture histograms
 *
 *  @param hist      histogram of visual codewords, size: (N,nc), N>0, nc>0
 *  @param filename  path and filename
 *
 *  @return 0 on OK
 */
int np::ucm_texturehist_save(const floatA& hist, const char* filename)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  hist.writeTagged(os, "texturehist_floatA", true);

  os.close();
  return 0;
}

/*! \brief Save final feature of an UCM tree
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
int np::ucm_final_features
(
 floatA& ff,
 const seg::UcmTree& tree,
 const floatA& texture,
 const floatA& color_HS,
 const floatA& color_V,
 const floatA& shape,
 const floatA& centers,
 const floatA& sizes
)
{
  uint num_nodes = tree.nodes.N, num_dim;
  float* pff;

  // NOTE it is assumed, that all features have been normalized

  // make sure that d0 of all input arguments matches the number of nodes in the
  // tree
  if ((texture.d0 != num_nodes)  ||
      (color_HS.d0 != num_nodes) ||
      (color_V.d0 != num_nodes)  ||
      (shape.d0 != num_nodes)    ||
      (centers.d0 != num_nodes)  ||
      (sizes.d0 != num_nodes))
  {
    std::cout << "num_nodes   = " << num_nodes << std::endl;
    std::cout << "texture.d0  = " << texture.d0 << std::endl;
    std::cout << "color_HS.d0 = " << color_HS.d0 << std::endl;
    std::cout << "color_V.d0  = " << color_V.d0 << std::endl;
    std::cout << "shape.d0    = " << shape.d0 << std::endl;
    std::cout << "centers.d0  = " << centers.d0 << std::endl;
    std::cout << "sizes.d0    = " << sizes.d0 << std::endl;
    np::msg_error(HERE, "at least one dimension d0 of input args not equal tree.nodes.N");
  }

  // TODO how can neighbor features be added to each feature?

  // concatenate all features
  num_dim = texture.d1 + color_HS.d1 + color_V.d1 + shape.d1 + centers.d1 + sizes.d1;
  ff.resize(num_nodes, num_dim);
  pff = ff.p;
  for (uint row = 0; row < num_nodes; row++)
  {
    for (uint dim = 0; dim < texture.d1; dim++)
      *pff++ = texture(row, dim);
    for (uint dim = 0; dim < color_HS.d1; dim++)
      *pff++ = color_HS(row, dim);
    for (uint dim = 0; dim < color_V.d1; dim++)
      *pff++ = color_V(row, dim);
    for (uint dim = 0; dim < shape.d1; dim++)
      *pff++ = shape(row, dim);
    for (uint dim = 0; dim < centers.d1; dim++)
      *pff++ = centers(row, dim);
    for (uint dim = 0; dim < sizes.d1; dim++)
      *pff++ = sizes(row, dim);
  }

  return 0;
}

/*! \brief Load final features of an UCM tree
 *
 *  @param ff        concatenated, normalized features
 *  @param filename  path and filename
 *
 *  @return 0 on OK, cf. parameter ff
 */
int np::ucm_final_features_load(floatA& ff, const char* filename)
{
  std::ifstream is;
  is.open(filename);
  if (!is)
    np::msg_error(HERE, "cannot open file");

  ff.readTagged(is, "final_features_floatA");

  is.close();
  return 0;
}

/*! \brief Save final features of an UCM tree
 *
 *  @param ff        concatenated, normalized features
 *  @param filename  path and filename
 *
 *  @return 0 on OK
 */
int np::ucm_final_features_save(floatA& ff, const char* filename)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  ff.writeTagged(os, "final_features_floatA", true);

  os.close();
  return 0;
}


// hidden implementations
inline void np::init_hsv_lut(intA& lut, uint num_bins)
{
  uint num_entries = lut.N;
  uint bin_diff = num_entries/num_bins;
  uint bin_idx = 0, bin_p = bin_diff;
  for (uint i = 0; i < num_entries; i++)
  {
    if (((i+1) > bin_p) && (bin_idx+1 < num_bins))
    {
      bin_p   += bin_diff;
      bin_idx += 1;
    }
    lut(i) =  bin_idx;
//     std::cout << "i = " << std::setw(3) << i << ", bin = " << bin_idx << std::endl;
  }
}

inline void np::normalize_histogram(floatA& hist, float s)
{
  float sum_hist;
  for (uint row = 0; row < hist.d0; row++)
  {
    sum_hist = sum(hist[row]);
    if (sum_hist != s && sum_hist != 0.)
      for (uint col = 0; col < hist.d1; col++)
        hist(row,col) /= sum_hist*s;
  }
}

inline void np::unique_indices(intA& indices, const intA& data)
{
  int data_max = data.p[data.maxIndex()]+1;
  intA indices_t(data_max);
  uint num_indices;
  indices_t = 0;
  for (uint i = 0; i < data.N; i++)
    indices_t(data.p[i]) = 1;

  num_indices = sum(indices_t);
  indices.clear();
  for (int i = 0; i < data_max; i++)
  {
    // index 0 is the background, and index 255 is a VOC-specific ignore value,
    // cf. VOC docs
    if (i == 0)
      continue;
    if (i == 255)
      continue;

    // add current index if it has been counted in the for-loop above
    if (indices_t(i) != 0)
      indices.append(i);
  }
}
