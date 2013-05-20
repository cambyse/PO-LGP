
#include "nputils.h"
#include "ucmdescriptors.h"
#include "efd.h"
#include "cvgabor.h"
#include "imgproc.h"
#include "opencv_helper.h"
#include <limits>

namespace np {
template <class S>
inline void enforce_sum_one(MT::Array<S>& d)
{
  S d_sum = sum(d);
  if (d_sum>0.)
  {
    d_sum = 1./d_sum;
    d *= d_sum;
  }
};
}; // namespace np

/** \brief Elliptic Fourier Descriptors (EFD) for an UCM tree
 *
 *  Computes for all N patches of an UCM segmentation tree the following
 *  shape descriptors:
 *    (1) Elliptic Fourier Descriptors, size: N-by-num_coef
 *
 *  @param d         descriptors, size: (N, num_coef), row sum == 1
 *  @param tree      UCM segmentation tree
 *  @param num_coef  number of Fourier coefficients to be used (default: 20)
 *
 *  @return cf. parameter d
 */
template <class S>
void np::ucm_descriptors_efd(MT::Array<S>& d, const UcmTree& tree, uint num_coef)
{
  uint num_nodes = tree.nodes_.N;
  uint num_patches = num_nodes;
  UcmNode *node_cur;
  uint length_perimeter;
  MT::Array<S> d_cur;
//   S d_norm;
  uintA contour;
  uint x,y,w=tree.pyramid_(0)->map.d1;

  if (num_coef == 0)
    msg_error(HERE, "Check the number of coefficients, must be > 0");
  d.resize(num_patches, num_coef);

  for (uint i=0; i<num_nodes; i++)
  {
    // edge pixels: transform from index in node_cur->contour to 2D coordinates
    node_cur = tree.nodes_(i);
    length_perimeter = node_cur->contour.N;
    contour.resize(length_perimeter,2);
    for (uint j=0; j<length_perimeter; j++)
    {
      x = node_cur->contour(j) % w;
      y = (node_cur->contour(j)-x) / w;
      contour(j,0)=x;
      contour(j,0)=y;
    };

    // compute EFD
    d_cur.referTo(&d.p[i*num_coef], num_coef);
    np::efd_contour(d_cur, contour, num_coef);

    // enforce sum of 1
    enforce_sum_one(d_cur);
  }
}
template void np::ucm_descriptors_efd(MT::Array<float>& d, const np::UcmTree& tree, uint num_coef);
template void np::ucm_descriptors_efd(MT::Array<double>& d, const np::UcmTree& tree, uint num_coef);

/** \brief HSV volor descriptors for patches in an UCM tree
 *
 *  Computes for all N patches of an UCM segmentation tree the following
 *  color descriptors:
 *    cf. Plath2009, ICML paper adapted for UCM trees
 *
 *  @param d        binned H+S and V values, size: (N, h*s+v), row sum == 1
 *  @param image    color image (HSV) of UCM tree, size: (y,x,3)
 *  @param tree     UCM segmentation tree (N nodes)
 *  @param h        number of bins for Hue,        0 < h <=255
 *  @param s        number of bins for Saturation, 0 < s <=255
 *  @param v        number of bins for Value,      0 < v <=255
 *
 *  @return cf. parameter d
 */
template <class S>
void np::ucm_descriptors_hsv
(
 MT::Array<S>& d,
 const byteA& image,
 const UcmTree& tree,
 unsigned char h, unsigned char s, unsigned char v
)
{
  if (image.nd != 3)
    msg_error(HERE, "make sure the image is a color image (HSV)");
  uint hs = h*s;                              // length of combined H and S bins
  SimpleBinning<float, float> sbh(0, 255, h), sbs(0, 255, s), sbv(0, 255, v);

  // final length of descriptor
  uint num_dim = hs+v;

  // apply simple binning to wavelet output
  uint num_nodes = tree.nodes_.N, start_hs, start_v, bin_hs, bin_v, p;
  d.clear();
  d.resize(num_nodes, num_dim);
  MT::Array<S> d_hs, d_v;
  for (uint i=0; i<num_nodes; i++)
  {
    // refer to memory in d, where output for current node will be written to
    start_hs = i*num_dim;
    start_v  = start_hs + hs;

    d_hs.referTo(&d.p[start_hs], hs); d_hs = 0.;
    d_v.referTo(&d.p[start_v], v); d_v = 0.;

    // bin the HSV pixels contained in the region of the current node
    const UcmNode *node = tree.nodes_.p[i];
    const uintA &region = node->region;
    for (uint j=0; j<region.N; j++)
    {
      p = region.p[j] * 3; // (*3) b/c its a color image, not gray
      bin_hs = h * sbs.bin(image.p[p+1]) + sbh.bin(image.p[p]); // HS is like a 2D array made 1D
      d_hs.p[bin_hs] += 1;
      bin_v = sbv.bin(image.p[p+2]);
      d_v.p[bin_v] += 1;
    }

    // enforce row-wise sum of 1
    enforce_sum_one(d_hs);
    enforce_sum_one(d_v);
    d_hs *= (S) 0.5; // so that it sums up to 0.5
    d_v  *= (S) 0.5; // so that it sums up to 0.5 --> 0.5+0.5=1.
  }
};
template void np::ucm_descriptors_hsv(MT::Array<float>&, const byteA&, const np::UcmTree&, unsigned char, unsigned char, unsigned char);
template void np::ucm_descriptors_hsv(MT::Array<double>&, const byteA&, const np::UcmTree&, unsigned char, unsigned char, unsigned char);

namespace np {
template <class S>
inline void ucm_gabor_binning
(
 MT::Array<S>& d,
 const MT::Array<S>& intensities,
 const uintA& region,
 np::SimpleBinning<S, S>& sb
)
{
  d=0.;
  uint bin;
  for (uint i=0; i<region.N; i++)
  {
    bin = sb.bin(intensities.p[region.p[i]]);
    d.p[bin]+=1.;
  }

  // enforce descriptor sum of 1
  enforce_sum_one(d);
};
}; // namespace np

/** \brief Gabor wavelet features for patches in an UCM tree
 *
 *  Computes for all N patches of an UCM segmentation tree features based
 *  on the intensities of Gabor wavelet responses.
 *
 *  @param d            binned H+S and V values, size: (N, length), row sum == 1
 *  @param image        gray image of UCM tree, size: (y,x)
 *  @param tree         UCM segmentation tree (N nodes)
 *  @param length       length of descriptors [default: 100]
 *  @param num_scales   number of wavelet scales [default: 4]
 *  @param num_orient   number of wavelet orientations [default: 6]
 *  @param sigma        Gaussian blur sigma [default: 0.5]
 *  @param freq         spatial frequency of wavelets [default: 1.41]
 *  @param num_octaves  number of levels in image scale space
 *  @param num_bin      number of bin; final desc. length: num_bin*num_scales*num_orient;
 *                      [default: 0 - no binning]
 *
 *  @return cf. parameter d
 */
template <class S>
void np::ucm_descriptors_gabor
(
 MT::Array<S>& d,
 const byteA& image,
 const UcmTree& tree,
 uint num_scales,
 uint num_orient,
 S sigma,
 S freq,
 uint num_bin
)
{
  if (image.nd != 2)
    msg_error(HERE, "make sure the image is a gray image");

  // prepape some simple binning
  SimpleBinning<S, S> sb(0., 255., num_bin);

  // final length of descriptor
  uint num_dim = num_bin*num_scales*num_orient;

  // compute the responses of the Gabor wavelets
  MT::Array<MT::Array<S> > intensities;
  np::gabor_wavelet(intensities, image, num_orient, num_scales, sigma, freq, CV_GABOR_MAG);

  // apply simple binning to wavelet output
  uint num_nodes = tree.nodes_.N, start;
  d.clear();
  d.resize(num_nodes, num_dim);
  MT::Array<S> d_current;
  for (uint i=0; i<num_nodes; i++)
  {
    const UcmNode *node = tree.nodes_.p[i];
    for (uint j=0; j<intensities.N; j++)
    {
      start = num_dim*i+num_bin*j;
      d_current.referTo(&d.p[start], num_bin);
      ucm_gabor_binning(d_current, intensities.p[j], node->region, sb);
    }
  }

  // enforce row-wise sum of 1
  S inv = 1./(num_scales*num_orient);
  d *= inv;
};
template void np::ucm_descriptors_gabor(MT::Array<float>&, const byteA&, const np::UcmTree&, uint, uint, float, float, uint);
template void np::ucm_descriptors_gabor(MT::Array<double>&, const byteA&, const np::UcmTree&, uint, uint, double, double, uint);


namespace np
{
template <class S, class T>
void ucm_surf_region(MT::Array<S>& d, const uintA& region, const MT::Array<T>& quantized, const uintA& grid_1D)
{
  uint num_region=region.N, num_grid = grid_1D.N;
  for (uint i=0, j=0; i<num_region && j<num_grid;)
  {
    if (region.p[i] < grid_1D.p[j])
      i++;
    else if (region.p[i] > grid_1D.p[j])
      j++;
    else if (region.p[i] == grid_1D.p[j])
    {
      d.p[quantized.p[j]] += (S) 1;
      i++;
      j++;
    }
    else
      msg_error(HERE, "this should never be the case");
  }
};
}; // namespace np

/** \brief Patch descriptors made of visual words based on SURF descriptors
 *
 *  Computes for all N patches of an UCM segmentation tree features based
 *  on SURF descriptors. Internally, a scale space pyramid with <num_scales>
 *  layers is constructed from the input image. In each layer a regular grid is
 *  constructed with a mesh aperture of <aperture> pixels. For each intersection
 *  point, the according SURF descriptor is computed (descriptor width is
 *  determined by the index of the current pyramid layer). Finally, the codebook
 *  entry with the shortest Euclidean distance is found (using a KD-tree) and
 *  the according bin in the codebook histogram is increased.
 *
 *  The length of the final descriptor L is determined by <num_scales> times the
 *  number of visual words in the codebook.
 *
 *  @param d            histogram of visual words, size: (N, L), row sum == 1
 *  @param image        gray image, size: (y,x)
 *  @param tree         UCM segmentation tree of input image (N nodes)
 *  @param codebook     visual codebook for SURFs, size: (C, {64, 128})
 *  @param aperture     mesh width of regular grid [default: 1]
 *  @param num_scales   number of scales in scale space pyramid [default: 4]
 *  @param surf_scale   scale of SURF descriptor [default: 2]
 *
 *  @return cf. parameter d
 */
template <class S>
void np::ucm_descriptors_surf
(
 MT::Array<S>& d,
 const byteA& image,
 const UcmTree& tree,
 const MT::Array<S>& codebook,
 uint aperture,
 uint num_scales,
 uint surf_scale=2
)
{
  if (aperture<=0)
    aperture=1;
  if (num_scales<=0)
    num_scales=1;
  if (codebook.N==0 || (codebook.d1!=64 && codebook.d1!=128) || codebook.d0==0)
    msg_error(HERE, "the codebook must a non-empty N-by-{64, 128} array");

  // image pyramid
  ImagePyramid ip(image, num_scales);

  uint num_codewords = codebook.d0;
  uint num_dim = num_scales * num_codewords;

  // build KD-tree
  MT::Array<S> codebook_temp;
  array2array(codebook_temp, codebook);
  flann kd(codebook_temp);

  // compute SURF descriptors and replace them by index of the corresponding visual word
  MT::Array<S> d_surf;
  MT::Array<intA> quantized(num_scales);
  np::SURF surf;
  floatA dists;
  MT::Array<uintA> grid(num_scales);
  MT::Array<uintA> grid_1D(num_scales);
  int knn=1;
  uint max_leafs=codebook.d0/10;
  for (uint i=0; i<num_scales; i++)
  {
    uintA &grid_cur = grid.p[i];
    uintA &grid_1D_cur = grid_1D.p[i];
    byteA &layer = ip.layers_.p[i];
    intA &quantized_cur = quantized.p[i];

    // setup a regular grid and compute SURF descriptors
    regular_grid(grid_cur, layer.d1, layer.d0, aperture, true);
    cvExtractSURF(d_surf, grid_cur, layer, surf, surf_scale);
#if defined(NP_DEBUG)
    std::cout << "SURF, scale = " << surf_scale << ", ";
    std::cout << "min_surf = " << d_surf.p[d_surf.minIndex()] << ", max_surf = " << d_surf.p[d_surf.maxIndex()] << std::endl;
    std::cout << d_surf[0] << std::endl;
#endif

    // translate 2D grid coordinates into 1D pixel indices
    uint num_points=grid_cur.d0;
    grid_1D_cur.resize(num_points);
    for (uint j=0; j<num_points; j++)
      grid_1D_cur.p[j] = layer.d1*grid_cur(j,1)+grid_cur(j,0);
#if defined(NP_DEBUG)
    std::cout << "max(grid_1D_cur) = " << grid_1D_cur.p[grid_1D_cur.maxIndex()] << std::endl;
#endif
    // match descriptors
    kd.knnSearch(d_surf, quantized_cur, dists, knn, cv::flann::SearchParams(max_leafs));
#if defined(NP_DEBUG)
    std::cout << "FLANN, codebook size = " << codebook.d0 << ", max_leaf = " << max_leafs << ", ";
    std::cout << "min_dist = " << dists.p[dists.minIndex()] << ", max_dist = " << dists.p[dists.maxIndex()] << std::endl;
    if ((uint)quantized_cur.p[quantized_cur.maxIndex()] >= num_codewords)
    {
      std::cout << "quantized_cur.p[quantized_cur.maxIndex()] = " << quantized_cur.p[quantized_cur.maxIndex()] << std::endl;
      std::cout << "num_codewords="<<num_codewords<<std::endl;
      exit(0);
    }
#endif
    d_surf.clear();
    dists.clear();
  }

  uint num_nodes = tree.nodes_.N;
  MT::Array<S> d_node, d_scale;
  d.resize(num_nodes, num_dim);
  for (uint i=0; i<num_nodes; i++)
  {
    const UcmNode *node = tree.nodes_.p[i];
    uintA region = node->region;

    d_node.referTo(&d.p[num_dim*i], num_dim);
    d_node = (S) 0;
    for (uint j=0; j<num_scales; j++)
    {
      intA &quantized_cur = quantized.p[j];
      if (quantized_cur.N ==0)
        std::cout << quantized_cur << std::endl;
      d_scale.referTo(&d_node.p[j*num_codewords], num_codewords);
      ucm_surf_region(d_scale, region, quantized_cur, grid_1D.p[j]);

      // enforce row-wise sum of 1
      enforce_sum_one(d_scale);

      // scale region for next layers, i.e. [x' y'] = [x/2 y/2] which equals
      // the line below for pixel indices
      region /= (uint) 4;
    }

    // enforce row-wise sum of 1
    enforce_sum_one(d_node);
  }
};
template void np::ucm_descriptors_surf(MT::Array<float>&, const byteA&, const np::UcmTree&, const MT::Array<float>&, uint, uint, uint);
template void np::ucm_descriptors_surf(MT::Array<double>&, const byteA&, const np::UcmTree&, const MT::Array<double>&, uint, uint, uint);