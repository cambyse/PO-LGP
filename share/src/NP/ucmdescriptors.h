
#ifndef _UCMDESCRIPTORS_H
#define _UCMDESCRIPTORS_H

#include <MT/array.h>
#include "ucm.h"

namespace np {
/** @brief Elliptic Fourier Descriptors (EFD) for an UCM tree
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
void ucm_descriptors_efd(MT::Array<S>& d, const UcmTree& tree, uint num_coef=20);

/** @brief HSV volor descriptors for patches in an UCM tree
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
void ucm_descriptors_hsv
(
 MT::Array<S>& d,
 const byteA& image,
 const UcmTree& tree,
 unsigned char h=10, unsigned char s=10, unsigned char v=10
);

/** @brief Gabor wavelet features for patches in an UCM tree
 *
 *  Computes for all N patches of an UCM segmentation tree features based
 *  on the intensities of Gabor wavelet responses.
 *
 *  @param d            binned responses, size: (N, length), row sum == 1
 *  @param image        gray image of UCM tree, size: (y,x)
 *  @param tree         UCM segmentation tree (N nodes)
 *  @param num_scales   number of wavelet scales [default: 4]
 *  @param num_orient   number of wavelet orientations [default: 6]
 *  @param sigma        Gaussian blur sigma [default: 0.5]
 *  @param freq         spatial frequency of wavelets [default: 1.41]
 *  @param num_bin      number of bin; final desc. length: num_bin*num_scales*num_orient;
 *                      [default: 10]
 *
 *  @return cf. parameter d
 */
template <class S>
void ucm_descriptors_gabor
(
 MT::Array<S>& d,
 const byteA& image,
 const UcmTree& tree,
 uint num_scales=4,
 uint num_orient=6,
 S sigma=0.5,
 S freq=1.41,
 uint num_bin=10
);

/** @brief Patch descriptors made of visual words based on SURF descriptors
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
void ucm_descriptors_surf
(
 MT::Array<S>& d,
 const byteA& image,
 const UcmTree& tree,
 const MT::Array<S>& codebook,
 uint aperture=1,
 uint num_scales=4,
 uint surf_scale=2
);
}; // namespace np
#endif