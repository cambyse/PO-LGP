//
// interface to Felzenszwalb's implementation of unsupervised image segmentation
//
#ifndef LIBS_LIBCOLORSEG_LIBCOLORSEG_H
#define LIBS_LIBCOLORSEG_LIBCOLORSEG_H

#include "MT/array.h"

namespace vision_low {

// Unsupervised graph-cut-based segmentation (single-scale)
//
// INPUT
//    const byteA& image         - zi input image
//    float sigma                - a blurring factor to pre-smoothen the image
//    float k                    - pixel-similarity threshold
//    int min                    - minimum size of segments
//
// OUTPUT
//    uintA& segmentation        - segmented input image
//
// NOTE
//  The segment indices of the output are enumerated from 0...N, i.e.
//  the number of segments can be queried by
//    uint num_segments = segmentation.p[segmentation.maxIndex()];
//
uint                    get_single_color_segmentation(
                                                      uintA& segmentation,  // segmented image
                                                      const byteA& image,   // input image
                                                      float sigma = 0.75,   // (Gaussian!?) blurring factor
                                                      float k = 500,        // similarity threshold
                                                      int min = 200         // min. no. of pixels per segment
                                                     );

// Unsupervised graph-cut-based segmentation (multi-scale)
//
// INPUT
//    const byteA& image         - zi input image
//    floatA& sigma              - an array of blurring factors to pre-smoothen the image
//    floatA& k                  - an array of pixel-similarity thresholds
//    intA& min                  - an array of minimum sizes of segments
//
// NOTE
//    "sigma.N == k.N == min.N"
//
// OUTPUT
//    MultiSegmentations& segmentation - scale hierarchy of segmented input image
//
// NOTE
//  The segment indices of the output are enumerated from 0...N, i.e.
//  the number of segments at scale level i can be queried by
//    uint num_segments = segmentations.p[i].p[segmentation.maxIndex()];
//
typedef MT::Array<uintA> MultiSegmentations;
void                    get_multiple_color_segmentations(
                                                         MultiSegmentations& segmentations,  // scale-hierarchy of segmented input
                                                         const byteA& image,                 // input image
                                                         const doubleA& sigma,
                                                         const doubleA& k,
                                                         const intA& min
                                                        );
} // namespace vision_low

#endif

