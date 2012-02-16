#ifndef MT_libcolorseg_h
#define MT_libcolorseg_h

#include <MT/array.h>

//----- patch analysis
void pch2img(byteA &img, const uintA &pch, floatA &pch_colormap);
void random_colorMap(floatA& pch_colormap, uint np);
uint incremental_patch_ids(uintA& pch);
void get_patch_colors(floatA& pch_col, byteA& img, uintA& pch, uint np);
void get_patch_centroids(doubleA& pch_cen, byteA& img, uintA& pch, uint np);

uint get_single_color_segmentation(uintA& segmentation,  // segmented image
				   const byteA& image,   // input image
				   float sigma = 0.75,   // (Gaussian!?) blurring factor
				   float k = 500,        // similarity threshold
				   int min = 200         // min. no. of pixels per segment
				   );

uint get_single_color_segmentation_rgb(uintA& segmentation,  // segmented image
				       byteA& rgb,           // avg. RGB values assigned to segments
				       const byteA& image,   // input image
				       float sigma = 0.75,   // (Gaussian!?) blurring factor
				       float k = 500,        // similarity threshold
				       int min = 200         // min. no. of pixels per segment
				       );

void get_patch_centers(arr& centers, const uintA& patches);

void patch_color_statistics(arr& stats, const uintA& patches, const byteA& image);

void colorize_patches(byteA& coloration, const uintA& patches, const arr& stats);

typedef MT::Array<uintA> MultiSegmentations;
void get_multiple_color_segmentations(MultiSegmentations& segmentations,  // scale-hierarchy of segmented input
				      const byteA& image,                 // input image
				      const arr& sigma,
				      const arr& k,
				      const intA& min
				      );

#endif

