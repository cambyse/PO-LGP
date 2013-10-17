
#ifndef _SEGMENTATION_H
#define _SEGMENTATION_H

#include <Core/array.h>

namespace np {
enum OverlapType {
  EQUAL            = 0,
  CONTAINS         = 1,
  PARTOF           = 2,
  CONTAINSORPARTOF = 3
};

struct Patch
{
  uintA region;
  uintA contour;
  uint  class_idx;
};
typedef MT::Array<Patch*> PatchList;

class SegmentationGroundtruth
{
public:
  SegmentationGroundtruth(const uintA& map_obj, const uintA& map_cls, uint num_cls);
  SegmentationGroundtruth(const byteA& img_obj, const byteA& img_cls, uint num_cls);

  template <class S>
  void overlap(MT::Array<S>& o, const uintA& region, OverlapType t, bool classwise=true);

  uint                        num_cls_;       // number of classes in general
                                              //   problem, e.g. VOC has 20
                                              // (NB: index 0 is always background)

  uintA                       map_obj_;       // object-wise groundtruth
  PatchList                   patches_obj_;   // (NB: index 0 is always background)

  uintA                       map_cls_;       // class-wise groundtruth
  PatchList                   patches_cls_;   // (NB: index 0 is always background)
};

}; // namespace np
#endif