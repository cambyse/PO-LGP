
#include "nputils.h"
#include "segmentation.h"
#include "imgproc.h"
#include "vocutils.h"
#include "opencv_helper.h"

template <class S>
inline void ucm_display(const char* name, const MT::Array<S>& p, uint wait=10)
{
  byteA p_temp;
  np::array2array(p_temp, p);
  cvShowImage(name, p_temp);
  cvWaitKey(wait);
};

namespace np {
void seg_build(PatchList &patches, const uintA& map)
{
  uintA intensities;
  MT::Array<uintA> lists;

  // object map: determine regions and contours
  pixel_intensities(intensities, map, (uint) 0);
  pixel_lists(lists, map, intensities, (uint) 0);
  uint num_obj = lists.N, start;
  for (uint i=0; i<num_obj; i++)
  {
    Patch *patch = new Patch();               // create a new patch
    array2array(patch->region, lists(i));     // add its region

    // determine the contour given the region
    start = patch->region.p[patch->region.minIndex()];
    ctrace(patch->contour, map, start);

    // assign the class label
    patch->class_idx = map.p[start];

    // add new patch to list
    patches.append(patch);
  }
};
}; // namespace np

np::SegmentationGroundtruth::SegmentationGroundtruth(const uintA& map_obj, const uintA& map_cls, uint num_cls)
 : num_cls_(num_cls)
{
  array2array(map_obj_, map_obj);
  array2array(map_cls_, map_cls);

  // object map: determine regions and contours
  seg_build(patches_obj_, map_obj_);

  // class map: determine regions and contours
  seg_build(patches_cls_, map_cls_);
};

np::SegmentationGroundtruth::SegmentationGroundtruth(const byteA& img_obj, const byteA& img_cls, uint num_cls)
 : num_cls_(num_cls)
{
  uintA map_obj, map_cls;
  np::voc_segmask(map_obj, img_obj);
  np::voc_segmask(map_cls, img_cls);
  array2array(map_obj_, map_obj);
  array2array(map_cls_, map_cls);

  // object map: determine regions and contours
  seg_build(patches_obj_, map_obj_);

  // class map: determine regions and contours
  seg_build(patches_cls_, map_cls_);
};

template <class S> void np::SegmentationGroundtruth::overlap
(
 MT::Array<S>& o,
 const uintA& region,
 OverlapType t,
 bool classwise
)
{
  uint num_dim = (classwise ? num_cls_+1 : patches_obj_.N);
  uintA& map = (classwise ? map_cls_ : map_obj_);
  PatchList& patches =  (classwise ? patches_cls_ : patches_obj_);

  if (o.N != num_dim)
    o.resize(num_dim);
  o=0.;

  // count absolute pixel overlap: A \cap B_i
  uint size_region = region.N;
  for (uint i=0; i<size_region; i++)
    o(map.p[region(i)]) += 1.;

  // normalize overlaps according to request type
  S inv = 1.;
  switch (t)
  {
    case EQUAL:                               // == |A \cap B| / |A \cup B|
      for (uint i=0; i<patches.N; i++)
      {
        if (patches(i)->region.N==0 || size_region==0)
          continue;
        inv = 1./(float)(size_region+patches(i)->region.N-o(i));
        o(patches(i)->class_idx) *= inv;
      }
      break;
    case CONTAINS:                            // == |A \cap B| / |A|
      for (uint i=0; i<patches.N; i++)
        o(patches(i)->class_idx) *= (patches(i)->region.N > 0 ? 1. / (float) patches(i)->region.N : 1.);
      break;
    case PARTOF:                              // == |A \cap B| / |B|
      inv = (size_region > 0 ? 1. / (float) size_region : 1.);
      o *= inv;
      break;
    case CONTAINSORPARTOF:                    // == |A \cap B|
      break;
    default:
      break;
  }
};
template void np::SegmentationGroundtruth::overlap(MT::Array<float>&, const uintA&, np::OverlapType, bool);
template void np::SegmentationGroundtruth::overlap(MT::Array<double>&, const uintA&, np::OverlapType, bool);
