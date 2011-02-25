#ifndef _UCM_H
#define _UCM_H

#include <MT/array.h>

typedef MT::Array<uintA>      PixelArray;

namespace np {
struct UcmNode
{
  UcmNode() : parent_idx(-1) {};
  uint                        index;          // index of this segment/node
  uint                        level;          // level of this node in UcmPyramid
  uintA                       contour;        // contour pixels of this node
  uintA                       region;         // region pixels of this node

  uint                        strength;       // strength of UCM edge to parent
  int                         parent_idx;     // node index of parent node
  uintA                       children_ids;   // node indices of children nodes
//   UcmNode*                    parent;         // pointer to parent node
//   MT::Array<UcmNode*>         children;       // pointers to children nodes
};
typedef MT::Array<UcmNode*>   UcmNodeList;

struct UcmPyramidLevel {
  uintA                       map;            // map with segments
  uint                        strength;       // min. edge strength in map
  uintA                       nodes_ids;      // indices of nodes at cur. level
};
typedef MT::Array<UcmPyramidLevel*> UcmPyramid;

class UcmTree {
public:
  UcmTree(byteA &ucm2);                       // binary double-size UCM image

  void prune(UcmTree& tnew, uint strength);   // new tree w/ edges above <strength>
  void node_region(intA& pixels, uint index);
  void node_contour(intA& pixels, uint index);
  uint num_nodes() {return nodes_.d0;};

  UcmPyramid                  pyramid_;
  UcmNodeList                 nodes_;

  byteA                       ucm2_;          // double-size UCM image
  uintA                       strengths_;     // edge strengths
  PixelArray                  edge_pixels_;   // indices of edge pixels for
                                              //   each edge strength
  PixelArray                  cc2_pixels_;    // pixel of segments in ucm2_cc_
  byteA                       ucm2_binary_;
  uintA                       ucm2_cc_;

private:
  void                        build_leaf_level();
  void                        build_level(uint index);
  void                        build_top_level();
};
}; // namespace np

std::ostream& operator<<(std::ostream& os, const np::UcmTree& t);
std::ostream& operator<<(std::ostream& os, const np::UcmNode& n);
#endif
