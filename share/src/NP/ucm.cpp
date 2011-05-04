
#include "ucm.h"
#include "nputils.h"
#include "imgproc.h"
#include "cvutils.h"
#include "opencv_helper.h"

#if 0
template <class S>
inline void ucm_display(const char* name, const MT::Array<S>& p, uint wait=10)
{
  byteA p_temp;
  np::array2array(p_temp, p);
  cvShowImage(name, p_temp);
  cvWaitKey(wait);
};

template <class S>
inline void ucm_display_patch(const char* name, const MT::Array<S>& p, S index, uint wait=10)
{
  MT::Array<S> p_temp(p.d0, p.d1);
  for (uint i=0; i<p_temp.N; i++)
    if (p.p[i] == index)
      p_temp.p[i] = 255;
    else
      p_temp.p[i] = 0;
  ucm_display(name, p_temp, wait);
};
#endif

inline void ucm_assign_merger(intA& mergers, byteA& is_merged, uint a, uint b)
{
  int v;
  while(mergers(b)!=-1)
  {
    v = mergers(b);
    if (v!=-1) b=v;
  }
  if (a==b)
    return;

  mergers(a)=b;
  is_merged(a)=1;
};

void ucm_find_mergers(MT::Array<uintA>& mergers, const uintA& ucm2_cc, const PixelArray& edge_pixels)
{
  uint x=0,y=0;
  uint n=0,s=0,w=0,e=0,z=0;
  int a=0,b=0;
  uint num_patches = ucm2_cc.p[ucm2_cc.maxIndex()]-ucm2_cc.p[ucm2_cc.minIndex()];
//   uint num_neighbors=0;
  int old_head=-1, new_head=-1;
  byteA neighbors_temp(num_patches+1, num_patches+1), is_merged(num_patches+1), new_patches(num_patches+1);
  intA mergers_temp(num_patches+1);
  neighbors_temp=0;
  mergers_temp=-1;
  is_merged=0;
  new_patches=0;

  for (uint m=0; m<edge_pixels.d0; m++)
  {
    for (uint i=0; i<edge_pixels(m).d0; i++)
    {
      n=0;s=0;w=0;e=0;

      // transform pixel index back into x- and y-coordinates
      x = edge_pixels(m)(i) % ucm2_cc.d1;
      y = (edge_pixels(m)(i)-x) / ucm2_cc.d1;

      // check the indices of the neighboring patches of current pixel position
      if (y+1 < ucm2_cc.d0)   n = ucm2_cc(y+1,x);
      if (y > 0)              s = ucm2_cc(y-1,x);
      if (x+1 < ucm2_cc.d1)   e = ucm2_cc(y  ,x+1);
      if (x > 0)              w = ucm2_cc(y  ,x-1);
      if (e>w) {z=w; w=e; e=z;} // swap
      if (s>n) {z=n; n=s; s=z;} // swap
//       std::cout <<"n="<<n<<" s="<<s<<" w="<<w<<" e="<<e<<std::endl;

      // only combination of n-s or w-e are interesting. All other combinations
      // are always from the same patch.
      if (n>0&&s>0) {a=n;b=s;}
      else if (w>0&&e>0) {a=w;b=e;}

      // build an relationship list: two previously unmatched patches assigned
      // together. If now an other patch is to be assigned with either one of
      // the previous two, it should, instead of creating a new pair, be
      // to that pair yielding a 3-tuple, and so forth. Later patches belonging
      // to one tuple will be assigned one new index alltogether.
      if (is_merged(a)==0&&is_merged(b)==0) // both not assigned
      {
        mergers_temp(a) = b;
        is_merged(a)=1;
      }
      else if (is_merged(a)==0&&is_merged(b)>0) // add a to tuple of b
        ucm_assign_merger(mergers_temp, is_merged, a, b);
      else if (is_merged(a)>0&&is_merged(b)==0)
        ucm_assign_merger(mergers_temp, is_merged, b, a); // add be to tuple of a
      else // if (is_merged(a)>0&&is_merged(b)>0) // this should not happen
      {
        if (mergers_temp(a)!=mergers_temp(b))
        {
          new_head = (mergers_temp(a)<mergers_temp(b)?mergers_temp(a):mergers_temp(b));
          old_head = (mergers_temp(a)>mergers_temp(b)?mergers_temp(a):mergers_temp(b));
          for (uint i=1; i<mergers_temp.N; i++)
            if (mergers_temp(i) == old_head)
              mergers_temp(i) = new_head;
        }
        if (mergers_temp(a)!=mergers_temp(b))
          np::msg_error(HERE,"there is something very wrong ...");
      }
    }
  }

  for (uint i=1; i<mergers_temp.N; i++)
  {
    a = mergers_temp(i);
    if (a != -1)
    {
      while(mergers_temp(a)!=-1)
      {
        b = mergers_temp(a);
        if (mergers_temp(a) == b && mergers_temp(b) == a) // break possible loops
        {
          a = b;
          break;
        }
//         std::cout << "a = " << a << ", b = " << b << std::endl;
        if (b!=-1) a=b;
      }
      mergers_temp(i) = a;
      new_patches(a)=1;
    }
  }

  int counter=1;
  for (uint i=0; i<new_patches.N; i++)
    if (new_patches(i) == 1)
      new_patches(i) = counter++;

  mergers.resize(counter-1);
  for (uint i=0; i<new_patches.N; i++)
    if (new_patches(i) > 0)
      mergers(new_patches(i)-1).append(i);
  for (uint i=0; i<mergers_temp.N; i++)
    if (mergers_temp(i)>0)
      mergers(new_patches(mergers_temp(i))-1).append(i);
};

// TODO imgproc.h : pixel_intensities()
// determine the strength levels in a given UCM image
void ucm_strengths(uintA& strengths, const byteA& ucm)
{
  if (ucm.N==0 || ucm.nd!=2) np::msg_error(HERE, "input: wrong size");
  strengths.clear();

  // check occuring edges strengths
  boolA strengths_temp(256); strengths_temp = false;
  for (uint i=0; i<ucm.N; i++)
    if (ucm.p[i] > 0)
      strengths_temp(ucm.p[i]) |= true;

  // append one entry for each edge strength (sorted and w/o doubles)
  for (uint i=1; i<strengths_temp.N; i++)
    if (strengths_temp(i) == true)
      strengths.append(i);
};

// TODO imgproc.h : pixel_lists()
// generate a list of pixel indices with the same value as in each entry
// of <pixel_value>
template <class S>
void ucm_pixel_list
(
 PixelArray& pixel_list,
 const MT::Array<S>& map,
 const uintA& pixel_values
)
{
  uint num_pixel_values = pixel_values.N;
  S max_value = map.p[map.maxIndex()];
  intA lut(max_value+1);                       // pixel value -> pixel_list(i)
  lut = -1;
  pixel_list.clear();                           // make sure output is empty

  if (pixel_list.N != num_pixel_values)
    pixel_list.resize(num_pixel_values);

  // fill LUT: pointing from pixel value to element in pixel_list
  for (uint i=0; i<pixel_values.N; i++)
    lut(pixel_values(i)) = i;

  // now generate pixel lists
  uint p;
  int lutp;
  for (uint i = 0; i < map.N; i++)
  {
    p = map.p[i];
    lutp = lut(p);
    if (p>0 && lutp>=0)
      pixel_list(lutp).append(i);
  }
};

template <class S>
inline void ucm_ucm2toucm(MT::Array<S>& ucm, const MT::Array<S>& ucm2)
{
  uint width = ucm2.d1/2, height = ucm2.d0/2;
  if (width != ucm.d1 || height != ucm.d0)
    ucm.resize(ucm2.d0/2, ucm2.d1/2);

  // jump over every second row and every second column; this is like
  // Y = X(2:2:end,2:2:end) in Matlab
  S* ucmp = ucm.p;
  for (uint row = 1; row < ucm2.d0; row+=2)
    for (uint col = 1; col < ucm2.d1; col+=2)
      *ucmp++ = ucm2(row, col);
}

// assign new labels to p in ascending order starting at <start>
inline void ucm_relabel(uintA& p, uint start=1)
{
  uint counter = start;
  uint num_labels = p.p[p.maxIndex()]+1;
  uintA cc(num_labels), cc2(num_labels);
  cc = 0; cc2 = 0;

  // count all assigned label indices
  for (uint i=0; i<p.N; i++)
    if (p.p[i] > 0)
      cc(p.p[i]) |= 1;

  // re-enumerate label indices from <start> to <start>+<num_labels>
  for (uint i = 0; i < cc.N; i++)
    if (cc(i) > 0)
      cc(i) = counter++;

  // apply new labeling to pixels
  for (uint i = 0; i < p.N; i++)
    p.p[i] = cc(p.p[i]);
};

template <class S>
inline void ucm_replace_values(MT::Array<S> &map, const uintA& pixels, S value)
{
  uint num_pixels = pixels.N;
  for (uint i=0; i<num_pixels; i++)
    map.p[pixels.p[i]] = value;
};

np::UcmTree::UcmTree(byteA& ucm2)
{
  array2array(ucm2_,ucm2);

  // build tree levels: start with the ground level and work the way up to the top
  build_leaf_level();
  uint num_levels = strengths_.N;
  for (uint i=0; i<num_levels; i++)
    build_level(i);
  build_top_level();

//   for (uint i=0; i<num_levels; i++)
//     for (uint j=0; j < pyramid_(i)->nodes_ids.N; j++)
//     {
// //       ucm_display_patch<uint>("test", pyramid_(i)->map, pyramid_(i)->nodes_ids(j), 25);
//       std::cout << "i = " << i << " j = " << j << " ";
//       std::cout << nodes_(pyramid_(i)->nodes_ids(j))->region.N << std::endl;
//       if (nodes_(pyramid_(i)->nodes_ids(j))->region.p[nodes_(pyramid_(i)->nodes_ids(j))->region.maxIndex()] >= pyramid_(0)->map.N)
//       {
//         std::cout << "Neeeeeeeein!!!" << std::endl;
//         exit(0);
//       }
//     }
};

void np::UcmTree::prune(UcmTree& tnew, uint strength)
{};

void np::UcmTree::node_region(intA& pixels, uint index)
{};

void np::UcmTree::node_contour(intA& pixels, uint index)
{};

void np::UcmTree::build_leaf_level()
{
  ucm_strengths(strengths_, ucm2_);          // edge strengths in UCM image

  // (1) edge pixels at each strength level (needed to merge segments)
  ucm_pixel_list(edge_pixels_, ucm2_, strengths_);

  // (2) make binary image: if (pixel) > 0 then 1 else 0
  ucm2_binary_.resize(ucm2_.d0, ucm2_.d1);
  np::mkbinary(ucm2_binary_, ucm2_, 0);

  // (3) label connected components (cc) in binary image from (2)
  ucm2_cc_.resize(ucm2_.d0, ucm2_.d1);
  np::cclabel(ucm2_cc_, ucm2_binary_, 8, true);

  // (4) relabel cc to range from 1 to N
  ucm_relabel(ucm2_cc_, 1);

  // (5) map  = downsize map2 (factor 2)
  uintA ucm_cc;
  ucm_ucm2toucm(ucm_cc, ucm2_cc_);

  // check for each CC which pixel indices belong to it (regions assigned to
  // nodes)
  PixelArray cc_pixels;
  uint num_cc = ucm_cc.p[ucm_cc.maxIndex()];
  uintA cc_indices(num_cc);
  for (uint i=0; i<num_cc; i++)
    cc_indices(i) = i+1;
  ucm_pixel_list(cc_pixels, ucm_cc, cc_indices);
  ucm_pixel_list(cc2_pixels_, ucm2_cc_, cc_indices);

  // (6.1) set up first level of the UcmPyramid
  UcmPyramidLevel *uplevel = new UcmPyramidLevel();
  array2array(uplevel->map, ucm_cc);
  uplevel->strength = 0;

  // (6.1) build leaf nodes
  for (uint i=1; i<=num_cc; i++)
  {
    UcmNode* new_node = new UcmNode();
    new_node->index = i;
    new_node->level = 0;
    array2array(new_node->region, cc_pixels(i-1));
    ctrace(new_node->contour, uplevel->map, new_node->region(0));
    new_node->strength = 0;
    new_node->parent_idx = -1;
    new_node->children_ids.clear();           // leaf nodes don't have children
    nodes_.append(new_node);

    uplevel->nodes_ids.append(i+1);           // remember this node in the pyramid, too
//     std::cout << *new_node << std::endl;
//     byteA mymap(uplevel->map.d0, uplevel->map.d1); mymap=0;
//     for (uint z=0; z<new_node->contour.N;z++)
//       mymap.p[new_node->contour(z)] = 255;
//     ucm_display("mymap", mymap, 0);
  }
  pyramid_.append(uplevel);
#if defined(NP_DEBUG)
  std::cout << "number of leaf nodes: " << num_cc << std::endl;
#endif
};

void np::UcmTree::build_level(uint index)
{
  // find patches which will be merged ...
  MT::Array<uintA> mergers;
  ucm_find_mergers(mergers, ucm2_cc_, edge_pixels_.sub(index,index));
  if (mergers.N == 0)
    msg_error(HERE, "no edges found");

#if defined(NP_DEBUG)
  uint num_mergers=0;
  std::cout << "index = " << index << " ";
  for (uint i=0; i<mergers.d0; i++)
    num_mergers += mergers(i).d0;
  std::cout << "merging " << num_mergers << " patches to " << mergers.d0 << " new patches: " << mergers << std::endl;
#endif

  // prepare the new pyramid level
  UcmPyramidLevel *uplevel = new UcmPyramidLevel();
  array2array(uplevel->map, pyramid_(index)->map);
  uplevel->strength = strengths_(index);

  // and change their pixel values to index of newly created patch
  uint new_index, n1;
  for (uint i=0; i<mergers.d0; i++)
  {
    // add new nodes
    new_index = nodes_.N+1;
    UcmNode* new_node = new UcmNode();
    new_node->index = new_index;
    new_node->level = index+1;

    uintA region2;
    for (uint j=0; j<mergers(i).N; j++)
    {
      n1 = mergers(i)(j)-1;

      // update level map
      ucm_replace_values(uplevel->map, nodes_(n1)->region, new_index);
      // build region of new patch consisting of its children
      new_node->region.append(nodes_(n1)->region);
      // add this current node n1 as child
      new_node->children_ids.append(n1+1);
//       nodes_(n1)->parent_idx = new_node->index-1;

      region2.append(cc2_pixels_(n1));
      cc2_pixels_(n1).clear();

    }
    cc2_pixels_.append(region2);
//     ucm_display("t", uplevel->map, 0);

    // update the global ucm2_cc_ map
    ucm_replace_values(ucm2_cc_, region2, new_index);

    // TODO construct contours
//     ctrace(new_node->contour, uplevel->map, new_node->region(0));
    new_node->strength = strengths_(index);
    nodes_.append(new_node);

    uplevel->nodes_ids.append(new_index);     // remember this node in the pyramid, too

  };
  // construct contours
#if defined(NP_DEBUG)
  std::cout << "uplevel->nodes_ids = " << uplevel->nodes_ids << std::endl;
#endif
  uint idx=0, start=0;
  for (uint i=0; i<uplevel->nodes_ids.N; i++)
  {
    idx = uplevel->nodes_ids(i)-1;
    start = nodes_(idx)->region.p[nodes_(idx)->region.minIndex()];
    ctrace(nodes_(idx)->contour, uplevel->map, start);
  }

  // add this new layer
  pyramid_.append(uplevel);
};

void np::UcmTree::build_top_level()
{
  // establish the children -> parent relation; let each child node know its
  // parent node
  uint num_nodes = nodes_.N;
  UcmNode *parent, *child;
  for (uint i=0; i<num_nodes; i++)
  {
    parent = nodes_.p[i];
    for (uint j=0; j<parent->children_ids.N; j++)
    {
      child = nodes_(parent->children_ids(j)-1);
      child->parent_idx = parent->index;
    }
  }

  // prepare the top pyramid level which comprises the entire image region
  UcmPyramidLevel *uplevel = new UcmPyramidLevel();
  uint new_index = nodes_.N+1;
  uint strength = 256;
  uplevel->map.resizeAs(pyramid_(pyramid_.N-1)->map);
  uplevel->map = new_index;
  uplevel->strength = strength;
  UcmNode* new_node = new UcmNode();
  new_node->index = new_index;
  new_node->level = pyramid_.N;
  // TODO determine contour
  for (uint i=0; i<num_nodes; i++)
    if (nodes_(i)->parent_idx < 0)
    {
      new_node->children_ids.append(i+1);
      nodes_(i)->parent_idx = new_node->index;
      new_node->region.append(nodes_(i)->region);
    }

  ctrace(new_node->contour, uplevel->map, new_node->region.p[new_node->region.minIndex()]);
  new_node->strength = strength;
  new_node->parent_idx = -1;
  uplevel->nodes_ids.append(new_node->index);
  pyramid_.append(uplevel);
  nodes_.append(new_node);
};

std::ostream& operator<<(std::ostream& os, const np::UcmTree& t)
{
  for (uint i=0; i<t.pyramid_.d0; i++)
    os << "t.pyramid_("<<i<<") = " << t.pyramid_(i)->nodes_ids.d0 << std::endl;
  return os;
};

std::ostream& operator<<(std::ostream& os, const np::UcmNode& n)
{
  os << "UcmNode:" << std::endl;
  os << "  index            = " << n.index << std::endl;
  os << "  level            = " << n.level << std::endl;
  os << "  contour (length) = " << n.contour.d0 << std::endl;
  os << "  region (size)    = " << n.region.d0 << std::endl;
  os << "  strength         = " << n.strength << std::endl;
  if (n.parent_idx >= 0)
    os << "  parent (index)   = " << n.parent_idx << std::endl;
  else
    os << "  parent (index)   = (no parent)" << std::endl;
  os << "  children         = ";
  for (uint i=0; i<n.children_ids.d0; i++)
    os << n.children_ids(i) << " ";
  if (n.children_ids.d0==0)
    os << "(no children)";
  os << std::endl;
  return os;
};
