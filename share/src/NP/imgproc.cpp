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
#include <limits>
#include <map>
#include <cmath>
#include "nputils.h"
#include "imgproc.h"
#include "opencv_helper.h"

namespace np {
/*! \brief Prior neighbors for pixel I(row, col)
 *
 *  Returns the indices of the neighboring pixels above and to the left 
 *  (4-neighborhood) or the indices of the west, northwest, north, and northeast
 *  neighbor pixels.
 *
 *  @param pn   int array containing indices of prior neighbors (row*cols+col)
 *  @param b    binary image
 *  @param row  row of current pixel
 *  @param col  column of current pixel
 *  @param n    number of neighbors [4, 8]
 *
 *  @return cf. parameter pn
 *
 *  @warning This function is hidden, i.e. only minor sanity checks are 
 *           implemented.
 */
inline int prior_neighbors(uintA& pn, const byteA& b, int row, int col, int n)
{
  int rows = b.d0, cols = b.d1;
  int y,x,p;
  int N = (n == 4 ? 2 : 4);                         // number of neighbor checks

  // offset for pns: north, west, northwest, and northeast
  int dy[4] = {-1,  0, -1, -1};                         // offset in x-direction
  int dx[4] = { 0, -1, -1, +1};                         // offset in y-direction

  // make sure neighborhood set is empty initially
  pn.clear();

//   // NOTE there is a glitch using pn.append(*), so there is this work-around
//   // see below ...
//   if (pn.N != 4)
//     np::msg_error(HERE, "pn.N must be 4!!");
//   int p2 = 0;

  // check prior 4-/8-neighborhood pixels and add them if they are within
  // the image boundaries and have a value > 0
  for (int i = 0; i < N; i++)
  {
    x = col + dx[i]; y = row + dy[i];
    p = y * cols + x;
    if ((x >= 0 && x < cols) && (y >= 0 && y < rows))
      if (b.p[p] > 0)
        pn.append(p);
//         pn.p[p2++] = p;
  }

  // NOTE there is a glitch using pn.append(*), so there is this work-around
  // see above ...
//   p2--;
//   if (p2 > 3)
//     np::msg_error(HERE, "p2>3");

//   return p2;
  return 0;
};

/*! \brief Retrieves component labels of prior neighbors pn
 *
 *  @param pl  int array containing component labels of prior neighbors
 *  @param lb  binary image with currently labeled components
 *  @param pn  set of prior neighbors
 *
 *  @return cf. parameter pl
 *
 *  @warning This function is hidden, i.e. only minor sanity checks are 
 *           implemented.
 */
inline int prior_labels(uintA& pl, const uintA& lb, const uintA& pn, uint num_pn)
{
  uint min = std::numeric_limits<uint>::max();
  // make sure set is empty initially
  pl.clear();
//   if (pl.N != 4)
//     np::msg_error(HERE, "pl.N != 4");

  // no neighbors, hence no labels either
  if (num_pn == 0)
    return -1;

  // retrieve labels of neighbors
  for (uint i = 0; i < num_pn; i++)
  {
//     pl.p[i] = lb.p[pn(i)];
    pl.append(lb.p[pn(i)]);
    if (min > pl.p[i])
      min = pl.p[i];
  }

  return min;
};

/*! \brief Construct a union of two components/sets
 *
 *  @param x       label of first set
 *  @param y       label of second set
 *  @param parent  set of prior neighbors
 *
 *  @warning This function is hidden, i.e. only minor sanity checks are 
 *           implemented.
 */
inline void union_parent(int x, int y, uintA& parent)
{
  while (parent(x) != 0)
    x = parent(x);
  while (parent(y) != 0)
    y = parent(y);
  if (x != y)
    parent(y) = x;
};

/*! \brief Find parent label of a set
 *
 *  @param x       label of set
 *  @param parent  union_parent data structure
 *
 *  @return label of root parent
 *
 *
 *  @warning This function is hidden, i.e. only minor sanity checks are 
 *           implemented.
 */
inline int find_parent(int x, const uintA& parent)
{
  while (parent(x) != 0)
    x = parent(x);
  return x;
};
} // namespace np

/*! \brief Connected component labeling of a binary image
 *
 *  This function is a re-implementation of Matlab's bwlabel function. The
 *  implementation follows
 *
 *    Shapiro, L., and Stockman, G. (2002). Computer Vision. Prentice Hall.
 *    pp. 69–73. http://www.cse.msu.edu/~stockman/Book/2002/Chapters/ch3.pdf
 *
 *  @param lb    int array containing the labels, size: (y,x,1)
 *  @param b     binary input image, size: (y,x,1)
 *  @param n     number of neighbors used in filter, [4, 8 (default)]
 *  @param sort  sort labels from 1...N, edges 0
 *
 *  @return an unique component/patch labeling
 *
 *  @see mkbinary()
 */
void np::cclabel(uintA& lb, const byteA& b, int n, bool sort)
{
  uint cols = b.d1, rows = b.d0;
//   int v = b.p[b.maxIndex()];
  int label_counter = 1;
  int label;
  uintA pn(4), pl(4);                    // prior neighbor set, prior labels set
  uintA parent(1);                             // union-part, cf. Shapiro et al.
  parent = 0;

  if (b.N == 0)
  {
    std::cout << "ERROR: binary pixelmap has size 0" << std::endl;
    return;
  }

  // make sure number of neighbors is correct
  if (n != 4 && n != 8)
  {
    std::cout << "NOTE: n = " << n << " is not valid, setting n = 8" << std::endl;
    n = 8;
  }

  // prepare output
  if (lb.d0 != rows || lb.d1 != cols) // NOTE resize glitch
    np::msg_error(HERE, "make sure output has the right size!");
//     lb.resize(rows, cols);
//   lb = 0;

  // Pass 1 -- determine preliminary labels for components and organize them in
  // a tree
  uint num_pn = 0;
  for (uint row = 0; row < rows; row++)
  {
    for (uint col = 0; col < cols; col++)
    {
      // process current pixel, if it is labeled as foreground (i.e. > 0)
      if (b(row, col) > 0)
      {
        // get the prior neighbors of current pixel
//           num_pn = prior_neighbors(pn, b, row, col, n);
          prior_neighbors(pn, b, row, col, n);
          num_pn = pn.N;

        // if no pn were found, assign new label to current pixel
        if (num_pn <= 0)
        {
          lb(row, col) = label_counter++;
          parent.append(0);
        }
        else // assign label of neighbor with smallest label and add as parent
        {
          // assign label
// // NOTE changed this because of the resize-glitch
          prior_labels(pl, lb, pn, num_pn);
          label = pl.p[pl.minIndex()];
          lb(row, col) = label;
//           label = prior_labels(pl, lb, pn, num_pn);
//           lb(row, col) = label;

          // remember to combine the different components (done in Pass 2)
          for (uint i = 0; i < num_pn; i++)
            union_parent(label, pl(i), parent);
        }
      }
    }
  }

  // Pass 2 -- unify component labels exploit tree from Pass 1
  for (uint row = 0; row < rows; row++)
    for (uint col = 0; col < cols; col++)
      if (b(row,col) > 0)
        lb(row,col) = find_parent(lb(row,col), parent);

  // Pass 3 -- enumerate components from 1 to N (optional)
  if (sort)
  {
    label_counter = 1;
    int num_pass2_labels = lb.p[lb.maxIndex()]+1;
    intA cc(num_pass2_labels), cc2(num_pass2_labels);
    cc = 0; cc2 = 0;

    // count all assigned label indices
    for (uint i = 0; i < lb.N; i++)
      if (lb.p[i] > 0)
        cc(lb.p[i]) |= 1;

    // re-enumerate label indices from 1 to N
    for (uint i = 0; i < cc.N; i++)
      if (cc(i) > 0)
        cc(i) = label_counter++;

    // apply new labeling to pixels
    for (uint i = 0; i < lb.N; i++)
      lb.p[i] = cc(lb.p[i]);
//     label_counter = 1;
// //     int num_pass2_labels = lb.p[lb.maxIndex()]+1;
//     // NOTE changed this because of the resize-glitch
// //     intA cc(num_pass2_labels);//, cc2(num_pass2_labels);
// //     cc = 0; //cc2 = 0;
//     std::map<uint, uint> cc;
// 
//     // count all assigned label indices
//     for (uint i = 0; i < lb.N; i++)
//       if (lb.p[i] > 0)
//         cc[lb.p[i]] |= 1;
// 
//     // re-enumerate label indices from 1 to N
//     std::map<uint, uint>::iterator iter;
//     for (iter = cc.begin(); iter != cc.end(); iter++)
//       (*iter).second = label_counter++;
// 
//     // apply new labeling to pixels
//     for (uint i = 0; i < lb.N; i++)
//       lb.p[i] = cc[lb.p[i]];

// NOTE changed this because of the resize-glitch
//     // count all assigned label indices
//     for (uint i = 0; i < lb.N; i++)
//       if (lb.p[i] > 0)
//         cc.p[lb.p[i]] |= 1;

//     // re-enumerate label indices from 1 to N
//     for (uint i = 0; i < cc.N; i++)
//       if (cc.p[i] > 0)
//         cc.p[i] = label_counter++;
// 
//     // apply new labeling to pixels
//     for (uint i = 0; i < lb.N; i++)
//       lb.p[i] = cc.p[lb.p[i]];
  }
}

/*! \brief Make binary pixel values
 *
 *  Set pixels with values t to 1 all other to 0
 *
 *  @param bw   byte array with binary regions, size: (y,x,1)
 *  @param img  grayscale image, size: (y,x,1)
 *  @param t    threshold
 *
 *  @return cf. parameter bw
 *
 *  @see cclabel()
 */
void np::mkbinary(byteA& bw, const byteA& img, int t)
{
  int n = img.N;

  if (n == 0)
  {
    std::cout << "UCM is empty" << std::endl;
    return;
  }
  if (img.nd >= 2 && img.d2 > 1)
  {
    std::cout << "UCM has to have the size (ysize, xsize, 1)" << std::endl;
    return;
  }

  // init the output
  if(bw.d0 != img.d0 || bw.d1 != img.d1) // NOTE resize glitch
    np::msg_error(HERE, "make sure the output has the right size already!!");
//     bw.resize(img.d0, img.d1);
//   bw = 0;

  // set edges to zeros and original zeros to one
  for (int i = 0; i < n; i++)
    if (img.p[i] == t)
      bw.p[i] = 1;
}

namespace np {
} // namespace np

/*! \brief Find an external or internal contour at a given point p
 *
 *  This function is the implementation of the contour tracing algorithm
 *  described in
 *
 *    F. Chang et al., A linear-time component-labeling algorithm using contour
 *    tracing technique, Journal of Computer Vision and Image Understanding, 2004
 *
 *  @param contour   int array with indices of contour, to be used with binimg.p[]
 *  @param binimg    binary input image, size: (y,x,1)
 *  @param s         index of starting point, i.e. binimg.p[p]
 *  @param internal  if true, internal contour, else, external contour
 *
 *  @return cf. parameter contour
 *
 *  @see mkbinary(), cclabel()
 */
template <class S>
void np::ctrace(MT::Array<S>& contour, const MT::Array<S>& binimg, int s, bool internal)
{
  int t, t_old, t_cur;
  int sn, tn, tn_old, tn_cur;
  int iter = 0, iter_max = binimg.N;
  intA neighbor_ids;

  // check input
  if (binimg.N == 0 || binimg.nd != 2)
    msg_error(HERE, "binimg has the wrong size, check input", true);
  if (s < 0)
    msg_error(HERE, "s points outside the image, s < 0", true);

  // check output
  contour.clear();

  // start tracer
//   contour.append(s);                                      // first contour point
  sn = -1;
  tracer(t, tn, s, sn, binimg, internal);

  if (t == s) // s is an isolated point, we're done
  {
    contour.append(s);                                      // first and only contour point
    return;
  }

  // trace the rest of the contour
  contour.append(t);
  t_old = t;
  tn_old = tn;
  for (;;iter++)
  {
    // find the next piece of the contour
    tracer(t_cur, tn_cur, t_old, tn_old, binimg, internal);

    // check if the contour has been completed                          [tag:01]
    if ((t_old == s) && (t_cur == t))
      break;

    if (iter >= iter_max)
      msg_error(HERE, "ctrace: more iterations than pixels - this is bad!!", true);

    // and new piece to the contour
    contour.append(t_cur);

    t_old = t_cur;
    tn_old = tn_cur;
  }

  // for [tag:01] to be true, s has to have been appended twice, so we'll remove
  // one of them
//   contour.remove(contour.N-1);
};
template void np::ctrace(MT::Array<int>& contour, const MT::Array<int>& binimg, int s, bool internal);
template void np::ctrace(MT::Array<uint>& contour, const MT::Array<uint>& binimg, int s, bool internal);
template void np::ctrace(MT::Array<float>& contour, const MT::Array<float>& binimg, int s, bool internal);
template void np::ctrace(MT::Array<double>& contour, const MT::Array<double>& binimg, int s, bool internal);


/*! \brief Generate an array of points on a regular grid
 *
 *  @param points  array with 2D points, size: (N,2)
 *  @param width   image width > 0
 *  @param height  image height > 0
 *  @param step    grid width > 0
 *  @param center  translate grid to have same distance to edges on all sides
 *
 *  @return cf. parameter points
 *
 */
template<class S>
void np::regular_grid
(
 MT::Array<S>& points,
 S width,
 S height,
 S step,
 bool center
)
{
  if (width <= 0 || height <= 0 || step <= 0)
    np::msg_error(HERE, "Make sure that height, with, step, and margin have a value > 0");

  uint num_nodes_x = (uint) width/step;
  uint num_nodes_y = (uint) height/step;
  int margin_x = (center ? (width-step*(num_nodes_x-1))/2 : 0);
  int margin_y = (center ? (height-step*(num_nodes_y-1))/2 : 0);
  uint pi = 0;

  points.resize(num_nodes_x * num_nodes_y, 2);
  for (uint y = 0; y < num_nodes_y; y++)
    for (uint x = 0; x < num_nodes_x; x++)
    {
      points(pi, 0) = margin_x + step*x;
      points(pi, 1) = margin_y + step*y;
      pi++;
    }
}
template void np::regular_grid(MT::Array<int>&, int, int, int, bool);
template void np::regular_grid(MT::Array<uint>&, uint, uint, uint, bool);
template void np::regular_grid(MT::Array<float>&, float, float, float, bool);
template void np::regular_grid(MT::Array<double>&, double, double, double, bool);

/*! \brief Distortion displacement maps
 *
 *  Creates two displacement maps, one for the x- and one for y-coordinates,
 *  simulating Radial and Tangential Distortion.
 *
 *  Radial distortion is generated by applying the first few terms of a Taylor
 *  series expansion to each image pixel (x,y):
 *
 *    x' = x / (1.1+k1*r^2+k2*r^4+k3*r^6)
 *    y' = y / (1.1+k1*r^2+k2*r^4+k3*r^6)
 *
 *  where r is the distance of the current pixel to the image center and k1-3
 *  and distortion parameters.
 *
 *  Tangential distortion is generated similarly:
 *
 *    x' = x - (2*p1*y        + p2*(r^2+2*x^2))
 *    y' = y - (p1*(r^2+2y^2) + 2*p2*x)
 *
 *  This is basically the inverse map of OpenCV's cvInitUndistortMap().
 *
 *  @param mapx  distortion map for x-coordinates, size: (h,w)
 *  @param mapy  distortion map for y-coordinates, size: (h,w)
 *  @param w     image width
 *  @param h     image height
 *  @param k1    radial distortion factor
 *  @param k2    radial distortion factor
 *  @param k3    radial distortion factor
 *  @param p1    tangential distortion factor
 *  @param p2    tangential distortion factor
 *
 *  @return 0 on OK, cf. parameters mapx, mapy
 *
 */
int np::distortion_maps
(
 floatA& mapx,
 floatA& mapy,
 uint w,
 uint h,
 float k1,
 float k2,
 float k3,
 float p1,
 float p2
)
{
  float center[] = {((float)h)/2., ((float)w)/2.};
  float maxd = (w > h ? w : h);
  float xdist, ydist, dist;
  float xnorm, ynorm;
  float r, r2;

  // compute distortion maps
  mapx.resize(h,w);
  mapy.resize(h,w);
  for (uint y = 0; y < h; y++)
  {
    ynorm = (((float)y-center[0]) / maxd);      // center and normalize [-1, +1]
    for (uint x = 0; x < w; x++)
    {
      xnorm = (((float)x-center[1]) / maxd);    // center and normalize [-1, +1]
      r = sqrt(xnorm*xnorm + ynorm*ynorm);               // distance from origin
      r2 = r*r;

      // apply radial distortion
      dist = 1/(1+k1*r2+k2*pow(r,4)+k3*pow(r,6));
      xdist = xnorm * dist;
      ydist = ynorm * dist;

      // apply tangential distortion
      dist = 2*p1*ynorm            + p2*(r2+2*xnorm*xnorm);
      xdist -= dist;
      dist = p1*(r2+2*ynorm*ynorm) + 2*p2*xnorm;
      ydist -= dist;

      // store distortion value for current pixel (+unnormalize and uncenter)
      mapx(y,x) = xdist * maxd + center[1];
      mapy(y,x) = ydist * maxd + center[0];
    }
  }

  return 0;
}

/*! \brief Map to reverse distortion displacements
 *
 *  Creates two displacement maps, one for the x- and one for y-coordinates,
 *  compensating Radial and Tangential Distortion.
 *
 *  Radial distortion is generated by applying the first few terms of a Taylor
 *  series expansion to each image pixel (x,y):
 *
 *    x = x' * (1.1+k1*r^2+k2*r^4+k3*r^6)
 *    y = y' * (1.1+k1*r^2+k2*r^4+k3*r^6)
 *
 *  where r is the distance of the current pixel to the image center and k1-3
 *  and distortion parameters.
 *
 *  Tangential distortion is similarly compensated for:
 *
 *    x = x + (2*p1*y        + p2*(r^2+2*x^2))
 *    y = y + (p1*(r^2+2y^2) + 2*p2*x)
 *
 *  @see cvInitUndistortMap().
 *
 *  @param mapx  distortion map for x-coordinates, size: (h,w)
 *  @param mapy  distortion map for y-coordinates, size: (h,w)
 *  @param w     image width
 *  @param h     image height
 *  @param k1    radial distortion factor
 *  @param k2    radial distortion factor
 *  @param k3    radial distortion factor
 *  @param p1    tangential distortion factor
 *  @param p2    tangential distortion factor
 *
 *  @return 0 on OK, cf. parameters mapx, mapy
 *
 */
int np::undistortion_maps
(
 floatA& mapx,
 floatA& mapy,
 uint w,
 uint h,
 float k1,
 float k2,
 float k3,
 float p1,
 float p2
)
{
  float center[] = {((float)h)/2., ((float)w)/2.};
  float maxd = (w > h ? w : h);
  float xdist, ydist, dist;
  float xnorm, ynorm;
  float r, r2;

  // compute distortion maps
  mapx.resize(h,w);
  mapy.resize(h,w);
  for (uint y = 0; y < h; y++)
  {
    ynorm = (((float)y-center[0]) / maxd);      // center and normalize [-1, +1]
    for (uint x = 0; x < w; x++)
    {
      xnorm = (((float)x-center[1]) / maxd);    // center and normalize [-1, +1]
      r = sqrt(xnorm*xnorm + ynorm*ynorm);               // distance from origin
      r2 = r*r;

      // apply radial distortion
      dist = (1+k1*r2+k2*pow(r,4)+k3*pow(r,6));
      xdist = xnorm * dist;
      ydist = ynorm * dist;

      // apply tangential distortion
      dist = 2*p1*ynorm            + p2*(r2+2*xnorm*xnorm);
      xdist += dist;
      dist = p1*(r2+2*ynorm*ynorm) + 2*p2*xnorm;
      ydist += dist;

      // store distortion value for current pixel (+unnormalize and uncenter)
      mapx(y,x) = xdist * maxd + center[1];
      mapy(y,x) = ydist * maxd + center[0];
    }
  }

  return 0;
};

/*! \brief Transforms RGB image to RGBA one
 *
 *  NOTE: needed this to add byteA to revel videos.
 *
 *  @param rgba  the RGBA image, size: (h,w,4)
 *  @param rgb   the RGB image, size: (h,w,3)
 *  @param a     opacity value
 *
 *  @return cf. parameters rgba
 *
 */
void np::rgb2rgba(byteA& rgba, const byteA& rgb, unsigned char a)
{
  rgba.resize(rgb.d0, rgb.d1, 4);
  unsigned char *p = rgb.p;
  for (uint i = 0; i < rgba.N;)
  {
    rgba.p[i++] = *p++;
    rgba.p[i++] = *p++;
    rgba.p[i++] = *p++;
    rgba.p[i++] = a;
  }
};


np::ImagePyramid::ImagePyramid(const byteA& image, uint scales)
{
  if (scales==0)
    scales=1;
  layers_.resize(scales);
  array2array(layers_.p[0], image);
  uint scale=2;
  for (uint i=1; i<scales; i++)
  {
    if (image.nd == 3)
      layers_.p[i].resize(image.d0/scale, image.d1/scale, 3);
    else
      layers_.p[i].resize(image.d0/scale, image.d1/scale);
    cvResize(layers_.p[i], image);
    scale*=2;
  }
};