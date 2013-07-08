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

/** @file imgproc.h
    @brief Image Processing */

#ifndef _IMGPROC_H
#define _IMGPROC_H

#include <Core/array.h>

namespace np {
const int opp[8] = {4, 5, 6, 7, 0, 1, 2, 3}; //<! mapping from t to t_old
const int nids[8][2] = {{ 0, 1}, { 1, 1}, { 1, 0}, { 1,-1}, { 0,-1},
                        {-1,-1}, {-1, 0}, {-1, 1}}; //<! neighbor indices
/** @brief Helper for clabel(): determine next contour point s of p
 *
 *  This function is the implementation of the contour tracing algorithm
 *  described in
 *
 *    F. Chang et al., A linear-time component-labeling algorithm using contour
 *    tracing technique, Journal of Computer Vision and Image Understanding, 2004
 *
 *  The neighborhood of pixel s is defined as:
 *
 *    5 6 7
 *    4 s 0
 *    3 2 1
 *
 *  @param t         next point
 *  @param tn        neighbor index of t relative to s
 *  @param s         current position
 *  @param sn        neighbor index of s relative to predecessor of s
 *  @param binimg    binary input image, size: (y,x,1)
 *  @param internal  if true, internal contour, else, external contour
 *
 *  @return cf. params t, tn
 *
 *  @see ctrace()
 *
 *  @warning This function is hidden, i.e. only minor sanity checks are
 *           implemented.
 */
template <class S>
inline void tracer(int& t, int& tn, int s, int sn, const MT::Array<S>& binimg, bool internal)
{
  int sn_prev, start;
  S v = binimg.p[s];                                 // pixel value at point s
  int n, t_temp;
  int xcur, ycur, xnext, ynext;
  int width = binimg.d1, height = binimg.d0;
  bool found_next = false;

  xcur = s % width;
  ycur = floor(s / width);
//   std::cout << "ycur xcur = " << ycur << " " << xcur << std::endl;

  // determine starting position
  if (sn == -1)
  {
    start = (internal ? 7 : 3);
    sn_prev = -1;
  }
  else
  {
    sn_prev = opp[sn];
    start = (sn_prev + 2) % 8;
  }

  for (int nid = 0; nid < 8; nid++)
  {
    n = (start + nid) % 8;
    ynext = ycur + nids[n][0];
    xnext = xcur + nids[n][1];
    t_temp = ynext * width + xnext;

    // skip if not current point is not within area of the image
    if ((ynext < 0) || (ynext >= height) || (xnext < 0) || (xnext >= width))
      continue;

//     std::cout << "ynext xnext = " << ynext << " " << xnext << std::endl;

    if(binimg.p[t_temp] == v)
    {
      found_next = true;
      break;
    }
  }

  if (found_next)
  {
    t = t_temp;
    tn = n;
  }
  else
  {
    t = s;
    tn = sn;
  }
};

/** @brief Find an external or internal contour at a given point p
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
void ctrace(MT::Array<S>& contour, const MT::Array<S>& binimg, int s, bool internal=true);

/** @brief Connected component labeling of a binary image
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
void cclabel(uintA& lb, const byteA& b, int n = 8, bool sort = true);

/** @brief Make binary pixel values
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
void mkbinary(byteA& bw, const byteA& img, int t = 0);

/** @brief Generate an array of points on a regular grid
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
void regular_grid
(
 MT::Array<S>& points,
 S width,
 S height,
 S step,
 bool center = true
);

/** @brief Distortion displacement maps
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
int distortion_maps
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
);

/** @brief Map to reverse distortion displacements
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
int undistortion_maps
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
);

/** @brief Transforms RGB image to RGBA one
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
void rgb2rgba(byteA& rgba, const byteA& rgb, unsigned char a);


template <class S, class T>
inline void pixel_intensities(MT::Array<S>& intensities, const MT::Array<T>& map, S min_value=1)
{
  if (map.N==0 || map.nd!=2)
    return;
  uint max_map=map.p[map.maxIndex()];
  intensities.clear();

  // check occuring edges strengths
  boolA intensities_temp(max_map+1); intensities_temp = false;
  for (uint i=0; i<map.N; i++)
    if (map.p[i] >= min_value)
      intensities_temp(map.p[i]) |= true;

  // append one entry for each edge strength (sorted and w/o doubles)
  for (uint i=min_value; i<intensities_temp.N; i++)
    if (intensities_temp(i) == true)
      intensities.append(i);
};

/** @brief Determine lists of pixel indices given a set of pixel values
 *
 *  Generate one list of pixel indices for every supplied pixel value
 *
 *  @param lists         array of pixel lists
 *  @param map           e.g. a segmentation image, size: (y,x)
 *  @param pixel_values  values for which lists will be created
 *
 *  @return cf. parameters rgba
 *
 */
template <class S>
inline void pixel_lists
(
 MT::Array<MT::Array<S> >& lists,
 const MT::Array<S>& map,
 const MT::Array<S>& pixel_values,
 S min_value = 1
)
{
  uint num_pixel_values = pixel_values.N;
  S max_value = map.p[map.maxIndex()];
  intA lut(max_value+1);                       // pixel value -> pixel_list(i)
  lut = -1;
  lists.clear();                               // make sure output is empty

  if (lists.N != num_pixel_values)
    lists.resize(num_pixel_values);

  // fill LUT: pointing from pixel value to element in pixel_list
  for (uint i=0; i<pixel_values.N; i++)
    lut(pixel_values(i)) = i;

  // now generate pixel lists
  S p;
  int lutp;
  for (uint i = 0; i < map.N; i++)
  {
    p = map.p[i];
    lutp = lut(p);
    if (p>=min_value && lutp>=0)
      lists(lutp).append(i);
  }
};

class ImagePyramid
{
public:
  ImagePyramid(const byteA&, uint scales);
  MT::Array<byteA>        layers_;
};

} // namespace np

#endif
