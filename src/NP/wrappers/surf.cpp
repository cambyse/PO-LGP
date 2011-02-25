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

#include "surf.h"
#include "../nputils.h"
#include "../cvutils.h"
#include <opencv/cv.h>
#include <surflib.h>

/*! \brief Extract SURF keypoints + descriptors from image
 *
 *  Description of columns of the SURF keypoint matrix:
 *    1: x-coordinate
 *    2: y-coordinate
 *    3: scale
 *    4: orientation
 *    5: sign of laplacian (-> fast matching)
 *    6: dx - frame-to-frame motion analysis
 *    7: dy - frame-to-frame motion analysis
 *    8: cluster index
 *
 *  @param keyp            SURF keypoints, size: N-by-8
 *  @param desc            SURF descriptors, size: N-by-64
 *  @param image           grayscale image, size: x-by-y-by-1
 *  @param num_octaves     number of octaves (downsizing) [4]
 *  @param num_intervals   number of intervals per octave (Gaussian blur) [4]
 *  @param start           index of octave level [2]
 *  @param edge_threshold  rejection criteria for keypoints [0.0004]
 *  @param upright         rotation invariance [true]
 *
 *  @return cf. parameters keyp, desc
 */
void np::wrappers::surf
(
  floatA& keyp,
  floatA& desc,
  const byteA& image,
  uint num_octaves,
  uint num_intervals,
  uint start,
  float edge_threshold,
  bool upright
)
{
  bool is_color = (image.nd == 3);
  if (is_color)
    np::msg_error(HERE, "SURF wants a grayscale image, not a color one");

  // create opencv image wrapper
  IplImage* image_cv =
    cvCreateImageHeader(cvSize(image.d1, image.d0), IPL_DEPTH_8U, 1);
  image_cv->origin = IPL_ORIGIN_TL;                   // image origin (top-left)
  image_cv->widthStep = image.d1;
  image_cv->imageData = (char*) image.p;

  // create SURF pois and descriptors
  IpVec keyp_t;
  surfDetDes(image_cv, keyp_t, upright, num_octaves, \
    num_intervals, start, edge_threshold);

  // copy interest points and descriptors to output
  IpVec::iterator iter;
  int num_keyp = keyp_t.size();
  keyp.resize(num_keyp, 8);
  desc.resize(num_keyp, 64);
  int i = 0, j = 0;
  for (iter = keyp_t.begin(); iter != keyp_t.end(); iter++)
  {
    // interest points
    keyp.p[i++] = (*iter).x;
    keyp.p[i++] = (*iter).y;
    keyp.p[i++] = (*iter).scale;
    keyp.p[i++] = (*iter).orientation;
    keyp.p[i++] = (*iter).laplacian;
    keyp.p[i++] = (*iter).dx;
    keyp.p[i++] = (*iter).dy;
    keyp.p[i++] = (*iter).clusterIndex;

    // descriptor
//    memcpy(j, (*iter).descriptor, sizeof((*iter).descriptor));
    for (int i = 0; i < 64; i++)
      desc.p[j++] = (double) (*iter).descriptor[i];
  }
  cvReleaseImageHeader(&image_cv);
}

/*! \brief Compute SURF descriptors for given set of points
 *
 *  Computes SURF descriptors for any set of points. No keypoints will be
 *  extracted.
 *
 *  @param desc     SURF descriptors, size: N-by-64 (gray), N-by-192 (color)
 *  @param image    an image, gray or color, size: x-by-y-by-{1,3}
 *  @param points   set of 2D points, size: N-by-2
 *  @param scales   scale or size of receptive field of SURF descriptors, N-by-1
 *  @param upright  rotation invariance [true]
 *
 *  @return cf. parameters desc
 */
void np::wrappers::surf_descriptors
(
  floatA& desc,
  const byteA& image,
  const intA& points,
  const floatA& scales,
  bool upright
)
{
  bool is_color = (image.nd == 3);
  int num_points = points.d0;
  floatA desc_cur;
  if ((points.nd != 2) || (points.N < 2))
    np::msg_error(HERE, "argument <points> has to be a N-by-2 array, N > 0");

  // compute descriptors
  if (is_color)
  {
    // extract the three color channels
    uint x = image.d1, y = image.d0, cntr = 0, cntg = x*y, cntb = 2*x*y;
    byteA channels(3, y, x);
    for (uint pxi = 0; pxi < image.N; pxi += 3)
    {
      channels.p[cntr++] = image.p[pxi];
      channels.p[cntg++] = image.p[pxi+1];
      channels.p[cntb++] = image.p[pxi+2];
    }

    // compute features for each channel
    desc.resize(num_points, 64*3);
    doubleA desc_t;
    byteA channel;
    for (int chi = 0; chi < 3; chi++)
    {
      channel.referTo(channels.p+(chi*x*y), x*y);
      channel.reshape(y, x);
      surf_descriptors(desc, channel, points, scales);

      for (uint dei = 0; dei < desc_t.d0; dei++)
        for (uint dii = 0; dii < 64; dii++)
          desc(dei, dii+(chi*64)) = desc_t(dei, dii);
    }
  }
  else
  {
    IpVec points_t;
    for (int kpi = 0; kpi < num_points; kpi++)
    {
      Ipoint ipt;
      ipt.x = (float) points(kpi,0);
      ipt.y = (float) points(kpi,1);
      ipt.scale = scales(kpi);
      points_t.push_back(ipt);
    }
    std::cout << "" << std::endl;

    IplImage* image_cv = cvCreateImageHeader(cvSize(image.d1, image.d0), IPL_DEPTH_8U, 1);
    image_cv->origin = IPL_ORIGIN_TL;             // image origin (top-left)
    image_cv->widthStep = (image.d2 == 3 ? 3 : 1) * image.d1;
    image_cv->imageData = (char*) image.p;
    surfDes(image_cv, points_t, false);
    cvReleaseImageHeader(&image_cv);

    // return the descriptor as double, not float
    desc.resize(num_points, 64);
    desc_cur.resize(1,64);
    for (int kpi = 0; kpi < num_points; kpi++)
      for (int dei = 0; dei < 64; dei++)
      {
        // TODO Due to homogenous areas in the image some descriptors could be
        // NANs. Until a better solution has been revised these NANs will be 
        // replaced with 0.0
        if (isnan(points_t[kpi].descriptor[dei]))
          desc(kpi,dei) = 0.;
        else
          desc(kpi,dei) = points_t[kpi].descriptor[dei];
      }
  }
}

float np::wrappers::surf_scale(int step)
{
  return ((float) step/7.5);
//  return ((0.3 * (float) step) - 0.6);
}

void np::wrappers::surf_load
(
 doubleA& keyp,
 doubleA& desc,
 const char* filename
)
{
  std::ifstream is;
  is.open(filename);
  if (!is)
    np::msg_error(HERE, "cannot open file");

  keyp.readTagged(is, "keyp_doublA");
  desc.readTagged(is, "keyp_doublA");

  is.close();
//   return 0;
}

void np::wrappers::surf_save
(
 const doubleA& keyp,
 const doubleA& desc,
 const char* filename
)
{
  std::ofstream os;
  os.open(filename);

  if(!os)
    np::msg_error(HERE, "Could not open file");

  keyp.writeTagged(os, "keyp_doubleA", true);
  desc.writeTagged(os, "desc_doubleA", true);

  os.close();
//   return 0;
}
