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

/*! \file surf.h
    \brief SURF image features */

#ifndef NP_EXTERNAL_SURF_H
#define NP_EXTERNAL_SURF_H

#include <MT/array.h>

namespace np {
namespace wrappers {

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
void surf
(
  floatA& keyp,
  floatA& desc,
  const byteA& image,
  uint num_octaves = 4,
  uint num_intervals = 4,
  uint start = 2,
  float edge_threshold = 0.0004f,
  bool upright = true
);

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
void surf_descriptors
(
  floatA& desc,
  const byteA& image,
  const intA& points,
  const floatA& scales,
  bool upright = true
);

float surf_scale(int step);

void surf_load(doubleA& keyp, doubleA& desc, const char* filename);
void surf_save(const doubleA& keyp, const doubleA& desc, const char* filename);
} // namespace wrappers
} // namespace np

#endif