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

#include <MT/algos.h>

#include <limits>
#include "nputils.h"
#include "vocutils.h"

/*! \brief Load a segmentation mask
 *
 *  Loads a PNG-file and converts the colors to the respective indices, cf.
 *  color codes of the VOC challenge.
 *
 *  @param mask      the segmentation mask
 *  @param img       image with a RGB-coded segmentation mask, VOC: *.png
 *
 *  @return 0 on OK, cf. parameter mask
 */
void np::voc_segmask(uintA& mask, const byteA& img)
{
  uint num_lut = 256;
  uintA lut = np::voc_labelcolormap(num_lut), idx;

  if (img.nd != 3) msg_error(HERE, "make sure <img> is a color image (RGB)");
  if (img.N  == 0) msg_error(HERE, "make sure <img> is not empty");
  byteA img_t;
  array2array(img_t, img);
  mask.resize(img.d0, img.d1);
  mask=0;

  uint diff_t, diff, min_diff, min_index;
  for (uint i=0; i<img.N; i+=3)
  {
    min_diff = std::numeric_limits<uint>::max();
    min_index=0;

    // NOTE The VOC groundtruth have whitish areas, which are not counted in the
    // contest. Here, we crop them and set the according pixel to the background
    // value 0.
    if (img_t.p[i]==224 && img_t.p[i+1]==224 && img_t.p[i+2]==192)
      continue;

    for (uint j=0; j<num_lut; j++)
    {
      diff=0;
      diff_t = img_t.p[i] - lut(j,0);
      diff += diff_t * diff_t;
      diff_t = img_t.p[i+1] - lut(j,1);
      diff += diff_t * diff_t;
      diff_t = img_t.p[i+2] - lut(j,2);
      diff += diff_t * diff_t;

      if (diff < min_diff)
      {
        min_diff = diff;
        min_index = j;
      };

      if (min_diff == 0)                      // perfect, that's the index we needed!
        break;
    }
    mask.p[i/3] = min_index;
  };
}
