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

/** @file vocutils.h
    @brief Utility routines specifically for the VOC Challenge */

#ifndef _NP_VOCUTILS_H
#define _NP_VOCUTILS_H

#include <Core/array.h>

namespace np {
static const uint voc_num_classes = 20;
// static const char* voc_classes[] =
// {
//   "background",
//   "aeroplane",
//   "bicycle",
//   "bird",
//   "boat",
//   "bottle",
//   "bus",
//   "car",
//   "cat",
//   "chair",
//   "cow",
//   "diningtable",
//   "dog",
//   "horse",
//   "motorbike",
//   "person",
//   "pottedplant",
//   "sheep",
//   "sofa",
//   "train",
//   "tvmonitor"
//  };

/** @brief The VOC label colormap
 *
 *  Generates a Look-up-table (LUT). Each index corresponds to a class index
 *  and gives the corresponding color for image coloration.
 *
 *  @param size      number of elements in the LUT
 *
 *  @return an (size, 3)-LUT
 */
inline uintA voc_labelcolormap(uint size=256)
{
  uintA lut(size, 3);

  int id; // NOTE shouldn't this be uint?
  unsigned char r = 0, g = 0, b = 0;
  for (uint i = 0; i < size; i++)
  {
    id = i;
    r = 0; g = 0; b = 0;
    for (int j = 0; j < 7; j++)
    {
      r = r | ((id & 1) << (7-j));
      g = g | ((id & 2) << (7-j-1));
      b = b | ((id & 4) << (7-j-2));
      id = id >> 3;
    }
    lut(i,0) = (int) r; lut(i,1) = (int) g; lut(i,2) = (int) b;
  }

  return lut;
}

/** @brief Load a segmentation mask
 *
 *  Takes a PNG-file and converts the colors to the respective VOC class indices,
 *  cf. color codes of the VOC challenge.
 *
 *  @param mask      the segmentation mask
 *  @param img       image with a RGB-coded segmentation mask, VOC: *.png
 *
 *  @return 0 on OK, cf. parameter mask
 */
void voc_segmask(uintA& mask, const byteA& img);

} // namespace np

#endif