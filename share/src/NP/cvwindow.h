/*  Copyright 2010 Nils Plath
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

/** @file cvwindow.h
    @brief Window/GUI class. */

#ifndef NP_CVWINDOW_H
#define NP_CVWINDOW_H

#include <string>
#include <MT/array.h>

namespace np {
class CvWindow
{
public:
  CvWindow(const char* name, uint w=10, uint h=10, uint posx=10, uint posy=10);
  ~CvWindow();
  void                    draw_image(const byteA& img);
  void                    draw_caption(const std::string& caption);
  void                    update(double scale=1.0);

//protected:
  std::string             name_;
  byteA                   buffer_;
  std::string             caption_;
};

};
#endif
