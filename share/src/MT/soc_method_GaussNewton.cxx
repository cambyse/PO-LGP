/*  Copyright 2009 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

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

#include "soc.h"
#include "util.h"

void soc::GaussNewtonSoc::init(SocSystemAbstraction& _sys,
                               double _tolerance, uint _display,
                               uint _scale){
  sys = &_sys;
  tolerance=_tolerance;
  display=_display;
  scale=_scale;
}

//===========================================================================
