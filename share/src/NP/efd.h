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

/*! \file efd.h
    \brief Elliptic Fourier Descriptors (EFD) */

#include <MT/array.h>

namespace np {
/*! \brief Compute the Elliptic Fourier Descriptors for a contour
 *
 *  This function is the implementation of the EFD algorithm described in
 *
 *    Nixon M. and Alberto A., Feature Extraction & Image Processing,
 *    2nd edition, 2008, p. 301f
 *
 *  @param efd       Fourier descriptor for the given contour, size: (coef)
 *  @param contour   int array with the [x,y] coordinates of the contour, size: (N,2)
 *  @param coef      number of coefficients to keep, if 0 then it's set to N/2
 *
 *  @return cf. parameter efd
 *
 *  @see ctrace(), cclabel()
 */
template <class S>
void efd_contour(MT::Array<S>& efd, const uintA& contour, uint coef);

}; // namespace np
