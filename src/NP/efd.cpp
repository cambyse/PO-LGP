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

#include "nputils.h"
#include "efd.h"

/*! \brief Compute the Elliptic Fourier Descriptors for a contour
 *
 *  This function is the implementation of the EFD algorithm described in
 *
 *    Nixon M. and Alberto A., Feature Extraction & Image Processing,
 *    2nd edition, 2008, p. 301f
 *
 *  @param efd       Fourier descriptor for the given contour, size: (coef)
 *  @param contour   int array with the [x,y] coordinates of the contour, size: (N,2)
 *  @param coef      number of coefficients to keep, coef > 0
 *
 *  @return cf. parameter efd
 *
 *  @see ctrace(), cclabel()
 */
template <class S>
void np::efd_contour(MT::Array<S>& efd, const uintA& contour, uint coef)
{
  if (contour.N == 0)
    msg_error(HERE, "contour is empty");
  if (contour.nd != 2)
    msg_error(HERE, "contour is not a m-by-2 array");

  uint m = contour.d0;

  // set the number of coefficients to half the number of points, if initially 0
  if (coef == 0)
    coef = (m/2 > 0 ? m/2 : 1);

  // 1. compute Fourier coefficients
  MT::Array<S> ax(coef), bx(coef), ay(coef), by(coef);
  ax = 0; bx = 0; ay = 0; by = 0;
  S t = 2*cpi/m;
  S h1, h2;                                                      // helpers
  for (uint k = 0; k < coef; k++)
  {
    for (uint i = 0; i < m; i++)
    {
      h1 = (k+1)*t*i;
      ax(k) = ax(k) + contour(i,0) * cos(h1);
      bx(k) = bx(k) + contour(i,0) * sin(h1);
      ay(k) = ay(k) + contour(i,1) * cos(h1);
      by(k) = by(k) + contour(i,1) * sin(h1);
    }
    h2 = 2./(S)m;
    ax(k) = ax(k) * h2;
    bx(k) = bx(k) * h2;
    ay(k) = ay(k) * h2;
    by(k) = by(k) * h2;
  }

  // 2. compute rotation, translation, and scaling invariant Fourier descriptor
  efd.resize(coef);
  efd = 0;
  S denom_a = 1./((ax(0)*ax(0))+(ay(0)*ay(0)));
  S denom_b = 1./((bx(0)*bx(0))+(by(0)*by(0)));
  S numer_a, numer_b;
  for (uint k = 0; k < coef; k++)
  {
    numer_a = (ax(k)*ax(k)) + (ay(k)*ay(k));
    numer_b = (bx(k)*bx(k)) + (by(k)*by(k));
    efd(k) = sqrt(numer_a*denom_a) + sqrt(numer_b*denom_b);
  }
}

template void np::efd_contour(MT::Array<float>& efd, const uintA& contour, uint coef);
template void np::efd_contour(MT::Array<double>& efd, const uintA& contour, uint coef);