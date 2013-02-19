/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
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
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
/**
 * @file
 * @ingroup group_soc
 */

#ifndef MT_soc_inverseKinematics_h
#define MT_soc_inverseKinematics_h

#include "soc.h"

namespace soc {
//===========================================================================
/**
 * @defgroup soc_ik_control Inverse Kinematics Control
 * @ingroup group_soc
 * @{
 */
//void bayesianIKControl(SocSystemAbstraction& soci, arr& dq, uint t);
void pseudoIKControl(SocSystemAbstraction& soci, arr& dq, uint t, double regularization=1e-8);
void hierarchicalIKControl(SocSystemAbstraction& soci, arr& dq, uint t, double regularization=1e-8);
void bayesianIterateIKControl(SocSystemAbstraction& soci,
                              arr& qt, const arr& qt_1, uint t, double eps, uint maxIter);
void bayesianIKTrajectory(SocSystemAbstraction& soci, arr& q, double eps=-1);
void bayesianDynamicControl(SocSystemAbstraction& soci, arr& x, const arr& x_1, uint t, arr *v=NULL, arr *Vinv=NULL);
void bayesianIKControl2(SocSystemAbstraction& soci, arr& q , const arr& q_1 , uint t, arr *v=NULL, arr *Vinv=NULL);

/** @} */
}

#endif
