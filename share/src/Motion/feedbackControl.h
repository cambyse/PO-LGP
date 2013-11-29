/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
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

#include "motion.h"

struct PDtask{
  TaskMap& map;
  MT::String name;
  bool active;
  double prec;

  arr y_ref, v_ref;      ///< immediate (next step) desired target reference
  double Pgain, Dgain;   ///< parameters of the PD controller or attractor dynamics
  double err, derr;

  PDtask(TaskMap* m):map(*m), active(true) {}

  void setTarget(const arr& yref, const arr& vref=NoArr);
  void setGains(double Pgain, double Dgain);
  void setGainsAsNatural(double decayTime, double dampingRatio);

  arr getDesiredAcceleration(const arr& y, const arr& ydot);
};

struct FeedbackMotionProblem {
  //engines to compute things
  ors::Graph *ors;
  SwiftInterface *swift;

  MT::Array<PDtask*> tasks;

  //-- transition cost descriptions
  arr H_rate_diag; ///< cost rate
  double tau; ///< duration of single step

  arr x_current, v_current; ///< memory for which state was set (which state ors is in)

  FeedbackMotionProblem(ors::Graph *_ors=NULL, SwiftInterface *_swift=NULL, bool useSwift=true);
  void loadTransitionParameters(); ///< loads transition parameters from cfgFile
  void setState(const arr& x, const arr& v);

  //adding task spaces
  PDtask* addTask(const char* name, TaskMap *map);

  void getTaskCosts(arr& phi, arr& J, arr& a); ///< the general (`big') task vector and its Jacobian
  arr operationalSpaceControl();

  //VectorFunction definitions
//  virtual void fv(arr& phi, arr& J, const arr& x);
};
