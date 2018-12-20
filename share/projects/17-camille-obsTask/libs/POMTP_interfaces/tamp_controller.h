/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include <task_planner.h>
#include <motion_planner.h>

class TAMPController
{
public:
  TAMPController( TaskPlanner & tp, MotionPlanner & mp )
    : tp_( tp )
    , mp_( mp )
  {

  }

  Skeleton plan( uint maxIt, bool saveInformed, bool saveFinal, bool show, int secs = 0 );

private:
  TaskPlanner & tp_;
  MotionPlanner & mp_;
};
