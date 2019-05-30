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

#include <Kin/taskMaps.h>
#include <Kin/proxy.h>
#include <Kin/frame.h>

//===========================================================================

struct CarKinematic:TaskMap{

  CarKinematic( const std::string & object );

  virtual mlr::String shortTag(const mlr::KinematicWorld& G);

  virtual void phi(arr& y, arr& J, const WorldL& Gs, double tau, int t=-1) override;

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1) override;

  virtual uint dim_phi(const mlr::KinematicWorld& K) override;

private:
  static const uint dim_ = 1;
  std::string object_;
};
