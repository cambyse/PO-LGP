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

#include <policy.h>

class TaskPlanner
{
public:
    typedef std::shared_ptr< TaskPlanner > ptr;

public:
    virtual void setFol( const std::string & folDescription ) = 0;
    virtual void solve() = 0;
    virtual void integrate( const Policy::ptr & policy ) = 0;

    virtual bool terminated() const = 0;
    virtual Policy::ptr getPolicy() const = 0;
    virtual MotionPlanningOrder getPlanningOrder() const = 0;
};
