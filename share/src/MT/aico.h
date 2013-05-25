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



/** \file aico.h
    \brief Approximate Inference Control */

#ifndef MT_aico_h
#define MT_aico_h

#include "socNew.h"

//#define ControlledSystem soc::SocSystemAbstraction

/** \brief Apprioximate Inference Control */
struct AICO {
  struct sAICO *self;
  
  AICO();
  AICO(ControlledSystem& sys);
  ~AICO();

  //initialization
  void init(ControlledSystem& sys); ///< reads parameters from cfg file
  void init_messages();
  void init_trajectory(const arr& x_init);
  void prepare_for_changed_task();
  void fix_initial_state(const arr& x_0);
  void fix_final_state(const arr& x_T);
  void shift_solution(int offset);
  
  //optimization
  double step();
  void iterate_to_convergence();

  //access
  arr q();
  arr& b();
  arr& v();
  arr& Vinv();
  double cost();
  double tolerance();
};


void AICO_multiScaleSolver(ControlledSystem& sys,
                           arr& x,
                           double tolerance,
                           uint display,
                           uint scalePowers);


//===========================================================================
//
// implementations
//

#ifdef  MT_IMPLEMENTATION
#  include "aico.cpp"
#endif

#endif
