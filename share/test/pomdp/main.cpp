/*
    Copyright (C) 2007, 2008  Marc Toussaint.
    email: mtoussai@cs.tu-berlin.de

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    A copy of the GNU Lesser General Public License can usually be found
    at http://www.gnu.org/copyleft/lesser.html; if not, write to the Free
    Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
    02110-1301 USA
*/

#define MT_IMPLEMENTATION
#define MT_NoLognormScale

#ifdef MT_QT
#  include"gui.h"
#endif
#include <MT/mdp_EMSolver.h>


int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  uint mode;
  MT::getParameter(mode,"mode",(uint)9);
  
  mdp::EMSolver sol;
  sol.getParameters();
  
#ifdef MT_QT
  QApplication *app;
  Gui *gui;
#endif
  char buf[256];
  ifstream is;
  
  switch(mode){
  case 0:
    MT::open(is,"README");
    while(is.good()){ is.getline(buf,256); cout <<buf <<endl; }
    break;
    case 1: {
      cout <<"\n*** file conversion..." <<endl;
      readMDP(sol.mdps,sol.problemFile);
      cout <<"\n*** MDP:" <<endl;    mdp::writeMDP_fg(sol.mdps,cout,true);
      cout <<"\nfull normalization of MDP = " << checkNormalization(sol.mdps) <<" (should be cardinality of action variables)" <<endl;
      ofstream z1(sol.problemFile+".mdp_fg");
      mdp::writeMDP_fg(sol.mdps,z1,false);
      mdp::MDP mdp_flat;
      mdp::collapseToFlat(mdp_flat,sol.mdps);
      mdp::writeMDP_arr(mdp_flat,sol.problemFile+".mdp_arr");
    } break;
  case 2: NIY; break;
  case 3:
    sol.reportParameters(cout);
    sol.initProblem();
    //sol.initFsc();
    sol.obsolete_loop_lev12();
    break;
  case 4:
    sol.reportParameters(cout);
    sol.initProblem();
    sol.initFsc();
    cout <<"\n*** MDP:" <<endl;    mdp::writeMDP_fg(sol.mdps,cout,true);
    cout <<"\n*** FSC:" <<endl;    mdp::writeFSC_fg(sol.fsc,cout,true);
    cout <<endl;
    checkJointNormalization(sol.mdps,sol.fsc);
    if(false){//for debugging
      //checkNormalization(sol.mdps);
      ofstream z1("z.fsc");
      mdp::writeFSC_fg(sol.fsc,z1,false);
      ofstream z("z.mdp_fg");
      mdp::writeMDP_fg(sol.mdps,z,false);
      mdp::MDP mdp_flat;
      mdp::collapseToFlat(mdp_flat,sol.mdps);
      mdp::writeMDP_arr(mdp_flat,"z.mdp_arr");
    }
    sol.loop();
    break;
  case 9:
#ifndef MT_QT
    HALT("gui only works when compiling with QT");
#else
    app = new QApplication(argn, argv);
    gui = new Gui;
    gui->exec();
    delete gui;
    delete app;
#endif
    break;
  default:
    NIY;
  }
  
  return 0;
}


