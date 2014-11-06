/*  ---------------------------------------------------------------------
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
    -----------------------------------------------------------------  */

#include "soc.h"
#include <Gui/opengl.h>
#include <Ors/ors.h>

//===========================================================================
//
// SocSystem_Toy
//

// mass on a spring
struct soc::sSocSystem_Toy{
  double m, d; //mass and spring constant
  double x, v;  //current position and velocity
  double x0, v0;//initial position and velocity

  arr W, H, Q;
  uint T;
  double tau;
};

soc::SocSystem_Toy::SocSystem_Toy(){
  s = new sSocSystem_Toy;
  s->m=1.;
  s->d=1.3;
  s->x0=1.;
  s->v0=0.;

  s->W.setId(1);
  static MT::Parameter<double> hc("Hcost");
  static MT::Parameter<double> qn("Qnoise");
  s->H.setDiag(hc, 1);
  s->Q.setDiag(qn, 2);
  s->Q.setZero();
}

soc::SocSystem_Toy::~SocSystem_Toy(){
  delete s;
}

uint soc::SocSystem_Toy::get_T(){  return s->T; }
uint soc::SocSystem_Toy::nTasks(){ return 1; }
uint soc::SocSystem_Toy::qDim(){   return 1; }
uint soc::SocSystem_Toy::uDim(){   return 1; }
uint soc::SocSystem_Toy::yDim(uint i){ return 1; }
double soc::SocSystem_Toy::getTau(bool scaled){  return s->tau; }

void soc::SocSystem_Toy::getq0 (arr& q){ q.resize(1); q(0)=s->x0; }
void soc::SocSystem_Toy::getqv0(arr& x){ x.resize(2); x(0)=s->x0; x(1)=s->v0; }
void soc::SocSystem_Toy::getqv0(arr& q, arr& qd){ q.resize(1); q(0)=s->x0; qd.resize(1); qd(0)=s->v0; }
void soc::SocSystem_Toy::getW  (arr& W){ W=s->W; }
void soc::SocSystem_Toy::getH  (arr& H){ H=s->H; }
void soc::SocSystem_Toy::getQ  (arr& Q){ Q=s->Q; }

void soc::SocSystem_Toy::setq(const arr& q, uint t){
  CHECK_EQ(q.N,1, "");
  s->x=q(0);
  s->v=0.;
}

void soc::SocSystem_Toy::setx(const arr& x, uint t){
  CHECK_EQ(x.N,2, "");
  s->x=x(0);
  s->v=x(1);
}

void soc::SocSystem_Toy::setqv(const arr& q, const arr& qd, uint t){
  CHECK(q.N==1 && qd.N==1, "");
  s->x=q(0);
  s->v=qd(0);
}

void soc::SocSystem_Toy::setq0AsCurrent(){
  s->x0=s->x;
  s->v0=s->v;
}

void soc::SocSystem_Toy::getMF(arr& M, arr& F){
  M.resize(1, 1);
  M(0, 0)=s->m;
  F.resize(1);
  F(0)=-s->d*s->x;
}

void soc::SocSystem_Toy::getMinvF(arr& Minv, arr& F){
  Minv.resize(1, 1);
  Minv(0, 0)=1./s->m;
  F.resize(1);
  F(0)=-s->d*s->x;
}

bool soc::SocSystem_Toy::isConditioned(uint i, uint t){
  CHECK_EQ(i,0, "");
  return true;
}

void soc::SocSystem_Toy::getPhi(arr& phiq_i, uint i){
  CHECK_EQ(i,0, "");
  phiq_i.resize(1);
  phiq_i(0)=s->x;
}

void soc::SocSystem_Toy::getJqd(arr& jqd_i, uint i){
  CHECK_EQ(i,0, "");
  jqd_i.resize(1);
  jqd_i(0)=s->v;
}

void soc::SocSystem_Toy::getJJt(arr& J_i, arr& tJ_i, uint i){
  CHECK_EQ(i,0, "");
  J_i .resize(1, 1); J_i (0, 0)=1.;
  tJ_i.resize(1, 1); tJ_i(0, 0)=1.;
}

void soc::SocSystem_Toy::getTarget(arr& y_i, uint i, uint t){
  y_i.resize(1);
  y_i=0; //-1.;
}

void soc::SocSystem_Toy::getTargetV(arr& v_i, uint i, uint t){
  v_i.resize(1);
  v_i=0; //3.;
}

void soc::SocSystem_Toy::getPrecision(double& prec, uint i, uint t){
  static MT::Parameter<double> ep("endPrec");
  if(t==s->T-1) prec=ep;
  else prec=0.;
  //prec=0.;
}

void soc::SocSystem_Toy::getPrecisionV(double& prec, uint i, uint t){
  static MT::Parameter<double> ep("endPrec");
  if(t==s->T-1) prec=ep;
  else prec=0.;
  //prec=0.;
}



//===========================================================================
//
// problem implementations
//

void toyDrawEnv(void *p){
#ifdef MT_GL
  soc::sSocSystem_Toy *s = (soc::sSocSystem_Toy*)p;
  glStandardLight(NULL);
  glDrawAxes(1.);
  glTranslatef(0, 0, s->x);
  glDrawSphere(.1);
#else
  NIY;
#endif
}

void soc::setupOpenGL(SocSystem_Toy &soci){
  if(!soci.gl) soci.gl=new OpenGL;
  soci.gl->add(toyDrawEnv, soci.s);
  soci.gl->camera.focus(0, 0, .8);
}


void soc::createDynamicProblem(SocSystem_Toy &soci,
                          const char *ors_file,
                          double trajectory_time,
                          uint trajectory_steps){
  MT_MSG("*** TOY problem");
  soci.s->T=trajectory_steps;
  soci.s->tau=trajectory_time/trajectory_steps;
}
