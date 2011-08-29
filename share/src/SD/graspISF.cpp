#include "graspISF.h"
#include <MT/plot.h>
#include <MT/util.h>
#include "utils.h"
#include "graspObjects.h"

void
configure_GraspISF(GraspISFTask* t){

  t->tv_palm_prec_m =             MT::getParameter<double>("grasp_tv_palm_prec_m", 1e3);
  t->tv_palm_trgt_m =             MT::getParameter<double>("grasp_tv_palm_trgt_m", .05);
  t->tv_oppose_prec =             MT::getParameter<double>("grasp_tv_oppose_prec", 1e1);
  t->tv_zeroLevel_prec_m =        MT::getParameter<double>("grasp_tv_zeroLevel_prec_m", 4e2);
  t->tv_fingAlign_prec_m =        MT::getParameter<double>("grasp_tv_fingAlign_prec_m", 1e3);
  t->tv_fingAlign_sin_m =         MT::getParameter<double>("grasp_tv_fingAlign_sin_m", 3.1415);
  t->tv_tipAlign_prec_m =         MT::getParameter<double>("grasp_tv_tipAlign_prec_m", 1e3);
  t->tv_q_v_prec =                MT::getParameter<double>("grasp_tv_q_v_prec", 2e-1);
  t->tv_q_y_prec =                MT::getParameter<double>("grasp_tv_q_y_prec", 1e-1);

  t->tv_skin_trgt =               MT::getParameter<double>("grasp_tv_skin_trgt", .02);
  t->tv_skin_fake =               MT::getParameter<double>("grasp_tv_skin_fake", .01);
  t->tv_skin_prec_thr_zeroLevel = MT::getParameter<double>("grasp_tv_skin_prec_thr_zeroLevel", .5);
}

GraspISFTask::GraspISFTask(){
  grip = false;
  graspobj = NULL;
  t = 0;
  T = 0;
  TV_tipAlign = NULL;
  TV_fingAlign = NULL;
  TV_palm = NULL;
  TV_oppose = NULL;
  TV_zeroLevel = NULL;

  configure_GraspISF(this);
}

double
GraspISFTask::phiAtFrame(ors::Transformation& X, arr &grad,double *sig){
  arr x;
  double _sig;
  x.setCarray(X.pos.p,3);
  double phi = graspobj->phi(NULL,NULL,&_sig,x);
  graspobj->getNormGrad(grad,x);
  if (sig)  *sig = _sig;
  return phi;
}

void
GraspISFTask::initTaskVariables(ControllerProcess *ctrl){
  TaskAbstraction::initTaskVariables(ctrl);
  
  // finger tip endeffector
  tipsN.append(ctrl->ors.getShapeByName("tipNormal1"));
  tipsN.append(ctrl->ors.getShapeByName("tipNormal2"));
  tipsN.append(ctrl->ors.getShapeByName("tipNormal3"));

  // finger tip endeffector
  skins.append((double)ctrl->ors.getBodyByName("tip3")->index);
  skins.append((double)ctrl->ors.getBodyByName("tip1")->index);
  skins.append((double)ctrl->ors.getBodyByName("tip2")->index);
  skin_prec = MT::Parameter<double>("TV_skin_yprec");

  // finger tip endeffector
  fingsN.append(ctrl->ors.getShapeByName("fingNormal1"));
  fingsN.append(ctrl->ors.getShapeByName("fingNormal2"));
  fingsN.append(ctrl->ors.getShapeByName("fingNormal3"));

  // wrist endeffector
  palm = ctrl->ors.getShapeByName("palmCenter");

  // finger tipsN orientation (aligned with gradient)
  TV_tipAlign = new PotentialFieldAlignTaskVariable("tips z align",ctrl->ors, tipsN, *graspobj);
  TV_tipAlign->y_prec = 50;
  TV_tipAlign->setGains(.1,.0);
  TVs_all.append(TV_tipAlign);

  TV_fingAlign = new PotentialFieldAlignTaskVariable("fings z align",ctrl->ors, fingsN, *graspobj);
  TV_fingAlign->y_prec = 50;
  TV_fingAlign->setGains(.01,.0);
  TVs_all.append(TV_fingAlign);

  // palm orientation
  /*
  TV_palmAlign = new PotentialFieldAlignTaskVariable(STRING("fings z align "<<s->name),ctrl->ors, TUPLE(), *graspobj);
  TV_palm_z->setGains(ori_gain,0);
  TV_palm_z->y_prec = 50;
  TVs_all.append(TV_palm_z);
  */
  
  // palm position
  TV_palm = new TaskVariable();
  TV_palm->set("palm pos",ctrl->ors, posTVT,
	       palm->body->index,palm->rel, -1, ors::Transformation(),ARR());
  TV_palm->y_prec = 50;
  TV_palm->setGains(.1,.0);
  TVs_all.append(TV_palm);

  TV_oppose = new zOpposeTaskVariable("oppose",ctrl->ors, tipsN);
  TV_oppose->setGains(.1,.0);
  TVs_all.append(TV_oppose);

  TV_zeroLevel = new PotentialValuesTaskVariable("zeroLevel", ctrl->ors, tipsN, *graspobj);
  TV_zeroLevel->setGains(.1,.0);
  TV_zeroLevel->y_target = ARR(0,0,0); 
  TVs_all.append(TV_zeroLevel);

  TV_skin = new TaskVariable("skin", ctrl->ors, skinTVT,0,0,0,0,skins);
  //TV_skin->targetType=gainsTT;
  TV_skin->targetType=directTT;
  TV_skin->v_prec=0;
  TV_skin->y_target=ARR(tv_skin_trgt,tv_skin_trgt,tv_skin_trgt);
  TVs_all.append(TV_skin);

  
  TVs_all.append(TV_q);
  TVs_all.append(TV_col);
  TVs_all.append(TV_lim);
  ctrl->sys.setTaskVariables(TVs_all);

}

void
GraspISFTask::updateTaskVariables(ControllerProcess *ctrl){
  activateAll(TVs_all,false); //deactivate all variables
  ctrl->useBwdMsg=false;      //deactivate use of bwd messages (from planning)

  if (grip) {
    TV_palm->active   = true;
    TV_palm->y_target(2) = 1.1;
    TV_palm->y_prec   = 5e1;

    TV_lim->active            = true;
    TV_col->active            = true;
    TV_q->active            = true;

    return;
  }

  arr nabla_fx_t, tip_nabla_fx_t, fing_nabla_fx_t,
      tip_sig_a(3), tip_var_a(3),
      rel_tip_var_a(3), var_m(3);
  double phi,phi_tip, phi_fing;
  uint i;
  
  phi = phiAtFrame(palm->X, nabla_fx_t,NULL);

  // palm position
  TV_palm->active   = true;
  TV_palm->y_target = TV_palm->y - nabla_fx_t * tv_palm_trgt_m;
  TV_palm->y_prec   = tv_palm_prec_m * (phi>0.?phi:0.); //care only away from surface

  // oppose fingers
  TV_oppose->active  = true;
  TV_oppose->y_target = 0;
  TV_oppose->y_prec = tv_oppose_prec; // * (phi>0.?phi:0.); //care only away from surface;

  phi_tip = ( phiAtFrame(tipsN(0)->X, tip_nabla_fx_t,tip_sig_a.p+0) 
      + phiAtFrame(tipsN(1)->X, tip_nabla_fx_t,tip_sig_a.p+1)
      + phiAtFrame(tipsN(2)->X, tip_nabla_fx_t,tip_sig_a.p+2)
      )/3;
  phi_fing = ( phiAtFrame(fingsN(0)->X, fing_nabla_fx_t, NULL)
      +phiAtFrame(fingsN(1)->X, fing_nabla_fx_t, NULL)
      +phiAtFrame(fingsN(2)->X, fing_nabla_fx_t, NULL)
      )/3;

  mult(tip_var_a, tip_sig_a, tip_sig_a);
  if(graspobj->max_var() > 0 )
    rel_tip_var_a = tip_var_a / graspobj->max_var();
  else
    rel_tip_var_a.setZero();
  var_m.setZero(); var_m +=1.; var_m -= rel_tip_var_a;
  SD_DBG1("max var: "<<graspobj->max_var()<<
      " tips' std deviation: " << tip_sig_a <<
      " tips' var: " << tip_var_a <<
      " variance fraction: "<<rel_tip_var_a <<
      " variance multiplier: "<<var_m);

  // position to surface (phi_tip=0)
  TV_zeroLevel->active  = true;
  /* set targetdependent on variance (large vaeiance-->less
   * confidence--> target near current; i.e. instead of precision we
   * alter targets)
   */
  mult(TV_zeroLevel->y_target, TV_zeroLevel->y, var_m); 
  //TV_zeroLevel->y_target = ARR(0,0,0);
  TV_zeroLevel->y_prec =
    tv_zeroLevel_prec_m  * (phi_tip>0?(1.-phi_tip):0); //care only at surface

  TV_fingAlign->active  = true;
  TV_fingAlign->y_target = ARR(-1.,-1.,-1.);
  //TV_fingAlign->y_prec = tv_fingAlign_prec_m * (phi_fing>0.?phi_fing:0.); //care only if palm is away from surface;
  //TV_fingAlign->y_prec = SD_MAX(5e3 * sin(phi_fing*3.), 0); //care in the middle
  //TV_fingAlign->y_prec = SD_MAX(tv_fingAlign_prec_m * sin(phi_fing*MT_PI), 0); //care in the middle
  TV_fingAlign->y_prec =
    SD_MAX(tv_fingAlign_prec_m * sin(phi_fing*tv_fingAlign_sin_m), 0); //care in the middle
  //TV_fingAlign->y_prec = SD_MAX(tv_fingAlign_prec_m  * sin(phi_fing*2.5), 0); //care in the middle

  // align tipsN with gradient
  TV_tipAlign->active  = true;
  TV_tipAlign->y_target = ARR(-1.,-1.,-1.);
  TV_tipAlign->y_prec =
    tv_tipAlign_prec_m * (phi_tip>0?(1.-phi_tip):0); //care only at surface


  // direct q control (hony pot & comfortable)
  TV_q->active            = true;
  TV_q->v_target.setZero();
  TV_q->v_prec=tv_q_v_prec;
  q_hand_home(TV_q->y_target);
  TV_q->y_prec=tv_q_y_prec;


  TV_skin->active = true;
  if(open_skin){
    TV_skin->y = skin_state;
    // trimm to target value to avoid oxilation
    for(uint i=0;i<TV_skin->y.N;i++)
      if(TV_skin->y(i)>tv_skin_trgt) TV_skin->y(i)=tv_skin_trgt;
  }else{
    TV_skin->y=ARR(tv_skin_fake,tv_skin_fake,tv_skin_fake);
  }
  // remove impact of arm joints. only fingers
  for(uint i=0;i<TV_skin->J.d0;i++) for(uint j=0;j<8;j++) TV_skin->J(i,j)=0.;
  transpose(TV_skin->Jt,TV_skin->J);
  SD_DBG1(TV_skin->J);
  TV_skin->y_prec = (((sum(TV_zeroLevel->y)) < .5 )? skin_prec : 0);
  SD_DBG1("sum of  zerolevel"<<sum(TV_zeroLevel->y));
  //TV_skin->y_prec = skin_prec;
  
  if (open_skin && sum(TV_skin->y)>tv_skin_trgt) grip = true;// start lift

  TV_lim->active            = true;
  TV_col->active            = true;

  /* ------- debug by activating TVs  ------------ */
  activateAll(TVs_all,false); //deactivate all variables

  TV_q->active            = true;
  TV_lim->active            = true;
  TV_col->active            = true;
  /*
  */

  TV_palm->active   = true;
  TV_oppose->active  = true;
  TV_fingAlign->active  = true;
  TV_zeroLevel->active  = true;
  TV_tipAlign->active  = true;
  /*
  */
  /* ------- end debug   ------------------------- */

  // output all active variables
#if 0
  TaskVariable *v;
  cout <<"*******************" <<endl;
  for_list(i,v,TVs_all) if(v->active){
    if(v->y_prec) cout <<"TV '" <<v->name<<"' y=" <<v->y <<" y_target=" <<v->y_target <<" y_prec=" <<v->y_prec <<" y_err=" <<v->err <<endl;
    if(v->v_prec) cout <<"TV '" <<v->name<<"' v=" <<v->v <<" v_target=" <<v->v_target <<" v_prec=" <<v->v_prec <<endl;
  }
#endif
  
  plot_append_data(ctrl);
}

/** collect data to plot. */
void
GraspISFTask::plot_append_data(ControllerProcess *ctrl){
  uint i;
  arr nabla_fx_t;
  double d;

  d = phiAtFrame(palm->X, nabla_fx_t, NULL);

  plot_data.append(d); // 0

  FORStringList(i, tipsN){
    d=phiAtFrame(tipsN(i)->X, nabla_fx_t,NULL);
    plot_data.append(d);//1, 8,15

    d=phiAtFrame(fingsN(i)->X, nabla_fx_t,NULL);
    plot_data.append(d);//2, 9,16

    plot_data.append(TV_fingAlign->y_target(i) - TV_fingAlign->y(i) );//3,10,17
    plot_data.append(TV_tipAlign->y_target(i) - TV_tipAlign->y(i));//4,11,18
    plot_data.append(TV_skin->y_target(i) - TV_skin->y(i));//5,12,19
    plot_data.append(TV_fingAlign->y_prec);//6,13,20
    plot_data.append(TV_tipAlign->y_prec);//7,14,21
  }

  plot_data.append(TV_skin->err);//22

}

/** after the loop plot the collected data */
void
GraspISFTask::plot_all(){
  uint graphs = 23; // adjust that accordin to plot_append_data
  uint samples = plot_data.N/graphs;

  //ctrl.sys->reportOnState(cout); 
  //plot
  plotGnuplot();

  plot_data.reshape( samples, graphs );
  plotFunctions(plot_data);
  plot(true);
  
  SD_INF("Please use plot.sh for a better visual experience!")
  SD_INF("Please adjust plot.sh according to the data collected in the loop!")
  
  plotOpengl();
}

