#include "graspISF.h"
#include<MT/plot.h>

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
}

double
GraspISFTask::phiAtFrame(ors::Transformation& X, arr &grad){
  arr x;
  x.setCarray(X.pos.p,3);
  double phi = graspobj->psi(NULL,x);
  graspobj->getNormGrad(grad,x);
  return phi;
}

void
GraspISFTask::initTaskVariables(ControllerModule *ctrl){
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
  TV_tipAlign->setGains(.01,.0);
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
  TVs_all.append(TV_zeroLevel);

  TV_skin = new TaskVariable("skin", ctrl->ors, skinTVT,0,0,0,0,skins);
  //TV_skin->targetType=gainsTT;
  TV_skin->targetType=directTT;
  TV_skin->v_prec=0;
  TV_skin->y_target=ARR(.02,.02,.02);
  TVs_all.append(TV_skin);

  
  TVs_all.append(TV_q);
  TVs_all.append(TV_col);
  TVs_all.append(TV_lim);
  ctrl->sys.setTaskVariables(TVs_all);

}

void
GraspISFTask::updateTaskVariables(ControllerModule *ctrl){
  //activateAll(TVall,false); //THIS IS THE WRONG LIST!!! deactivate all variables
  activateAll(TVs_all,false); //deactivate all variables
  ctrl->useBwdMsg=false;             //deactivate use of bwd messages (from planning)

  if (grip) {
    TV_palm->active   = true;
    TV_palm->y_target(2) = 1.1;
    TV_palm->y_prec   = 5e1;

    TV_lim->active            = true;
    TV_col->active            = true;
    TV_q->active            = true;

    return;
  }
  cout <<"*******************" <<endl;

  arr nabla_fx_t, x_t;
  double phi;
  uint i;
  
  phi = phiAtFrame(palm->X, nabla_fx_t);

  // palm position
  TV_palm->active   = true;
  TV_palm->y_target = TV_palm->y - nabla_fx_t * .2;
  TV_palm->y_prec   = 1e3 * (phi>0.?phi:0.); //care only away from surface

  // oppose fingers
  TV_oppose->active  = true;
  TV_oppose->y_target = 0;
  TV_oppose->y_prec = 1e1; // * (phi>0.?phi:0.); //care only away from surface;

  // position to surface (phi=0)
  TV_zeroLevel->active  = true;
  TV_zeroLevel->y_target = -phi;
  TV_zeroLevel->y_prec = 2e1  * (phi>0?(1.-phi):0); //care only at surface

  // TODO: phi for ifnger is not phi for palm!!!!!!
  TV_fingAlign->active  = true;
  TV_fingAlign->y_target = -1.;
  //TV_fingAlign->y_prec = 1e3 * (phi>0.?phi:0.); //care only if palm is away from surface;
  TV_fingAlign->y_prec = SD_MAX(1e3 * sin(phi*3.), 0); //care in the middle
  //TV_fingAlign->y_prec = SD_MAX(1e3 * sin(phi*MT_PI), 0); //care in the middle
  //TV_fingAlign->y_prec = SD_MAX(1e3 * sin(phi*2.5), 0); //care in the middle

  // align tipsN with gradient
  TV_tipAlign->active  = true;
  TV_tipAlign->y_target = -1.;
  TV_tipAlign->y_prec = 1e3 * (1.-phi); //care only at surface


  // direct q control (hony pot & comfortable)
  TV_q->active            = true;
  TV_q->v_target.setZero();
  TV_q->v_prec=2e-1;
  q_hand_home(TV_q->y_target);
  TV_q->y_prec=1e-1;


  TV_skin->active = true;
  if(open_skin){
    TV_skin->y = skin_state;
    // trimm to target value to avoid oxilation
    for(uint i=0;i<TV_skin->y.N;i++)
      if(TV_skin->y(i)>.02) TV_skin->y(i)=.02;
  }else{
    TV_skin->y=ARR(.01,.01,.01);
  }
  // remove impact of arm joints. only fingers
  for(uint i=0;i<TV_skin->J.d0;i++) for(uint j=0;j<8;j++) TV_skin->J(i,j)=0.;
  transpose(TV_skin->Jt,TV_skin->J);
  SD_DBG(TV_skin->J);
  TV_skin->y_prec = (((sum(TV_zeroLevel->y)) < .5 )? skin_prec : 0);
  SD_DBG("sum of  zerolevel"<<sum(TV_zeroLevel->y));
  //TV_skin->y_prec = skin_prec;
  
  if (open_skin && sum(TV_skin->y)>.02) grip = true;// start lift

  TV_lim->active            = true;
  TV_col->active            = true;


  // output all active variables
  TaskVariable *v;
  cout <<"*******************" <<endl;
  for_list(i,v,TVs_all) if(v->active){
    if(v->y_prec) cout <<"TV '" <<v->name<<"' y=" <<v->y <<" y_target=" <<v->y_target <<" y_prec=" <<v->y_prec <<" y_err=" <<v->err <<endl;
    if(v->v_prec) cout <<"TV '" <<v->name<<"' v=" <<v->v <<" v_target=" <<v->v_target <<" v_prec=" <<v->v_prec <<endl;
  }
  
  plot_append_data(ctrl);
}

/** collect data to plot. */
void
GraspISFTask::plot_append_data(ControllerModule *ctrl){
  uint i;
  arr nabla_fx_t;
  double d;

  d = phiAtFrame(palm->X, nabla_fx_t);

  plot_data.append(d); // 0

  FORStringList(i, tipsN){
    d=phiAtFrame(tipsN(i)->X, nabla_fx_t);
    plot_data.append(d);//1, 8,15

    d=phiAtFrame(fingsN(i)->X, nabla_fx_t);
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

