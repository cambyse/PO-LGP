struct sMotionPlanner{
  soc::SocSystem_Ors sys, *sys_parent;
  AICO planner;
}

void MotionPlanner::step(){
  if(!varPlan->get_hasGoal(this);){
    MT::sleep(0.01);
    MT_MSG("TODO");
    //broadcast mechanism instead of sleep?
    //stop your own loop and let somebody else wake you up?
    return;
  }

  //pull the current world configuration from the world
  varGeometricState->readAccess(this);
  s->sys.ors.copyShapesAndJoints(varGeometricState->sys);
  varGeometricState->deAccess(this);

  planner.init_messages();
  double d=s->planner.step();

  varPlan->writeAccess(this);
  varPlan->q_plan = planner.q;
  varPlan->tau  = sys->getTau();
  varPlan->converged=(d<planner.tolerance);// NIKOLAY : enssure reasonable plans
  varPlan->deAccess(this);
}

