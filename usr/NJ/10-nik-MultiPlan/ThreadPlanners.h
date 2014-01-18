//for debugging
ofstream file("costK.dat");
ofstream dfile("costD.dat");

#include <MT/specialTaskVariables.h>
#include <MT/threads.h>

#define NIKAICO

//should have a large time resolution, find empirically exact values
struct SecKinPlanner:public StepThread{
#ifdef NIKAICO
  soc::AICO aico;
#else
  soc::iLQG aico;
#endif

  soc::SocSystem_Ors *sys;
  TaskAbstraction *task;
  const soc::SocSystem_Ors *sys_parent;
  uint T;//tiem steps
  char * target;//which shape is goal
  uint nMod;//grasp or rreach
  double lastcost;
  bool showTrajectory;
  uint nTMessage;//which message to pass

  bool bInited;
  uint nSteps ;
  uint nInit;//type of initial path
  arr Clusters;
  double midPrec,endPrec,limPrec,colPrec, protoPrec;

  bool bReady;//what is proper way to do this with threads??

  //OUTPUT
  arr bwdMsg_v,bwdMsg_Vinv;
  //uint bwdMsg_count;

  //INPUT from dyn planner/hardware
  arr q0,v0;

  SecKinPlanner(char * name,uint _T):StepThread(name){T = _T;};

  void init(const soc::SocSystem_Ors& _sys,TaskAbstraction *_task, char * name, int nmod, uint _nInit = 0){
    sys_parent = &_sys;
    task=_task;
    bInited = false;
    nSteps = 0;
    nInit = _nInit;
    Clusters <<FILE("ClustQ4.txt");
    MT::getParameter(midPrec,"reachPlanMidPrec");
    MT::getParameter(endPrec,"reachPlanEndPrec");//load here to avoid tthread problems with CFG file

    MT::getParameter(limPrec,"reachPlanLimPrec");
    MT::getParameter(colPrec,"reachPlanColPrec");
    MT::getParameter(protoPrec,"reachPlanProtoPrec");

    //T = 100;//100
    nMod = nmod;
    target = name;

    lastcost = 10000;
    showTrajectory = true;

    nTMessage = 15;

  }

  void open(){
    sys=sys_parent->newClone(true);
    sys->setTimeInterval(T*0.01,T);
    sys->dynamic = true;
    sys->os = &cout;
#ifdef NIKAICO
    aico.init(*sys,.9,.0,.0,10,0);//0.7 con rate
#else
    aico.init(*sys,.9,10,0);
#endif
  }

  ors::Vector	EndEffPos(ors::KinematicWorld * C, const arr & q){
    C->setJointState(q);
    C->calcNodeFramesFromEdges();

    ors::Frame f=C->getBodyByName("m9")->X;
    f.addRelativeFrame(ors::Frame().setText("<t( .02   .022 -.366)>"));
    return f.p;
  }

  void step(){
    // if(nSteps++ > 3 && lastcost > 100) return;//avoid doing stuff when too bad
    bReady = false;
    sys->s->q0=q0;
    sys->s->v0=v0;//here or in everz case ??
    sys->setq(q0);//only in conjunction with set targets ??, otherwise s->q is enough, aico reads it
    //SetGoals();//in 10 bwd message or outside

    //if(true)
    if(!bInited){
      aico.v = NULL;
      aico.Vinv = NULL;
      aico.qhat = NULL;
      SetGoals();
      if(nInit > 0)
        soc::straightTaskTrajectory(*sys,aico.q,5);//hack to get qitself, more clever way ?
      else
        soc::straightTaskTrajectory(*sys,aico.q,0);
      aico.stepDynamic();//necessary, otherwise no v message inited from q
      file << "ni "  << q0 << endl;//nInit << " cost " << aico.cost << " q " << aico.q[100] << endl;
      bInited = true;
      if (nInit > 0){
        TaskVariable * V=listGetByName(sys->vars,"qitself");
        V->setInterpolatedTargetsConstPrecisions(T,0,0.);//trajec used onlzy for init
        V->active = false;
        V=listGetByName(sys->vars,"posNew");
        V->y_prec_trajectory *= 10000.0;
        //   V->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
        //      V=listGetByName(sys->vars,"collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T,colPrec,0.);
        //      V=listGetByName(sys->vars,"limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T,limPrec,0.);
      }
      //aico.stepDynamic();
    }

    // file << " plan " << nInit <<  " q " << aico.v[100] << endl;
   //if(nInit == 0)
     aico.stepDynamic();//in same step as init, or on next step    // aico.stepKinematic();
    double cost = aico.cost;//sys->totalCost(NULL,aico.q,true) ;
    //file << cost<< " q0 " << q0.sub(0,6) << " target  " <<  sys->ors->getBodyByName("target")->X.p <<  endl;
    file << " plan " << nInit <<  " cost " << cost << " q " << aico.qhat.sub(100,100,0,6) << endl;// " q " << aico.v[100] << endl;
    file.flush();
    lastcost = cost;
    bReady = true;
  }

  void close(){
  }

  void setKReachGoals(const char* objShape){
    sys->setTimeInterval(T*0.01,T);//T/0.01
    sys->setq0AsCurrent();
    // aico.initMessages(); !!!! thi hacks my intiializations
    activateAll(sys->vars,false);
    ors::Shape *obj = sys->ors->getShapeByName(objShape);
    TaskVariable *V;

    arr xtarget;
    xtarget.setCarray(obj->X.p.v,3);
    file << " target " << xtarget << endl;
    V=listGetByName(sys->vars,"posNew");				// V->updateState();
    V->y_target = xtarget;
    V->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
    V=listGetByName(sys->vars,"collision");
    V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T,colPrec,0.);
    V=listGetByName(sys->vars,"limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T,limPrec,0.);

    if(nInit > 0)   // if(true) !bInited
    {
      V=listGetByName(sys->vars,"qitself");
      arr currentClust0 = Clusters.sub((nInit-1)*400,(nInit-1)*400 + 399,0,6);
      arr currentClust(T+1,7);//1 more value
      currentClust[0] = q0;//currentClust0[0];
      for (uint i = 0; i < 400; i++)
      if(T == 400){
        currentClust[i+1]= currentClust0[i];
      }
      else if(T == 800){
        currentClust[i*2+1]= currentClust0[i];
        currentClust[i*2+2]= currentClust0[i];
      }
      else if(T == 200){
        currentClust[i/2+1]= currentClust0[i];
      }
      V->setInterpolatedTargetsConstPrecisions(T,protoPrec,0.);//tricky method,  destroys my targets !!
      V->y_trajectory = currentClust;
      for(uint i = 0; i<= T; i++)
        if(i < 0.125*T){
          V->y_prec_trajectory(i) = protoPrec*i/(0.125*T);
          V->y_trajectory[i] = (V->y_trajectory[i]*(i+0.0) + (0.125*T-i)*q0)/(0.125*T);
        }
        else if (i < 0.875*T)
          V->y_prec_trajectory(i) = protoPrec;
        else{
          V->y_prec_trajectory(i) = protoPrec*(i-0.875*T)/(0.125*T);
       //   V->y_trajectory[i] = (V->y_trajectory[i]*(400.0-i) + (i-350.0)*q0)/50.0;
        }
      V=listGetByName(sys->vars,"posNew");
      V->y_prec_trajectory *= 0.0001;
    }
  }

  void setKGraspGoals(const char* objShape){
    sys->setTimeInterval(4.,T);
    sys->setq0AsCurrent();
    activateAll(sys->vars,false);
    setGraspGoals(*sys,T,objShape);
    target = (char*)objShape;
  }

  void SetGoals(){
#ifdef NIKAICO
    aico.initMessages();
    aico.convergenceRate = 0.9;
#endif
    if (nMod == 1)
      setKReachGoals(target);

    if (nMod == 2)
      setKGraspGoals(target);
  }

  void UpdateExtState(ors::Body * b){
    int ind = b->index;
    sys->ors->bodies(ind)->X.r = b->X.r;//thus give orientation as well, pos is given to external state
    sys->ors->bodies(ind)->X.p = b->X.p;//actually, X.p is read in goals without ext state, so change pos as well
    arr state(T+1,4);//50 is number of time steps in planner, later make smarter
    for (uint i = 0; i < state.d0; i++){
      state(i,0)=ind;
      for(uint j = 0; j <3;j++)
        state(i,j+1)= b->X.p(j) + b->X.v(j)*4.0*i/(double)(T+1);//estimated position, this planner stands for 4s in t steps
    }
    sys->s->q_external = state;
    //SetGoals();//here or in other loop...
  }
};

struct ThreadPlanner:public StepThread{
  //  TaskAbstraction *task;
  uint T;//time steps

  uint nBest;//best sub planner

  MT::Array<SecKinPlanner*> helpers;
  double lastCost,diffCost;//judge planners progress and whether costs worsening
 // bool bStop;

  //OUTPUT
  arr bwdMsg_v,bwdMsg_Vinv;
  uint bwdMsg_count;

  //INPUT from simulator
  arr q0,v0;



  ThreadPlanner():StepThread("dynamicplanner"){};

  //current position and velocity in m/s of body
  void UpdateExtState(ors::Body * b){
    for(uint i = 0; i < helpers.N; i++)
      helpers(i)-> UpdateExtState(b);
  }


  void UnsetInit(){
    for(uint i = 0; i < helpers.N; i++){
      helpers(i)->bInited = false;
      helpers(i)->nSteps = 0;
    }
    // bStop = true;
    bwdMsg_count = 0;
    bwdMsg_v = arr(0);
    lastCost = 1000000;
    diffCost = 0;
    nBest =666;

  //  bStop = false;
  }

  void init(const soc::SocSystem_Ors& _sys,TaskAbstraction *_task, char * name, int nMod, uint _T){
    T = _T;
    helpers.append(new SecKinPlanner("kp0",T));
    helpers.append(new SecKinPlanner("kp1",T));
    helpers.append(new SecKinPlanner("kp2",T));
    helpers.append(new SecKinPlanner("kp3",T));
    helpers.append(new SecKinPlanner("kp4",T));
    for(uint i = 0; i < helpers.N; i++)
       helpers(i)->init(_sys,_task,name,nMod,i);

    lastCost = 1000000;
    diffCost = 0;
    nBest =666;
    bwdMsg_count=0;
   // bStop = false;
  }

  void open(){
    for(uint i = 0; i < helpers.N; i++)
      helpers(i)->threadOpen();
  }

  void step(){
    //if(q0.N == 0)
    ///  return;//sometimes other threads slower in beginning due to vision

    if(lastCost > 0.2)
    for (uint i = 0; i < helpers.N; i++)
      helpers(i)->threadStep();
    else //already moving, just bset updates
      helpers(nBest)->threadStep();

    for (uint i = 0; i < helpers.N; i++)
      helpers(i)->threadWait();

  /*  bool bReady;//wait to finish all planners
    while(bReady == false){
      bReady = true;
      for (uint i = 0; i < helpers.N; i++)
        if(helpers(i)->bReady == false ){//|| helpers(i)->bInited == false
          bReady = false;
          break;
        }
      MT::wait(0.0003);
     // if(bStop){
     //   bStop = false;
     //   return;
     // }
    }*/

    for (uint i = 0; i < helpers.N; i++)
      if(helpers(i)->bInited == false )
        return;//without giving state in this case

    //  if(nBest != 666)
    //    return;//simplify, only best at beginning...
    //check all planners and get index of best
    //nBest =666;
    double costBest = 10000000000;
    for (uint i = 0; i < helpers.N; i++)
      if(helpers(i)->aico.cost < costBest){
        costBest = helpers(i)->aico.cost;
        nBest = i;
      }

    dfile << lastCost << endl;

    diffCost =  helpers(nBest)->aico.cost-lastCost;
    if (diffCost > 0){//no new best
      cout << " no new best " << endl;
      for (uint i = 0; i < helpers.N; i++)
        helpers(i)->aico.convergenceRate *= 0.9;
      return;
    }
#ifdef NIKAICO
    bwdMsg_v = helpers(nBest)->aico.qhat;//v or qhat ??
#else
    bwdMsg_v = helpers(nBest)->aico.v;
#endif
    bwdMsg_Vinv = helpers(nBest)->aico.Vinv;


    lastCost = helpers(nBest)->aico.cost;



    // cout << " ai q" << helpers(nBest)->aico.q.sub(bwdMsg_count,bwdMsg_count,0,3) << endl;
    //   if(bwdMsg_count == 0)
    //    cout << helpers(nBest)->aico.q << endl;
    cout << endl << endl << " BESTTT " << nBest << " cost: " << lastCost << endl;
  }

  void close(){
    for(uint i = 0; i < helpers.N; i++)
      delete helpers(i);
  }

  void SetQV0(const arr & q, const arr & v){
    q0 = q;
    v0 = v;
    for(uint i = 0; i < helpers.N; i++){
      helpers(i)->q0 =q;
      helpers(i)->v0 = v;
      //   helpers(i)->aico.initMessages();
    }
  }
};

