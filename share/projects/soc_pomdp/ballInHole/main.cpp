#include <Ors/roboticsCourse.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/feedbackControl.h>
#include <Optim/optimization.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

extern double stickyWeight;

//GET R_t, r_t using linearization
void getQuadraticTaskCost(arr& R, arr& r, Simulator& S, const arr&qt, const int n, bool vec){
  arr y_target,pos_target,y,J;
  double prec=1e-1;;

  R.resize(n,n); R.setZero();
  r.resize(n) ;  r.setZero();

  S.kinematicsPos(pos_target,"hole");

  //cout<<"HOLE = "<<pos_target<<endl;

  //1st task: peg positioned within hole
  S.kinematicsPos(y,"peg");
  S.jacobianPos  (J,"peg");
  y_target = pos_target;
  //if(vec) y_target = y_target + ARR(1.0,0.,0.95); // plus the length of the peg
  if(vec) y_target = y_target + ARR(0.0,0.,0.6); // plus the length of the peg

  R += prec*(~J)*J;
  r -= (double)2.*prec*(~J)*(y_target - y + J*qt);

  //2nd task: rotation
  if(vec){
      S.kinematicsVec(y, "peg");
      S.jacobianVec(J, "peg");
      y_target = ARR(0,0,-1.);
      R += prec*(~J)*J;
      r -= (double)2.*prec*(~J)*(y_target - y + J*qt);

  }

  //3rd task: collisions
  S.kinematicsContacts(y);
  S.jacobianContacts(J);
  y_target = ARR(0.); //target is zero collision costs

  R += prec*(~J)*J;
  if(vec){
      r -= (double)2.*prec*(~J)*(y_target - y + J*qt);
  }else
      r -= (double)2.*prec*J[0]*(y_target - y + (~J[0])*qt); //ball in a hole (HACKING)
}
//////////////////////////////////////////////////////////////////////////////////////////
/// \brief peg_in_a_hole
/// iLQG
///
void peg_in_a_hole(){

    Simulator S("pegInAHole.ors");
    S.setContactMargin(.10); //this is 1 cm (all units are in meter)


    uint T = 100;
    //arr q0;
    arr q0=ARR(-0.624044, 0.0917871, 1.15677, 0.302998, -1.21139, -0.822536,-0.0317157);
    //S.setJointAngles(q0);

    // c)

    cout <<"iLQG:" <<endl;

    uint t,n = q0.d0;
    arr v,V,r,R;      //!< fwd, bwd, and task messages
    arr q(T+1,n),HVinv(T+1,n,n),VHVinv;
      //we assume A=B=\id for now
    q.setZero();

    q[0]() = q0;


      //trajectory needs to be initialized!
     // CHECK(q.nd==2 && q.d0==T+1 && q.d1==n,"please initialize trajectory!");

    for(uint iter=0;iter<1000;iter++){
        arr q_old(q);

        V.resize(T+1,n,n);  v.resize(T+1,n);
        arr H = eye(n,n); //identity matrix for control cost
        R.resize(T+1,n,n);  r.resize(T+1,n);

          //linearize around current trajectory
        for(t=0;t<=T;t++){
            S.setJointAngles(q[t],false);
            getQuadraticTaskCost(R[t](),r[t](),S,q[t](),n,true);
        }


        //bwd Ricatti equations
        V[T]() = R[T];
        v[T]() = r[T];
        for(t=T;t--;){
            inverse_SymPosDef(HVinv[t+1](), H+V[t+1]);
            VHVinv = V[t+1]*HVinv[t+1];
            V[t]() = R[t] + V[t+1] - VHVinv*V[t+1];
            v[t]() = r[t] + v[t+1] - VHVinv*v[t+1];
        }

        //fwd with optimal control
        //LQG without collision check: FAIL
        double convergenceRate = 0.8;
        q[0]() = q0;
        for(t=1;t<=T;t++){
            q[t]() = ((double)1.-convergenceRate)*q[t] + convergenceRate*(q[t-1] - HVinv[t]*((double).5*v[t] + V[t]*q[t-1]));
        }

          //check kinematic step
        double diff = 0;
        if(q_old.N==q.N) diff=maxDiff(q_old,q);

        cout <<"iLQG:[iter] = "<<iter<<"  diff = "<<diff <<endl;
        if(diff < 1e-2) break;
    }

    // replay the trajectory
    for(t=0;t<=T;t++){
        cout<<q[t]()<<endl;
        S.setJointAngles(q[t]());
    }

    S.watch(true);

}



void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, const double& height){

  //set table height
  world.getBodyByName("hole")->X.pos.z = height;

  MotionProblem P(world, false);
  P.loadTransitionParameters();
  x = P.getInitialization();
  P.makeContactsAttractive=true;
  stickyWeight=1.;

  //-- setup the motion problem
  TaskCost *pos = P.addTask("position",
                            new DefaultTaskMap(posTMT, world, "peg", NoVector, "target", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  TaskCost *vel = P.addTask("position_vel", new DefaultTaskMap(posTMT, world, "peg", NoVector));
  vel->map.order=1;
  P.setInterpolatingCosts(vel, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  TaskCost *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "peg", ARR(0,0,-1,.7)));
  P.setInterpolatingCosts(cons, MotionProblem::constant, ARRAY(0.), 1.);

  //-- convert
  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  //-- optimize
  MT::timerStart();
  optConstrained(x, dual, Convert(MF));
  cout <<"** optimization time = " <<MT::timerRead() <<endl;
  P.dualMatrix = dual;
  P.costReport(false);

  if(&y){
    y.resize(x.d0, pos->map.dim_phi(world));
    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      pos->map.phi(y[t](), NoArr, world);
    }
  }
  if(&dual) dual.reshape(dual.N);
}



////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief POMDPExecution
/// \param allx
/// \param ally
/// \param alldual
/// \param world
/// \param num
///
void POMDPExecution(const arr& allx, const arr& ally, const arr& alldual, ors::KinematicWorld& world, int num){
  arr q, qdot;
  world.getJointState(q, qdot);

  ofstream data(STRING("data-"<<num<<".dat"));

  ors::Shape *endeff = world.getShapeByName("endeff");
  ors::Shape *true_target = world.getShapeByName("truetarget");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("table");
  double mean_table_height = table->X.pos.z;

  double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);

  FeedbackMotionControl MC(world,false);
  //MC.nullSpacePD.active=false;
  MC.qitselfPD.active=true;

  //position PD task
  PDtask *pd_y=
      MC.addPDTask("position", .1, .8,
                   new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x=
      MC.addPDTask("pose", .1, .8,
                    new TaskMap_qItself());
  pd_x->prec = .1;

  //plane constraint task
#define USE_DUAL
#ifdef USE_DUAL
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeff", ARR(0,0,-1,table->X.pos.z+0.02));
  ConstraintForceTask *pd_c =
      MC.addConstraintForceTask("planeConstraint", plane_constraint );
//      MC.addConstraintForceTask("touchTable",
//                                new PairCollisionConstraint(world, "endeff2", "table"));
#endif


  double tau = 0.01;
  arr x = allx[0];
  arr y = ally[0];
  arr dual = alldual[0];

  cout<< allx.nd << "  "<<ally.nd << "  "<<alldual.nd<<endl;

  MT::Array<bool> particles;
  particles.resize(allx.d0);
  uint eligible_counts = allx.d0;
  uint index = 0, prev=index;
  for(int i=0;i<particles.d0;i++)
      particles(i) = true;

  //loop over time

  // remaining 100 steps is for reaching to the target.
  double observation = -10000.0;
  for(uint t=0;t<x.d0+100;t++){
    MC.setState(q, qdot);


    // POMDP's online action selection
    #ifdef USE_DUAL
    observation = pd_c->infraRed_obs;
    //cout<<"observation "<<observation<<endl;
    prev = index;
   if(t<y.d0){
        for(uint sample = 0; sample < alldual.d0 ; sample++){
            //observation: equivalent to touch or not?

            if(particles(sample)){
                //un-touch: but lambda > 0 (desired touch), then eliminate this particle (sample)
                if((observation < -1e-2) && (alldual[sample](t) > 0)){
                    particles(sample) = false;
                    eligible_counts = eligible_counts - 1;
                }
                else if((observation > -1e-2) && (alldual[sample](t) == 0)){
                    particles(sample) = false;
                    eligible_counts = eligible_counts - 1;
                }else
                    index = sample;
            }
        }
   }
    #endif


    //adapt the PD task references following the plan

    if(prev!=index) cout<<" at "<<t<<"; using model # "<<index << " "<<endl;
    //cout<<" at "<<t<<"; size = "<<eligible_counts<<endl;
    if(t<y.d0){
      pd_y->y_ref = ally[index][t];
      pd_x->y_ref = allx[index][t];
#ifdef USE_DUAL
      pd_c->desiredForce = alldual[index](t);
#endif
    }


#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;

    if(pd_c->desiredApproach.y.N){ //this tell that the plane was here before.
      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
        est_target->X.pos.z = endeff->X.pos.z+0.1; //est_target position update
      }
    }

#endif
    //external sinus on the table height
    table->X.pos.z = mean_table_height+sin_jitter*::sin(double(t)/15);
#ifdef USE_DUAL
    plane_constraint->planeParams(3) = table->X.pos.z+0.02;
#endif

    //operational space loop
    for(uint tt=0;tt<10;tt++){
      MC.updateConstraintControllers();
      arr a = MC.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
    }

    //display and record video
//    world.watch(false, STRING(t));
    world.gl().update(STRING(t), true, false, true);
    //    flip_image(world.gl().captureImage);
    //    vid->addFrame(world.gl().captureImage);

    //write data
    MT::arrayBrackets="  ";
    data <<t <<' ' <<(t<dual.N?dual(t):0.) <<' '
        <<table->X.pos.z <<' '
       <<endeff->X.pos.z <<' '
      <<endeff->X.pos.z-table->X.pos.z <<' '
      <<est_target->X.pos.z <<' '
     <<true_target->X.pos.z <<' '
    <<endl;
  }
  data.close();

  FILE(STRING("data-"<<num<<"-err.dat")) << ARRAY(true_target->X.pos)- ARRAY(endeff->X.pos);
}

/////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
///
int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  //peg_in_a_hole();
  //ball_in_a_hole(); 

  //ors::KinematicWorld world("pegInAHole.ors");
  ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));

  MT::timerStart(true);

  uint T = 200;

  //compute the primal and dual trajectories
  arr heights;

  uint numSamples = 10;

  heights.resize(numSamples);
  arr allX, allY, allDual;
  arr values; //2-dim: sample, time
  values.resize(numSamples,T+1);

  //arr x, y, dual;
  //getTrajectory(x, y, dual, world);

  //table height
  heights(0) = .6 + 0.1*rnd.gauss();

  arr x, y, dual;
  getTrajectory(x, y, dual, world, heights(0));

  allX.resize(numSamples,x.d0,x.d1);
  allY.resize(numSamples,y.d0,y.d1);
  allDual.resize(numSamples,dual.d0);

  allX[0]() = x;
  allY[0]() = y;
  allDual[0]() = dual;

  for(uint i=1;i<numSamples;i++){
       heights(i) = .6 + 0.1*rnd.gauss();
       arr x, y, dual;
       getTrajectory(x, y, dual, world, heights(i));

       //3. store the trajectories
       allX[i]() = x;
       allY[i]() = y;
       allDual[i]() = dual;
   }

  cout<<"Offline Computation Time = "<< MT::realTime() <<" (s)"<<endl;

  cout<<allDual[0]<<endl;

/*/
  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;
  //world.setJointState(x[0]);
  world.setJointState(allX[0][0]);


  for(uint i=0;i<10;i++){
    world.getBodyByName("table")->X.pos.z = .6 + .1*rnd.gauss();
    //testExecution(x, y, dual, world, i);
    POMDPExecution(allX, allY, allDual, world, i);
  }
  /*/


  return 0;

}
