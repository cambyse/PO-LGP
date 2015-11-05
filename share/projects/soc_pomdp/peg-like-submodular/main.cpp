#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/feedbackControl.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

//extern double stickyWeight;

//VideoEncoder_libav_simple *vid;

void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, const arr target, uint horizon){
    /////////////


  //set the sampled model's paramter

  world.getBodyByName("target")->X.pos.z = target(2);
  world.getBodyByName("target")->X.pos.y = target(1);
  world.getBodyByName("target")->X.pos.x = target(0);


  cout<<"target: "<< target <<endl;
  ors::Shape *endeff = world.getShapeByName("endeff");
  //check
  cout<< endeff->X.pos.x<<" "<< endeff->X.pos.y<<" "<< endeff->X.pos.z<<" "<<endl;



  MotionProblem P(world, false);
  //P.loadTransitionParameters(); // can change horizon here
  P.T = horizon;

  x = P.getInitialization();


  //-- setup the motion problem
  Task *t;


  t = P.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, P.T, {0.}, 1e0);

  t = P.addTask("final_vel", new TransitionTaskMap(world));
  t->map.order=1; //make this an acceleration task!
  t->setCostSpecs(P.T-4, P.T, {0.}, 1e2);


  Task *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target", NoVector));
  pos->setCostSpecs(P.T, P.T, {0.}, 1e3);
 // pos->setCostSpecs(P.T, P.T,{0.,0.,0.}, 1e3);


    // ARR(0,0,-1,.7): ax + by + cz + d: where n=(0,0,-1) is its normal vector; d = 0.7
  Task *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1, target(2))));
  cons->setCostSpecs(0, P.T, {0.}, 1.);



  //coll->setCostSpecs(0, P.T, {0.}, 1.);



  t = P.addTask("collisionConstraints", new CollisionConstraint());
  t->setCostSpecs(0, P.T, {0.}, 1.);

  //Task *sticky = P.addTask("collisionStickiness", new ConstraintStickiness(t->map));
  //sticky->setCostSpecs(0, P.T, {0.}, 1.e1);
  //t->setCostSpecs(0, P.T, {0.}, 1.);
  //Task *sticky = P.addTask("collisionStickiness", new ConstraintStickiness(cons->map));
  //sticky->setCostSpecs(0, P.T, {0.}, 1.e1);
  //sticky->active = true;

  //Task *sticky = P.addTask("planeStickiness", new ConstraintStickiness(cons->map));
  //sticky->setCostSpecs(0, P.T, {0.}, 1.);






  //stickyWeight=1.;
  //P.makeContactsAttractive = true;

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);


  /*/
  for(uint k=0;k<5;k++){

    //optConstrainedMix(x, P.dualMatrix, ConstrainedP);

    optConstrainedMix(x,  NoArr, ConstrainedP, OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-3));
    //cout<<P.dualMatrix <<endl;

    P.costReport(false);

  }
  //get the final optimal cost at each time slice
  P.costReport(false);
/*/


  optConstrainedMix(x,  P.dualMatrix, ConstrainedP, OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-3));

  if(&y){
    y.resize(x.d0, pos->map.dim_phi(world));
    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      pos->map.phi(y[t](), NoArr, world);
    }
  }


  if(&dual) dual = P.dualMatrix;
}

  void switchToTransparent(void*){
      orsDrawAlpha = .3;
  }

  void switchToNormal(void*){
      orsDrawAlpha = 1.0;
  }


/// Using Online Planning as Submodularity
void OnlineSubmodularity(const double tableW, const double tableL, ors::KinematicWorld& world, int num, const arr target, int type, arr &pos){
    ors::Shape *endeff = world.getShapeByName("endeff");
    ors::Shape *true_target = world.getShapeByName("truetarget");
    ors::Body *est_target = world.getBodyByName("target");
    ors::Body *table = world.getBodyByName("table");

  arr q, qdot;
  world.getJointState(q, qdot);


  //optimize the trajectory w.r.t a selected particle (model)
  arr x, y, dual, temp;
  getTrajectory(x, y, temp, world, target, 200); //one example of best model
  dual.resize(x.d0);

  int tt=0;
  for(int i=0;i<temp.d0;i++){
      if((i+1)%9==0){
          dual(tt) = temp(i);
          tt++;
      }
      if(tt>=x.d0) break;

  }

  ofstream data(STRING("data-"<<num<<".dat"));


  FeedbackMotionControl MC(world);
  MC.qitselfPD.active=false;

  //position PD task:  decayTime = 0.1, dampingRatio = 0.8
  PDtask *pd_y =  MC.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x = MC.addPDTask("pose", .1, .8, new TaskMap_qItself());
  pd_x->prec = .1;

  //plane constraint task

#define USE_DUAL
ConstraintForceTask *pd_c ;

#ifdef USE_DUAL
  if(type>0){
      PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeff", ARR(0,0,-1,table->X.pos.z+0.02));

      pd_c = MC.addConstraintForceTask("planeConstraint", plane_constraint );
  }

#endif

  double tau = 0.01;


  for(uint t=0;t<x.d0 + 50;t++){
      world.gl().add(switchToNormal);


    MC.setState(q, qdot);

      if(t<y.d0){
        pd_y->y_ref = y[t];
        pd_x->y_ref = x[t];
  #ifdef USE_DUAL
        if(type>0)
            pd_c->desiredForce = dual(t);
        //else
        //    pd_c->desiredForce = 0.; //dont need stickyness
  #endif
    }

    if(type==0){
        if ((t>100)&&(endeff->X.pos.z <= table->X.pos.z + 0.02)){
            est_target->X.pos.z = endeff->X.pos.z + 0.12; // estimate table's height

            cout<<"height = "<< endeff->X.pos.z <<endl;

            pos.resize(3);
            pos(0) = endeff->X.pos.x;
            pos(1) = endeff->X.pos.y;
            pos(2) = endeff->X.pos.z;

            break; //terminate the macro action when receiving sensing information

        }
    }
    if(type==1){ //left edge: termination
        if ((t>100)&&(endeff->X.pos.x <= table->X.pos.x - tableL/2.)){

            cout<<"edge in X left = "<< endeff->X.pos.x <<endl;

            pos.resize(3);
            pos(0) = endeff->X.pos.x;
            pos(1) = endeff->X.pos.y;
            pos(2) = endeff->X.pos.z;

            break; //terminate the macro action when receiving sensing information

        }
    }

    if(type==2){ //right edge: termination
        if ((t>100)&&(endeff->X.pos.x >= table->X.pos.x + tableL/2.)){

            cout<<"edge in X right = "<< endeff->X.pos.x <<endl;

            pos.resize(3);
            pos(0) = endeff->X.pos.x;
            pos(1) = endeff->X.pos.y;
            pos(2) = endeff->X.pos.z;

            break; //terminate the macro action when receiving sensing information

        }
    }

    if(type==3){ //right edge: termination
        if ((t>100)&&(endeff->X.pos.y >= table->X.pos.y + tableW/2.)){

            cout<<"edge in Y = "<< endeff->X.pos.y <<endl;

            pos.resize(3);
            pos(0) = endeff->X.pos.x;
            pos(1) = endeff->X.pos.y;
            pos(2) = endeff->X.pos.z;

            break; //terminate the macro action when receiving sensing information

        }
    }

    if(type==4){ //left edge: termination
        if ((t>100)&&(endeff->X.pos.y <= table->X.pos.y - tableW/2.)){

            cout<<"edge in Y = "<< endeff->X.pos.y <<endl;

            pos.resize(3);
            pos(0) = endeff->X.pos.x;
            pos(1) = endeff->X.pos.y;
            pos(2) = endeff->X.pos.z;

            break; //terminate the macro action when receiving sensing information

        }
    }

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
    mlr::arrayBrackets="  ";
    data <<t <<' ' <<(t<dual.N?dual(t):0.) <<' '
        <<table->X.pos.z <<' '
       <<endeff->X.pos.z <<' '
      <<endeff->X.pos.z-table->X.pos.z <<' '
      <<est_target->X.pos.z <<' '
     <<true_target->X.pos.z <<' '
    <<endl;
  }
  data.close();

  FILE(STRING("data-"<<num<<"-err.dat")) << conv_vec2arr(true_target->X.pos)- conv_vec2arr(endeff->X.pos);
}




////////////////////////////////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
///////////////////////////////////////////////////////////////////////////////
int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  ors::KinematicWorld world(mlr::getParameter<mlr::String>("orsFile"));
  uint T = 200; //time horizon
  ors::Body *table = world.getBodyByName("table");
  double tableW = table->shapes(0)->size[1];
  double tableL = table->shapes(0)->size[0];
  double tableT = table->shapes(0)->size[2];
  double x,y,z;
  x = table->X.pos.x;
  y = table->X.pos.y;
  z = table->X.pos.z;


  mlr::timerStart(true);


  //OpenGL gl;
  ors::KinematicWorld hypotheses;
  for(int i=0;i<100;i++){
    ors::Shape *s = new ors::Shape(hypotheses,NoBody);
    s->type = ors::boxST;
    s->X.pos.x = x + 0.1*rnd.gauss();
    s->X.pos.y = y + 0.1*rnd.gauss();
    s->X.pos.z = z + 0.1*rnd.gauss();
    s->size[0] = tableL + 0.1*rnd.gauss();
    s->size[1] = tableW + 0.1*rnd.gauss();
    s->size[2] = tableT + 0.1*rnd.gauss();
    s->color[0] = 0.0 + i*0.001 ;
    s->color[1] = 0.5  ;
    s->color[2] = 0.5  ;
  }
  //gl.add(ors::glDrawGraph, &world);
  world.gl().add(switchToTransparent);
  world.gl().add(ors::glDrawGraph, &hypotheses);

  //hypotheses.shapes.remove(0);
  //world.gl().watch();
 // return 0;

  arr x0;
  world.getJointState(x0);



  for(uint i=0;i<10;i++){
      cout<<" [TRIAL] "<<i<<"     ################################################# "<<endl;
      double height;
      double length;
      double width;

      //four edges
      arr left_edge(3);
      arr right_edge(3);
      arr front_edge(3);
      arr rear_edge(3);

      // DETECT_EDGE
      world.setJointState(x0);


     // DETECT_HEIGHT
      ors::Shape *endeff = world.getShapeByName("endeff");
      arr target;
      target.resize(3);
      target(0) = endeff->X.pos.x;
      target(1) = endeff->X.pos.y;
      target(2) = 0.60; //lowest height of the sampled table //assuming having biggest IGvietna

      arr terminalPos, temp;


      OnlineSubmodularity(tableW, tableL, world, i, target, 0, terminalPos);

      arr start_detect_edge;
      world.getJointState(start_detect_edge);
      height = endeff->X.pos.z;
      cout<<" height" <<height <<endl;


      //store: initial pose and end-eff after detecting edge
      //start_detect_edge, terminalPos
      //cout<< terminalPos <<endl;
      //cout<<endeff->X.pos.x <<" "<<endeff->X.pos.y <<" "<<endeff->X.pos.z <<" " <<endl;



      //DETECT EDGES: Currently hard-coded. and known orientation?
      // TODO: Automate the action selection using SUBMODULARITY


      world.setJointState(start_detect_edge);

      target(0) = terminalPos(0) - 1.0; //very far to the left
      target(1) = terminalPos(1);
      target(2) = terminalPos(2);

      OnlineSubmodularity(tableW, tableL, world, i, target, 1, temp);

      left_edge(0) = endeff->X.pos.x;
      left_edge(1) = endeff->X.pos.y;
      left_edge(2) = endeff->X.pos.z;


      cout<<" left_edge" <<left_edge <<endl;
      //cout<< terminalPos <<endl;
      //cout<<endeff->X.pos.x <<" "<<endeff->X.pos.y <<" "<<endeff->X.pos.z <<" " <<endl;


      world.setJointState(start_detect_edge);

      target(0) = terminalPos(0) + 0.4; //very far to the right
      target(1) = terminalPos(1);
      target(2) = terminalPos(2);

      OnlineSubmodularity(tableW, tableL, world, i, target, 2, temp);

      right_edge(0) = endeff->X.pos.x;
      right_edge(1) = endeff->X.pos.y;
      right_edge(2) = endeff->X.pos.z;

      cout<<" right_edge" <<right_edge <<endl;


     length = right_edge(0) - left_edge(0);

     cout<< "FOUND LENGHT = "<<length<<endl;


      world.setJointState(start_detect_edge);

      target(0) = terminalPos(0); //very far to the right
      target(1) = terminalPos(1) + 0.35;
      target(2) = terminalPos(2);

      OnlineSubmodularity(tableW, tableL, world, i, target, 3, temp);

      rear_edge(0) = endeff->X.pos.x;
      rear_edge(1) = endeff->X.pos.y;
      rear_edge(2) = endeff->X.pos.z;

      cout<<" rear_edge" <<rear_edge <<endl;

      world.setJointState(start_detect_edge);

      target(0) = terminalPos(0); //very far to the right
      target(1) = terminalPos(1) - 0.4;
      target(2) = terminalPos(2);

      OnlineSubmodularity(tableW, tableL, world, i, target, 4, temp);

      front_edge(0) = endeff->X.pos.x;
      front_edge(1) = endeff->X.pos.y;
      front_edge(2) = endeff->X.pos.z;

      cout<<" front_edge" <<front_edge <<endl;

      width = rear_edge(1) - front_edge(1);

      cout<< "FOUND WIDTH = "<<width<<endl;



      //////////////////////////////////////////////////////////////////////
      // GOAL REACHING
      /////////////////////////////////////////////////////////////////////



      target(0) = (right_edge(0) + left_edge(0))/2; //very far to the right
      target(1) = (rear_edge(1) + front_edge(1))/2;
      target(2) = height;

      OnlineSubmodularity(tableW, tableL, world, i, target, 5, temp);


 }



  return 0;
}


