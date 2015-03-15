#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motionHeuristics.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Ors/ors_swift.h>

void pickandplace(arr finalpos){
  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));
  for(ors::Shape*s: G.shapes) s->cont = true;
  MT::Array<const char*> targets = {"leg1","leg2","leg3","leg4","chair_back_main","chair_sitting"};
  const char* actuator = "l_wrist_roll_link";
  uint current =5;

  for(ors::Shape *s: G.shapes)
    if (s->body->inLinks.N>0 ) s->mesh.makeConvexHull();


  G.watch(true);

  arr x, xT;

for (uint i=0; i<3;i++){

current = 5-i*2;
#if 1
  MotionProblem MP(G);
//G.watch(true);
  MP.loadTransitionParameters();
  MP.H_rate_diag = pr2_reasonable_W(G);

  ors::Shape *s = G.getShapeByName(targets(current));
 

    threeStepGraspHeuristic(xT, MP, s->index, 0);

    MotionProblemFunction F(MP);

    sineProfile(x, MP.x0, xT, MP.T);

  ///  optNewton(x, Convert(F), OPT(verbose=0, stopIters=100, useAdaptiveDamping=false, damping=1e-0, stopTolerance=1e-2, maxStep=.5));
   // MP.costReport();
  // gnuplot("load 'z.costReport.plt'", false, true);

    displayTrajectory(x, 1, G, "planned trajectory");
 
  //  G >>FILE("z.ors");

    MP.x0 = x[MP.T-1];
    MP.prefix.clear();
//! another task

#endif
cout << "DIM = "<<G.getJointStateDimension();
 G.glueBodies(G.getBodyByName(actuator),G.getBodyByName(targets(current)));

 G.getBodyByName(targets(current))->inLinks(0)->type =ors::JT_fixed;
 G.getBodyByName(targets(current))->inLinks(0)->name ="test";
//G.getJointByName("l_gripper_l_finger_joint")->agent =0;
// G.getBodyByName(targets(current))->inLinks(0)->qIndex = G.getJointStateDimension();
// G.getBodyByName(targets(current))->inLinks(0)->agent = G.getBodyByName(actuator)->inLinks(0)->agent;

 G.calc_fwdPropagateFrames();	
 //G.swift().initActivations();
 //  G.watch(true);

  MotionProblem MP2(G);
  MP2.loadTransitionParameters();
  MP2.H_rate_diag = pr2_reasonable_W(G);
    arr  xT2;
cout <<"DIM =" <<G.getJointStateDimension();
//x2=x;


  //-- setup the motion problem
finalpos(0) -= 0.2 * i;
  Task *c;
 c = MP2.addTask("position", new DefaultTaskMap(posTMT, G, targets(current), ors::Vector(0, 0, 0)));
 MP2.setInterpolatingCosts(c, MotionProblem::finalOnly, finalpos, 1e3);

 //c = MP2.addTask("orientation", new DefaultTaskMap(quatTMT, G, targets(current), ors::Vector(0, 0, 0)));
 //MP2.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(G.getBodyByName("reference")->X.rot), 1e2);


// ors::Quaternion my_quat; my_quat.set(0, 0, 0, 1);
// MP2.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(my_quat), 1e3);


ors::Vector orient;  orient.set(0, 0, 1) ;
if (current<5) orient.set(0,1,0);

//  c = MP2.addTask("upAlign", new DefaultTaskMap(vecAlignTMT, G, targets(current), ors::Vector(1, 0, 0),"reference", orient ,NoArr));
 
//if (current<5) MP2.setInterpolatingCosts(c, MotionProblem::finalOnly, ARR(-1.), 1e3);
 // MP2.setInterpolatingCosts(c, MotionProblem::finalOnly, ARR(-1.), 1e3);
  //c = MP2.addTask("orientation", new DefaultTaskMap(vecTMT, G, targets(current), ors::Vector(0, 0, 0)));
  //MP2.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,1.), 1e3);
/*
  c = MP2.addTask("q_vel", new TaskMap_qItself());
  c->map.order=1; //make this a velocity variable!
  MP2.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);
*/
 //   c = MP2.addTask("collision",
 //                  new DefaultTaskMap(collTMT, G, NULL, NoVector, NULL, NoVector, ARR(.1)));
 //   MP2.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);
 
cout << "TEST\n";
   MotionProblemFunction MF2(MP2);
//sineProfile(x2, MP2.x0, xT2, MP2.T);
//initial
uint T = MF2.get_T(); 
//arr x(T+1,MF2.dim_x()); 
for(uint t=0;t<=T;t++) x[t]() = MP2.x0;

  // optNewton(x, Convert(MF2), OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-0, stopTolerance=1e-2, maxStep=.5));
   MP2.costReport();
   displayTrajectory(x, 1, G, "planned trajectory",0.05);
 //! remove the joint
  G.getBodyByName(actuator)->outLinks.removeValue(G.getJointByName("test"));
  G.getBodyByName(targets(current))->inLinks.removeValue(G.getJointByName("test"));
  G.joints.removeValueSafe(G.getJointByName("test"));
  G.calc_fwdPropagateFrames();	
 //G.swift().initActivations();

    MP2.x0 = x[MP2.T-1];
    MP2.prefix.clear();
}
 //! end of remove
 
 //  displayTrajectory(x, 1, G, "planned trajectory");
}

void testPickAndPlace(const char* target,arr finalpos){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  arr positions;
  positions.resize(5,7);
   ifstream out3("constraints.txt"); positions.readRaw(out3); out3.close();
  //setup the problem
  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));
  makeConvexHulls(G.shapes);

//for(ors::Shape *s: G.shapes)
//    if (s->body->inLinks.N>0 ) s->mesh.makeConvexHull();
  G.watch(true);

  MotionProblem MP(G);
  MP.loadTransitionParameters();
  MP.H_rate_diag = pr2_reasonable_W(G);

  arr x, xT;
  MT::Array<const char*> targets = {"leg1","leg2","leg3","leg4","chair_back","chair_sitting"};

for (uint i=0;i<5;i++)  {
   // finalpos = ARRAY(0.0,-1.0,0.8);
//    finalpos = ARRAY(G.getBodyByName("chair_sitting")->X.pos);

  threeStepGraspHeuristic(xT, MP, G.getShapeByName(targets(i))->index, 2);

  MotionProblemFunction MF(MP);
  sineProfile(x, MP.x0, xT, MP.T);
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, damping=1e-0, stopTolerance=1e-2, maxStep=.5));
  MP.costReport();
  gnuplot("load 'z.costReport.plt'", false, true);

  displayTrajectory(x, -1, G, "planned trajectory");

  G.glueBodies(G.getBodyByName("l_wrist_roll_link"), G.getBodyByName(targets(i)));
  G.getShapeByName(target)->cont=false;
  //!!!!!!G.getShapeByName("target2")->cont=false;
  //G.getShapeByName("target")->cont=false;
  G.swift().initActivations(G);
  listDelete(MF.configurations);


  //-- setup new motion problem
  MP.prefix.clear();
  listDelete(MP.taskCosts);
  MP.x0 = x[MP.T-1];

  ors::Vector current; current.set(positions(i,0),positions(i,1),positions(i,2));
  G.getBodyByName("reference")->X.rot.set(0,0,0,1);

//  arr relativ; relativ =ARRAY(G.getBodyByName("chair_sitting")->X*current);
//  relativ.reshape(relativ.N);
//   for (uint j=0;j<3;j++)
//    finalpos(j)+=relativ(j);//positions(i,j);
 // finalpos = ARRAY(G.getBodyByName("chair_sitting")->X*current);
  finalpos = ARRAY(G.getBodyByName("chair_sitting")->X*current);
cout << "POS = "<<G.getBodyByName("chair_sitting")->X<<"---------"<< finalpos<< endl;


  Task *c;
  double shift; if (i>3) shift=0; else shift =  -0.18;
  c = MP.addTask("position", new DefaultTaskMap(posTMT, G, targets(i), ors::Vector(0, 0,shift)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, finalpos, 1e3);

  c = MP.addTask("q_vel", new TaskMap_qItself());
  c->map.order=1; //make this a velocity variable!
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);

 // c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .04));
 // MP.setInterpolatingCosts(c, MotionProblem::constant, NoArr, 1e-0);

  G.getBodyByName("reference")->X.rot.set(0,0,0,1);
  if (i>3) G.getBodyByName("reference")->X.rot.set(0,0.7,0.7,0);

  c = MP.addTask("orientation", new DefaultTaskMap(quatTMT, G, targets(i), ors::Vector(0, 0, 0)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(G.getBodyByName("chair_sitting")->X.rot*G.getBodyByName("reference")->X.rot), 1e3);
//
  //initialize trajectory
  for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

  //-- optimize
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
  MP.costReport();

  displayTrajectory(x, 1, G, "planned trajectory", .01);

  delete G.joints.last();
  G.getShapeByName(target)->cont=false;
 // G.getShapeByName("target2")->cont=false;
 // G.getShapeByName("target")->cont=false;
  G.swift().initActivations(G);

  //-- setup new motion problem
  MP.prefix.clear();
  listDelete(MP.taskCosts);
  MP.x0 = x[MP.T-1];
}
/*
  c = MP.addTask("position", new DefaultTaskMap(posTMT, G, "graspCenter", ors::Vector(0, 0, 0)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(MP.world.getShapeByName("target2")->X.pos), 1e3);

  c = MP.addTask("q_vel", new TaskMap_qItself());
  c->map.order=1; //make this a velocity variable!
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);

  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .04));
  MP.setInterpolatingCosts(c, MotionProblem::constant, NoArr, 1e-0);

  //initialize trajectory
  for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

  //-- optimize
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
  MP.costReport();

  displayTrajectory(x, 1, G, "planned trajectory", .1);
*/
}

void test_Loading_submeshes()
{
    ors::Mesh mesh;
    mesh.readObjFile(FILE("chair_back_decomposed.obj"));
    OpenGL gl;
    gl.add(ors::glDrawMesh, &mesh);
    gl.watch();
}

void AssembleChair(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;
 int show_steps = -1;
  arr positions;
  positions.resize(5,7);
   ifstream out3("constraints.txt"); positions.readRaw(out3); out3.close();
  //setup the problem
   ors::KinematicWorld G("ikea2.kvg.txt");
  makeConvexHulls(G.shapes);
  arr initial = G.getJointState();

//for(ors::Shape *s: G.shapes)
//    if (s->body->inLinks.N>0 ) s->mesh.makeConvexHull();
//  G.watch(true);

  MotionProblem MP(G);
  MP.loadTransitionParameters();
  MP.H_rate_diag = pr2_reasonable_W(G);

  arr x, xT;
  MT::Array<const char*> targets = {"leg1","leg2","leg3","leg4","chair_back","chair_sitting"};
  arr finalpos; ors::Vector current; ors::Quaternion original; ors::Quaternion orientation;
   original.set(sqrt(0.5),-sqrt(0.5),0,0);
  //original.set(0,0,0,1);


#if 0
  for (uint i=0;i<5;i++)  {

      if (i<4) current.set(positions(i,0),positions(i,1)-0.16,positions(i,2));
       else current.set(positions(i,0),positions(i,1)+0.1,positions(i,2));

      finalpos = ARRAY(G.getShapeByName("chair_sitting_main")->X*(original*current));
      G.getShapeByName(targets(i))->X.pos = finalpos;
      orientation.set(0,0,0,1);
       if (i>3) orientation.set(0,0.7,0.7,0);
      G.getShapeByName(targets(i))->X.rot =G.getShapeByName("chair_sitting_main")->X.rot*orientation;
      //G.calc_fwdPropagateShapeFrames();
      G.watch(true);
}
#else
for (uint i=0;i<5;i++)  {
      threeStepGraspHeuristic(xT, MP, G.getShapeByName(targets(i))->index, 0);
      MotionProblemFunction MF(MP);
      sineProfile(x, MP.x0, xT, MP.T);
      optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, damping=1e-0, stopTolerance=1e-2, maxStep=.5));
      MP.costReport();
      gnuplot("load 'z.costReport.plt'", false, true);

      displayTrajectory(x, -1, G, "planned trajectory");

      G.glueBodies(G.getBodyByName("l_wrist_roll_link"), G.getBodyByName(targets(i)));
      G.swift().initActivations(G);
      listDelete(MF.configurations);

      //-- setup new motion problem
      MP.prefix.clear();
      listDelete(MP.taskCosts);
      MP.x0 = x[MP.T-1];

      if (i<4) current.set(positions(i,0),positions(i,1)-0.16,positions(i,2));//! strange offset
       else current.set(positions(i,0),positions(i,1)-0.04,positions(i,2));

    // current.set(positions(i,0),positions(i,1),positions(i,2));

      finalpos = ARRAY(G.getShapeByName("chair_sitting_main")->X*(original*current));
     // finalpos = ARRAY(G.getShapeByName("chair_sitting_main")->X.pos);
         cout << "POS = "<< finalpos<< endl;

      Task *c;
      c = MP.addTask("position", new DefaultTaskMap(posTMT, G, targets(i), ors::Vector(0, 0,0)));
      MP.setInterpolatingCosts(c, MotionProblem::finalOnly, finalpos, 1e3);

      c = MP.addTask("q_vel", new TaskMap_qItself());
      c->map.order=1; //make this a velocity variable!
      MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);

      //c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .04));
      //MP.setInterpolatingCosts(c, MotionProblem::constant, NoArr, 1e-2);

     orientation.set(0,0,0,1);
      if (i>3) orientation.set(0,0.7,0.7,0);

      c = MP.addTask("orientation", new DefaultTaskMap(quatTMT, G, targets(i), ors::Vector(0, 0, 0)));
      MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(G.getShapeByName("chair_sitting_main")->X.rot*orientation), 1e3);
    //
      //initialize trajectory
      for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

      //-- optimize
      optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
      MP.costReport();

      displayTrajectory(x, show_steps, G, "planned trajectory", .01);

      delete G.joints.last();
      //G.getShapeByName(target)->cont=false;
     // G.getShapeByName("target2")->cont=false;
     // G.getShapeByName("target")->cont=false;
      G.swift().initActivations(G);

      //-- setup new motion problem
      MP.prefix.clear();
      listDelete(MP.taskCosts);
      MP.x0 = x[MP.T-1];
      //! go to intermediate state

      c = MP.addTask("q_vel", new TaskMap_qItself());
      c->map.order=1; //make this a velocity variable!
      MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);

      c = MP.addTask("q_itself", new TaskMap_qItself());
     // c->map.order=1; //make this a velocity variable!
      MP.setInterpolatingCosts(c, MotionProblem::finalOnly,initial, 1e1);


      //initialize trajectory
      for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

      //-- optimize
      optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
      MP.costReport();

      displayTrajectory(x, show_steps, G, "planned trajectory", .001);
      MP.prefix.clear();
      listDelete(MP.taskCosts);
      MP.x0 = x[MP.T-1];

}
 MotionProblemFunction MF(MP);
 Task *c;

  c = MP.addTask("q_vel", new TaskMap_qItself());
  c->map.order=1; //make this a velocity variable!
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);


  //initialize trajectory
  for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

  //-- optimize
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
  MP.costReport();

  displayTrajectory(x, 1, G, "planned trajectory", .001);

#endif
}
//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

// testPickAndPlace("leg4",ARRAY(0.0,-1.0,0.8));
  AssembleChair();
// test_Loading_submeshes();
//testPickAndPlace("leg4",ARRAY(1.0,-1.0,0.5));
 // pickandplace(ARRAY(1.0,1.0,0.5));
  return 0;
}
