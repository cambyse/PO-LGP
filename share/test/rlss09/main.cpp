//#define MT_IMPLEMENTATION

#include <MT/ors.h>
#include <MT/soc.h>
#include <MT/socSystem_ors.h>
#include <MT/soc_inverseKinematics.h>
#include <MT/opengl.h>
#include <MT/aico.h>


int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  //===========================================================================
  //
  // initialization of the simulator (should be ok for the lab)
  
  // load the ors file
  ors::Graph ors;
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("iCub.ors")));
  // start the collision computation engine
  SwiftInterface swift;
  swift.init(ors,.1);               //.1 is the threshold for distance consideration
  // start the OpenGL engine
  OpenGL gl;
  gl.add(glStandardScene);          //add a standard static draw routine for the floor...
  gl.add(ors::glDrawGraph,&ors);    //adds a draw routine to draw the geometry
  gl.camera.setPosition(5,-10,10);  //sets the perspective...
  gl.camera.focus(0,0,1);
  gl.watch("loaded configuration - press ENTER");
  // init the SocAbstraction (tying together the simulator, the collision engine, and the display)
  uint T=200;
  soc::SocSystem_Ors soc;
  soc.initBasics(&ors,&swift,&gl,T,3.,MT::getParameter<bool>("dynamic",false),NULL);
  soc.os=&std::cout;

  //===========================================================================
  //
  // defining the task variables -- that defines the actual motion problem!

  //TaskVariable(name of variable, simulator, type of variable,
  //                name of 1st reference body, extra relative transformation of 1st reference,
  //                name of 2nd reference body, extra relative transformation of 2nd reference,
  //                array of parameters);
  
  // the endeffector task variable (3D)
  TaskVariable *pos = new TaskVariable("position",ors,posTVT, "endeff","<t(0 .04 0)>", 0,0, ARR());
  pos->y_target = arr(ors.getBodyByName("target")->X.pos.p,3);  //set its final target equal to the current position of "target"
  
  // the collision task variable (1D)
  TaskVariable *col = new TaskVariable("collision",ors,collTVT, 0,0, 0,0, ARR(.05)); //collision margin=5cm
  col->y_target = ARR(0.);   //set the target equal to ZERO collision

  // a 3D variable representing the `palm normal' of the endeffector (to control its orientation)
  //TaskVariable *handup = new TaskVariable("hand up",ors,zoriTVT, "endeff","<d(90 0 1 0)>", 0,0, ARR());
  //handup->x_target = ARR(0.,0.,1.);   //set the target to pointing upwards (in world coordinates)

  // tell SOC that these variables exists!
  soc.setTaskVariables(ARRAY(pos,col));  //,handup));


  //===========================================================================
  //
  // online generation of movement -- using inverse kinematics

  // defining the desired behavior of task variables for the online (non-planned) movement
  pos->setGainsAsAttractor(20,.2);         //choose an attractor dynamics
  col->setGains(.5,.0);   col->y_prec=1e-0;  //choose gains and precision
  //handup->setGainsAsAttractor(20,.2);    //choose an attractor dynamics
  
  arr q,qnew;
  soc.getq0(q); //get the current state (array of joint angles)
  for(uint t=0;t<T;t++){
    soc::bayesianIKControl2(soc,qnew,q,0); //compute an IK step
    q = qnew;                          //add the step to the state
    soc.setq(q);                      //set the new state
    //soc.reportOnState(cout);        //->would generate detailed ouput on the state of all variables...
    gl.update(STRING("bayesian Inverse Kinematics: iteration "<<t)); //display with OpenGL
    //gl.watch();
  }
  gl.watch("<press ENTER>");
  
  
  //===========================================================================
  //
  // planning (AICO) to generate an optimal trajectory

  // defining the desired behavior of task variables for the planned movement
  
  soc.getq0(q); //get the start posture
  soc.setq(q);  //set this as the current state
  
  pos->setInterpolatedTargetsEndPrecisions(T,1e-3,1e2,0.,1e3);
  //pos should follow linear interpolation to the target
  //with minimal precision on the way, but very high precision at the final step
  //with no velocity precision on the way, but high velocity precision at the final step

  //pos->x_trajectory[50]()=ARR(0,-.3,.5);
  //pos->prec_trajectory(50)=1e2;
  
  //handup->setInterpolatedTargetTrajectory(T);      //pos should follow linear interpolation to the target
  //handup->setPrecisionTrajectoryFinal(T,1e-3,1e2); //with minimal precision on the way, but very high precision at the final step
  
  col->setInterpolatedTargetsConstPrecisions(T,1e-2,0.);
  //col should constantly be on the target (=zero)

  //do the planning!
  q.clear();
  //AICO_solver(soc,q,40,.7,.01,0,0);
  AICO aico(soc);
  aico.iterate_to_convergence();
  q = aico.q;
  
  //write the trajectory in a file
  ofstream os("z.traj");
  q.writeRaw(os);
  os.close();

  //display forever
  for(;;) soc.displayTrajectory(q,NULL,1,"AICO (planned trajectory)");
  
  return 0;
}
