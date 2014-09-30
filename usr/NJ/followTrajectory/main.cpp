#define MT_IMPLEMENTATION

#include <MT/robot.h>
#include <signal.h>
//#include <monitor.h>

struct MyDemo:public TaskAbstraction {
	arr q;
	uint q_index;
	bool started_trajectory;
  
	
  virtual void updateTaskVariables(ControllerModule *ctrl); //overloading the virtual
	void init(RobotProcessGroup *robot);
	
	void loadPlainTrajectory(const char* filename);
	void followTrajectory(RobotProcessGroup *robot);
};

/* DANGER: when we all start defining different task
   variables it gets confusing - so let's try to use the predefined ones

void MyDemo::initTaskVariables(){
  eff = new TaskVariable("endeffector",ors, posTVT,"m9","<t(0 0 -.24)>",0,0,0);
  eff->prec = MT::Parameter<double>("TV_x_prec");
  eff->active=true;    eff->targetType=directTT;
  sys.setTaskVariables(TUPLE(eff)); //that's important
}*/

ofstream positions_file("positions.dat");


void MyDemo::init(RobotProcessGroup *robot){
	cout << "init TV_q = "<<TV_q->y << endl;
	cout << "init TV_x->x="<<TV_eff->y << endl;
	
	MT::IOraw = true;
	
	robot->gui.gl->camera.setPosition(3.5,-8.,2.8);  // position of camera
        robot->gui.gl->camera.focus(0., -0.5, 1.);  // rotate the frame to focus the point (x,y,z)
	
	started_trajectory = false;
}


void MyDemo::updateTaskVariables(ControllerModule *ctrl){
	activateAll(TVall,false); //deactivate all variables
	
	TV_lim->active=true;
	TV_lim->y_prec = 1e1;
	TV_col->active=true;
 	TV_col->y_prec=1e-1;
	TV_eff->active = true;
	TV_eff->y_prec = 0.;
	
	if (!started_trajectory && maxDiff(ctrl->q_home, TV_q->y)<0.001) {
		cout<<"Starting trajectory."<<endl;
		started_trajectory = true;
	}
	
	
	if (started_trajectory) {
		TV_q->active = true;
		TV_q->y_prec = 0.9*1e1;
		//TV_q->vprec = 0.;
		TV_q->y_target = q[q_index];
		TV_q->v_prec=TV_q_vprec*1.5;  TV_q->v_target.setZero(); //damping on joint velocities
		q_index++;
	}
	else {
		TV_q->active = true;
		TV_q->y_prec = 0.;
		TV_q->v_prec = 1.;
		TV_q->v_target = ctrl->q_home - TV_q->y;
		double vmax = .5, v=norm(TV_q->v_target);
		if (v>vmax) TV_q->v_target *= vmax/v;
	}
}

void MyDemo::followTrajectory(RobotProcessGroup *robot){
  std::ostringstream filenameL, filenameR;
  for(;!robot->signalStop || (q_index == q.d0);){
		if (q_index%100 == 1) {cout<<q_index << "  of " << q.d0 << endl;}
		positions_file <<TV_eff->y <<endl;
    robot->step();
   if(robot->gamepad.state(0)==16 || robot->gamepad.state(0)==32) break;
	if (q_index == q.d0-1) {
			started_trajectory = false;
			q_index = 0;
			cout<<"Random movements finished: Need to rehome."<<endl;
		}
  }
}


void MyDemo::loadPlainTrajectory(const char* filename){
  ifstream fil;
  MT::open(fil,filename);
	q.read(fil);
  fil.close();
	q_index = 0;
}


int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);
  RobotProcessGroup robot;

  MyDemo demo;
  robot.ctrl.task=&demo;
  
  
  robot.open();
  
  demo.loadPlainTrajectory("Q.txt");
 demo.init(&robot);

demo.followTrajectory(&robot);
//when finished -- splot 'positions.dat'
  
  return 0;
}


