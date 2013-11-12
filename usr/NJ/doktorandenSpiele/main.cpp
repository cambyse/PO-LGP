// #define MT_IMPLEMENTATION

#include <MT/robot.h>
#include <signal.h>
#include <monitor.h>

struct MyTask:public TaskAbstraction{
	ors::Vector c;//(0,-0.8,1);
	ors::Vector y;//(0,-1,0);
	double target_radius;
	
	void init();
	void loop(RobotController& robot);
	
  virtual void updateTaskVariables(RobotController*); //overloading the virtual
};



ofstream data_file("positions.dat");
ofstream e_file("e.dat");
ofstream eneu_file("e_neu.dat");
ofstream ev_file("ev.dat");


void MyTask::init(){
// 	cout << "init q_ors = "<<q_ors << endl;
// 	cout << "init TV_eff->y="<<TV_eff->y << endl;
	
	
	ors::Vector e;
	e.set(TV_eff->y.p);
	c(0) = e(0); c(1) = e(1); c(2) = e(2)-0.15;
	target_radius = (e-c).length();
	
	
}



void MyTask::updateTaskVariables(RobotController *ctrl) {
  activateAll(TVall,false); //deactivate all variables
  ctrl->useBwdMsg=false;             //deactivate use of bwd messages (from planning)
      
	TV_q->active=true;
	TV_q->y_prec=1.;   TV_q->v_prec=TV_q_vprec;  TV_q->v_target.setZero(); //damping on joint velocities
	
	TV_lim->active=true;
	TV_lim->y_prec = 1e-2;
	
// 	TV_col->active=true;
// 	TV_col->y_prec=1e-2;
	
  TV_eff->active=true;
	MT::IOraw = true;

	ors::Vector e;
	e.set(TV_eff->y.p);
	y = ors::Vector(0,-1,0);
	
	ors::Vector c2e = e - c;
	ors::Vector x = c2e ^ y;
	x = x / x.length() * 0.25;
	#if 0
	ors::Vector q = e + x;
	ors::Vector c2q = q - c;
	ors::Vector c2q_scaled = c2q / c2q.length() * target_radius;
	ors::Vector e_neu = c + c2q_scaled;
	ors::Vector directions = e_neu - e;
	TV_eff->v_target = arr(directions.v,3);
	#endif
 	TV_eff->v_prec =1e2;
	TV_eff->v_target = arr(x.v,3);
	
// 	cout<<"Geplant: "<<(e_neu-c).length() - target_radius << endl;
//  	TV_eff->y_prec =1e4;
//  	TV_eff->y_target = arr(e_neu.v,3);
	
	
	
// 	eneu_file<<e_neu<<endl;
// 	e_file<<e<<endl;
// 	ev_file << e_neu -e << endl;
// 	e = e_neu;
}


void MyTask::loop(RobotController& robot){
  for(;!robot.signalStop;){ //catches the ^C key
    robot.step();
//     if(TV_eff->y(2)>1.2) break;
    if(robot.joy.state(0)==16 || robot.joy.state(0)==32) break;
    data_file <<TV_eff->y <<endl;
// 		cout << norm(arr(c.v,3) - TV_eff->y) -target_radius<< endl;
  }
}










struct MyHandTask:public TaskAbstraction{
	bool in_move;
	
	void init();
	void loop(RobotController& robot);
	
  virtual void updateTaskVariables(RobotController*); //overloading the virtual
};



void MyHandTask::init() {
	in_move = true;
}


void MyHandTask::updateTaskVariables(RobotController* ctrl) {
	activateAll(TVall,false); //deactivate all variables
  ctrl->useBwdMsg=false;             //deactivate use of bwd messages (from planning)
  
	TV_col->active = true;
	TV_col->y_prec=10000.;
	
	TV_q->active=true;
	TV_q->y_prec=1.;   
	TV_q->v_prec=1e-2;
// 	TV_q->v_prec=TV_q_vprec;  
	TV_q->v_target.setZero(); //damping on joint velocities
	
	TV_lim->active=true;
	TV_lim->y_prec = 1e-2;

	
	TV_q->y_target = TV_q->y;
	
	double GOAL_OUT_LOW = -.05;
	double GOAL_OUT_HIGH = -.05;
	double GOAL_IN_LOW = .01;
	double GOAL_IN_HIGH = .01;
	
	if (TV_q->y(10) < GOAL_OUT_LOW + 0.01)
		in_move = false;
	if (TV_q->y(10) > GOAL_IN_LOW - 0.01)
		in_move = true;
	
	if (in_move) {
		TV_q->y_target(10) = GOAL_OUT_LOW;
		TV_q->y_target(11) = GOAL_OUT_HIGH;
		TV_q->y_target(12) = GOAL_OUT_LOW;
		TV_q->y_target(13) = GOAL_OUT_HIGH;
		TV_q->y_target(14) = GOAL_OUT_LOW;
		TV_q->y_target(15) = GOAL_OUT_HIGH;
	}
	else {
		TV_q->y_target(10) = GOAL_IN_LOW;
		TV_q->y_target(11) = GOAL_IN_HIGH;
		TV_q->y_target(12) = GOAL_IN_LOW;
		TV_q->y_target(13) = GOAL_IN_HIGH;
		TV_q->y_target(14) = GOAL_IN_LOW;
		TV_q->y_target(15) = GOAL_IN_HIGH;
	}
// 	cout << "collision " << TV_col->y << endl;
}











struct MySkinTask:public TaskAbstraction{
	void init();
	void loop(RobotController& robot);
	
  virtual void initTaskVariables(RobotController*); //overloading the virtual
  virtual void updateTaskVariables(RobotController*); //overloading the virtual
};


void MySkinTask::initTaskVariables(RobotController* ctrl) {
	TaskAbstraction::initTaskVariables(ctrl);
}


void MySkinTask::updateTaskVariables(RobotController* ctrl) {
  if(ctrl->openSkin){
    TV_skin->y = ctrl->skin.y_real; //can get this state only from sensors!
    //cout <<"\r" <<TV_skin->y <<flush;
  }else{
    TV_skin->y=ARR(.01,0,.01,0,.01,0);
  }
  
  ctrl->q_ors(0)=0.;
  ctrl->q_ors(1)=0.;
  ctrl->q_ors(2)=0.;
  ctrl->q_ors(3)=0.;
  ctrl->q_ors(4)=0.;
  ctrl->q_ors(5)=0.;
  ctrl->q_ors(6)=0.;
  ctrl->v_ors(0)=0.;
  ctrl->v_ors(1)=0.;
  ctrl->v_ors(2)=0.;
  ctrl->v_ors(3)=0.;
  ctrl->v_ors(4)=0.;
  ctrl->v_ors(5)=0.;
  ctrl->v_ors(6)=0.;
  
	activateAll(TVall,false); //deactivate all variables
  ctrl->useBwdMsg=false;             //deactivate use of bwd messages (from planning)
  
// 	TV_col->active = true;
// 	TV_col->y_prec=10000.;
	
	TV_q->active=true;
	TV_q->y_prec=1e-1;   
	TV_q->v_prec=1e-2;
// 	TV_q->v_prec=TV_q_vprec;  
  TV_q->y_target.setZero(); //soft homing
	TV_q->v_target.setZero(); //damping on joint velocities
	
// 	TV_lim->active=true;
// 	TV_lim->y_prec = 1e-2;

	cout<<"TV_skin->y = " << TV_skin->y << endl;
	
	TV_skin->active = true;
	TV_skin->y_target = ARR(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	TV_skin->y_prec = 100;
}




#include "syscall.h" 

int main(int argc,char** argv){
  
  
  
  pid_t tid;
  tid = syscall(SYS_gettid);
  int priority = 10;
  int ret = setpriority(PRIO_PROCESS, tid, priority);
  cout << "tid = " << tid << endl;
  cout << "ret = " << ret << endl;
  
  
  
  
  
  MT::initCmdLine(argc,argv);
  signal(SIGINT,RobotController::signalStopCallback);
  RobotController robot;
	
/* 	MyTask task;
  robot.task=&task;
  
	robot.open();
	task.init();
        
	robot.gui->gl->camera.setPosition(3.5,-8.,2.8);  // position of camera
  robot.gui->gl->camera.focus(0., -0.5, 1.);  // rotate the frame to focus the point (x,y,z)
        
        
	task.loop(robot);
  robot.close();*/
	
	
// 	MyHandTask task;
	MySkinTask task;
	
  robot.task=&task;
  
	robot.open();
  if(robot.gui){
// 	robot.gui->threadWait();
    robot.gui->width=800;
    robot.gui->height=800;
    robot.gui->gl->camera.setPosition(3.5,-8.,2.8);  // position of camera
    robot.gui->gl->camera.focus(0., -0.5, 1.);  // rotate the frame to focus the point (x,y,z)
  }	
	for(;!robot.signalStop;){ //catches the ^C key
    robot.step();
    if(robot.joy.state(0)==16 || robot.joy.state(0)==32) break;
    //cout <<task.TV_eff->y <<endl;
  }
	
  robot.close();
	
  
  return 0;
}




