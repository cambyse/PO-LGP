/*
 * ORSImitate.h
 *
 *  Created on: Nov 30, 2010
 *      Author: nikolay
 */

#ifndef ORSIMITATE_H_
#define ORSIMITATE_H_

//#include "ors.h"
#include <MT/robot.h>

struct ImitateTask:public TaskAbstraction{

	virtual void updateTaskVariables(ControllerModule*); //overloading the virtual
	virtual void initTaskVariables(ControllerModule*);
	TaskVariable * TVImitate;

};

struct ImitateTaskVariable:public TaskVariable{
	MT::Array<ors::Shape*> landmarks;
	arr lastFeature,lastJoint;
	virtual void userUpdate();
	ImitateTaskVariable(const char* _name, ors::Graph& _ors);
	int nCounter;
	int nIterations;//when finding argin and using some last state
};


void PlanTrajectory(ors::Graph * ors,RobotProcessGroup & robot,GraspObject* objM){//imitate variable and collisions, plan
	soc::SocSystem_Ors sys;

	uint T=MT::getParameter<uint>("T");
	sys.initBasics(ors,NULL,NULL,T,6.,true,NULL);//3rd is gl, disabled
	sys.os = &cout;
	//sys.dynamic = true;//needed to have velocity targets
	TaskVariable * TVImitate = new ImitateTaskVariable("grasp imitate",*sys.ors);
	TVImitate->active = true;
	TVImitate->setInterpolatedTargetsConstPrecisions(T,0,1e2);//only velocity targets
	TVImitate->v_trajectory = -0.005;//code constant decrease
	TVImitate->y_target = -0.15;//functional MLP form can give us hint of maximum, 0.08 for linear allW3, -0.15 for allW12
	TVImitate->setInterpolatedTargetsEndPrecisions(T,1e-5, 1e3, 0,0);
	TaskVariable *TV_col  = new TaskVariable("collision", *sys.ors, collTVT,0,0,0,0,ARR(.03)); //MARGIN, perhaps .05?
	TV_col->active = true;
	TV_col->setInterpolatedTargetsConstPrecisions(T,1e1,0);
	TV_col->y_trajectory = 0.0;

	arr limits;
	limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
			-1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
	TaskVariable *TV_lim  = new TaskVariable("limits", *sys.ors, qLimitsTVT,0,0,0,0,limits);
	TV_lim->active = true;
	TV_lim->setInterpolatedTargetsConstPrecisions(T,1e-1,0);
	TV_lim->y_trajectory = 0.0;

	TaskVariable * TVPos = new TaskVariable("posArm", *sys.ors,posTVT,"tip1",0,0,0,0);
	TVPos->updateState();
	TVPos->y_target = arr(sys.ors->getBodyByName("o1")->X.pos.p,3);
	TVPos->setInterpolatedTargetsConstPrecisions(T,0,0);//no weight, just for initial path

	sys.setTaskVariables(TUPLE(TVImitate,TV_col,TV_lim,TVPos));

	ofstream f("planneriters.txt");
#ifdef AICO
	soc::AICO aico;
	aico.init(sys, 0.3, 0.,0.,0,0);
	aico.damping = 0;//role ???
#else
	soc::LQG aico;
	aico.init(sys, 0.4,0,0);
#endif
	soc::straightTaskTrajectory(sys, aico.q, 3);//soc::getPhaseTrajectory(aico.q,qK,sys.getTau());
	aico.cost = sys.analyzeTrajectory(aico.q,1);
	f << aico.cost << endl;

	int iter =MT::getParameter<int>("iterations");
	for(int i = 0; i < iter-1; i++){
		cout << "starting iter " << i << " out of " << iter << " cost " << aico.cost << endl;
		//if(sys.dynamic)
#ifdef AICO
			aico.stepDynamic();
#else
	aico.stepGeneral();
#endif;
		aico.cost = sys.analyzeTrajectory(aico.q,1);
		f << aico.cost << endl;
	}
	arr best = aico.q;
	cout << " o1" << sys.ors->getBodyByName("o1")->X.pos << endl;
	OpenGL gl;// = glold;
	gl.add(ors::glDrawGraph,sys.ors);
	gl.add(glDrawMeshObject,objM);
	gl.setClearColors(1.,1.,1.,1.);
	gl.camera.setPosition(.0,0.,10.);
	gl.camera.focus(.0,0,0);
	for(uint c = 0; c < 10; c++){
		for(uint t = 0; t < T; t++){
			MT::wait(0.05);
			sys.ors->setJointState(best[t]);
			sys.ors->calcBodyFramesFromJoints();		//	robot.gui.step();
			gl.update();
			gl.text.clear()<< t<< endl;
		}
		TVImitate->updateState();
		cout << " last value " << TVImitate->y << endl;
	}
}


#include "ORSImitate.cpp"
#endif
