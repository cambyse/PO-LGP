#define MT_IMPLEMENTATION
#define MT_GL

#include <MT/robot.h>
#include <signal.h>

int bHack = 0;//for changing validation heuristic
#include<SD/ISF_GP.h>
#include<SD/graspISF.h>
#include<SD/utils.h>
#include "EnergyRoutines.h"
#include "ORSImitate.h"

struct DummyTask:public TaskAbstraction{//do nothing
	DummyTask(){};
	//virtual void initTaskVariables(ControllerModule*){};
	virtual void updateTaskVariables(ControllerModule*){
		activateAll(TVall,false);
		TV_col->active=true;
		TV_col->y_prec = 0;
		TV_lim->active=true;
	};
};



int
get_gamepad_state(RobotProcessGroup& robot){
	if (!robot.openGamepad) return 0;
	return robot.gamepad.state(0);
}

PotentialField *
obj_pot_field(){

	GraspObject *obj;
	switch (MT::getParameter<uint>("obj_comes_from")) {
	case 0 : // vision
		switch (MT::getParameter<uint>("shape")) {
		case 2 :
			obj = new GraspObject_Cylinder1();
			break;
		case 0 :
			obj = new GraspObject_Sphere();
			break;
		default:
			HALT("You should be kidding with that shape!");
		}
		break;

		case 1 : // cmd line

			/* create the correct prior (only in case of GraspObject_GP_analytical_prior) */
			GraspObject *prior;
			switch (MT::getParameter<uint>("shapeprior", 0)) {
			case 3 :
				prior = new GraspObject_InfCylinder();
				break;
			case 2 :
				prior = new GraspObject_Cylinder1();
				break;
			case 1 :
				prior = new GraspObject_Sphere();
				break;
			default:
				prior = NULL;
			}


			/* create the right object type */
			switch (MT::getParameter<uint>("shape", 0)) {
			case 1 :
				obj = new GraspObject_InfCylinder();
				break;
			case 2 :
				obj = new GraspObject_Cylinder1();
				break;
			case 0 :
				obj = new GraspObject_Sphere();
				break;
			case 3 :
				obj = new GraspObject_GPblob();
				break;
			case 4 :
				if (!prior) HALT("You should be kidding with that shape!");
				obj = new GraspObject_GP_analytical_prior(prior);
				break;
			case 5 :
				// obj = new GraspObject_Cylinder1();
				obj = new GraspObject_Sphere();
				break;
			default:
				HALT("You should be kidding with that shape!");
			}
			break;
			default:
				SD_INF("provide meaningful obj_comes_from");
				SD_INF("0: vision (camera running), or");
				SD_INF("1: cmd line (-center '[.0 -.9 .99 ]' -shape 0 -height .25  -sigma 10 -radius .04) ");
				HALT("need object to grasp");
	}

	SD_INF("Building mesh, patience...");
	obj->buildMesh();
	return obj;
}


void
get_skin_state(GraspISFTask& task, RobotProcessGroup& robot ){
	if(task.open_skin){
		//robot.skin.lock.readLock();
		task.skin_state =
				ARR( robot.skin.y_real(0), robot.skin.y_real(2),robot.skin.y_real(4));
		//robot.skin.lock.unlock();
		// trimm to target value to avoid oxilation
		for(uint i=0;i<task.TV_skin->y.N;i++)
			if(task.TV_skin->y(i)>.02) task.TV_skin->y(i)=.02;
	}else{
		task.TV_skin->y=ARR(.01,.01,.01);
	}
}

void InitLoadedData(int nMode){
	bDynamical = MT::getParameter<int>("bDyn");
	if(nMode == 0) return;
	if (nMode == 0 && false){//pos has to exist BUGGED, ignore
		lastCenter = arr(3);
		arr temp;
		temp <<FILE("pos.txt");
		int index = temp.d0;
		if (index < 27){
			//27 totally, index to determine
			int c = index/9;
			lastCenter(0) = -0.2 + c*0.2;
			index = index%9 ;
			c = index/3;
			lastCenter(1) = -1.05 + c*0.15;
			index = index%3;
			c = index;
			lastCenter(2) = 0.85 + c*0.1;
		}
		else{
			rnd.seed(index);
			lastCenter(0) = -0.2 + rnd.uni()*0.4;
			lastCenter(1) = -1.05 + rnd.uni()*0.3;
			lastCenter(2) = 0.85 + rnd.uni()*0.2;
		}
		return;
	}
	sDiscriminant = MT::getParameter<MT::String>("sDiscriminant");
	if (nMode == 5){//batch
		arr temp;
		temp <<FILE(sDiscriminant + MT::String("/validate.txt"));
		int nTest = MT::getParameter<int>("nTest");
		int ns = floor(temp.d0/nTest) +1 ;//careful, divide number of test data situations
		sDiscriminant = sDiscriminant + MT::String("/allW") + ns;
	}
	if(meaN.N == 0){//load arrays
		meaN <<FILE(sDiscriminant + MT::String("/mean.txt"));
		prN <<FILE(sDiscriminant + MT::String("/pr.txt"));
	}
	if(w1.N == 0 && bDynamical < 2){
		w1 <<FILE(sDiscriminant + MT::String("/w1.txt"));
		w2 <<FILE(sDiscriminant + MT::String("/w2.txt"));
		w3 <<FILE(sDiscriminant + MT::String("/w3.txt"));
		//now scale w2 in a meaningful range, does not change discriminants
		double nSum = 0;
		for(uint i = 0; i < w2.d0; i++)
			if(w2(i,0) < 0)
				nSum += fabs(w2(i,0));
		for(uint i = 0; i < w3.d0; i++)
			//if(w2(i,0) < 0)
			nSum += fabs(w3(i,0));
		w2*=1/nSum;
		w3*=1/nSum;
	}

	if(w1.N == 0 && bDynamical == 2){//lin regression in task sapce
		w1 <<FILE(sDiscriminant + MT::String("/linW.txt"));

	}

	if(w1.N == 0 && bDynamical == 3){//x and u task spaces regressions
		//w1 <<FILE(sDiscriminant + MT::String("/linW.txt"));

		w1 <<FILE(sDiscriminant + MT::String("/w1.txt"));
		w2 <<FILE(sDiscriminant + MT::String("/w2.txt"));

	}
}

double JointLength(const arr & q){
	double ans = 0;
	for (uint i = 0; i < q.d0-1; i++)
		ans += norm(q[i]-q[i+1]);
	return ans;
}

double Validate(RobotProcessGroup & robotOld,RobotProcessGroup & robot2,const arr & c, int nMode, const arr & joints){
	bHack = 0;
	GraspISFTask task2;
	task2.graspobj = obj_pot_field();
	robot2.ctrl.task=&task2;
	robot2.open();
	task2.open_skin = robot2.openSkin;// skin -> graspISFtask
	get_skin_state(task2,robot2);
	if(robot2.openGui && false)robot2.gui.gl->addView(0,glDrawMeshObject,task2.graspobj);
	ors::Shape * o1 = robot2.ctrl.ors.getBodyByName("o1")->shapes(0);
	robot2.ctrl.ors.getBodyByName("o1")->X.pos = ors::Vector(c(0),c(1),c(2));
	o1->type = 3;//mesh
	o1->mesh.clear();
	o1->mesh = ((GraspObject*)task2.graspobj)->m;
	if(robot2.openGui){
		robot2.gui.ors->getBodyByName("o1")->X.pos = ors::Vector(c(0),c(1),c(2));
		robot2.gui.ors->getBodyByName("o1")->shapes(0)->type = 3;
		robot2.gui.ors->getBodyByName("o1")->shapes(0)->mesh.clear();
		robot2.gui.ors->getBodyByName("o1")->shapes(0)->mesh = o1->mesh;
	}
	arr qval = robotOld.ctrl.q_reference;//why discrepance btw qreference and ors joint state???
	cout << " qval " <<endl <<  qval << endl << robotOld.ctrl.q_reference << endl;
	robot2.ctrl.ors.setJointState(qval);
	robot2.ctrl.q_reference =  qval;

	for(uint i = 0; i < 1; i++){
		get_skin_state(task2,robot2);
		robot2.step();
		double C = robot2.ctrl.sys.taskCost(NULL,0,-1);
		if(robot2.openGui)
			robot2.gui.gl->text.clear()<< C << endl;
		//if() cout << " taskcost: " << C << " ";
	}
	//robot2.ctrl.task->updateTaskVariables(&robot2.ctrl);
	double cJ = JointLength(joints);
	double C = robot2.ctrl.sys.taskCost(NULL,0,-1);
	if(!( C > 0 && C < 1000))
		cout << " HACKKK " << endl << C << endl << "HACKK" << endl;
	if(nMode == 5 || nMode == 2 || nMode == 0){
		ofstream f(MT::getParameter<MT::String>("sDiscriminant") + MT::String("/validate.txt"),ofstream::app);
		f << C << " " << cJ << endl;//		cout << cJ ;
	}
	return C;
}

arr GetEndPath(const arr & joints, ors::KinematicWorld * ors){
	arr orig;
	ors->getJointState(orig);
	arr x(joints.d0,3);
	for(uint i = 0; i < joints.d0; i++ ){
		ors->setJointState(joints[i]);
		ors->calcBodyFramesFromJoints();
		x(i,0) = ors->getBodyByName("m9")->X.pos(0);
		x(i,1) = ors->getBodyByName("m9")->X.pos(1);
		x(i,2) = ors->getBodyByName("m9")->X.pos(2);
		if (i > 0 && norm(x[i]-x[i-1]) > 0.02)
			cout << joints[i] << " " << joints[i+1] << " " <<x[i] << " " << x[i-1] << endl;

	}
	ors->setJointState(orig);
	ors->calcBodyFramesFromJoints();
	return x;
}

int main(int argc,char** argv){
	MT::initCmdLine(argc,argv);
	int nMode = MT::getParameter<int>("Nmode");//0 is heuristic + noise, 1 is random sampling, 2 is gradient move, 7 is gradient check, 3 is planniong,5 is batch validate
	InitLoadedData(nMode);
	arr c = MT::getParameter<arr>("center");lastCenter = c;

	signal(SIGINT,RobotProcessGroup::signalStopCallback);
	GraspISFTask task;
	RobotProcessGroup robot;
	task.graspobj = obj_pot_field();// NJ: Assign object to task., in which place???

	if (nMode == 0){
		robot.ctrl.task=&task;
		bHack = 1;
	}
	else if (nMode == 2 || nMode == 5)
		robot.ctrl.task= new ImitateTask();
	else
		robot.ctrl.task= new DummyTask();

	plotClear();//deletes stupid mesh arrows
	robot.open();

	if(robot.openGui && true)//modify draw fnction....
		robot.gui.gl->addView(0,glDrawMeshObject,task.graspobj);

	if(nMode == 7){NJCH::NJCheck(&robot.ctrl.ors);	    return 0;}
	if (nMode == 0){
		task.open_skin = robot.openSkin;// skin -> graspISFtask
		get_skin_state(task,robot);
		cout << " starting grasp " << endl;
	}

	ors::Shape * o1 = robot.ctrl.ors.getBodyByName("o1")->shapes(0);//change this , otherwise features will deviate from reading
	robot.ctrl.ors.getBodyByName("o1")->X.pos = ors::Vector(c(0),c(1),c(2));

	ors::Quaternion rot;
	if (nMode != 3){//leave be in case of planner
		o1->type = 3;//mesh
		o1->mesh.clear();
		o1->mesh = ((GraspObject*)task.graspobj)->m;
		arr z = MT::getParameter<arr>("orientation");
		ors::Vector ax;robot.ctrl.ors.getBodyByName("o1")->X.rot.getZ(ax);
		rot.setDiff(ax,ors::Vector(z(0),z(1),z(2)));
		robot.ctrl.ors.getBodyByName("o1")->X.rot= robot.ctrl.ors.getBodyByName("o1")->X.rot*rot;
	}
	else{//make corect type
		o1->type = 1;//sphere
		o1->size[3] = MT::getParameter<double>("radius");
		o1->mesh.clear();
	}
	if(robot.openGui){
		robot.gui.ors->getBodyByName("o1")->X.pos = ors::Vector(c(0),c(1),c(2));
		robot.gui.ors->getBodyByName("o1")->shapes(0)->type = 3;
		robot.gui.ors->getBodyByName("o1")->shapes(0)->mesh.clear();
		robot.gui.ors->getBodyByName("o1")->shapes(0)->mesh = o1->mesh;

		robot.gui.ors->getBodyByName("o1")->X.rot= robot.gui.ors->getBodyByName("o1")->X.rot*rot;
		//robot.gui.threadStep();
		//	robot.gui.ors->getBodyByName("o1")->X.rot.getZ(ax);
		//	robot.ctrl.ors.getBodyByName("o1")->X.
		//	cout << "axis X" << ax << endl;
	}

	MT::IOraw = true;
	if (nMode  == 1){
		SampleTrajectory(robot);
		return 0;
	}
	if (nMode  == 11){
		GradTrajectory(robot);
		RobotProcessGroup robot2;
		cout << endl << Validate(robot,robot2,c,nMode,arr(1,1)) << endl;
		return 0;
	}
	if (nMode  == 3){
		robot.ctrl.threadStop();//strange hack, why is it still running...
		//robot.gui.threadLoop();
		PlanTrajectory(&robot.ctrl.ors,robot,(GraspObject*)task.graspobj);
		return 0;
	}
	ofstream f("joints.txt");
	uint nSteps = 400;
	if(nMode == 2 || nMode == 5 || nMode == 0)nSteps = MT::getParameter<int>("T");
	arr joints(nSteps,robot.ctrl.ors.getJointStateDimension());
	arr qOrig;robot.ctrl.ors.getJointState(qOrig);
	for(uint i = 0;i < nSteps && !robot.signalStop;i++){ //catches the ^C key
		if(nMode == 0) get_skin_state(task,robot);	// skin -> graspISFtask
		if(robot.openGui)robot.gui.gl->text.clear()<< i << endl;
		robot.step();
		arr q;robot.ctrl.ors.getJointState(q);
		joints[i]=q;
		cout << " step " << i << " taskcost " << robot.ctrl.sys.taskCost(NULL,0,-1);
		if(get_gamepad_state(robot)==16 || get_gamepad_state(robot)==32) break;
	}
	f << joints << endl;

	if (true){
		robot.ctrl.threadStop();//strange hack, why is it still running...
		ofstream fx("path.txt");
		arr x = GetEndPath(joints,&robot.ctrl.ors);
		x.writeRaw(fx);fx << endl;
		plotModule.thickLines = 3;
		robot.gui.linesToDisplay.clear();
		robot.gui.linesToDisplay.append(x);
		robot.step();

		while(true){
			//
			cout << " wait " << endl;
			MT::wait(0.1);
		}
	}
	robot.ctrl.ors.reportProxies();
	if(nMode == 0 && nSteps%100 == 0){
		PrintJointsNoise(robot,joints,qOrig);
		cout << " ready noise " << endl;
	}

	if(nMode == 5 || nMode == 2 || nMode == 0){//validate Stanio costs
		cout << endl << " start validate" << endl << endl;
		RobotProcessGroup robot2;
		cout << endl << Validate(robot,robot2,c,nMode,joints) << endl;
		robot2.close();
	}
	robot.close();	//plotClear();	//task.plot_all();
	return 0;
}
//~/TUB/mlr/share/robot/10-nik-graspdata/x.exe -shape 2 -radius .08 -height .2 -center "[ .5, -.5, .4 ]"


//robot.gui.ors->getBodyByName("o1")->shapes(0)->X = o1->X;
//robot.gui.ors->getBodyByName("o1")->shapes(0)->rel = o1->rel;
//memmove(robot.gui.ors->getBodyByName("o1")->shapes(0)->size, o1->size, 4*sizeof(double));*/
//((GraspObject*)task.graspobj)->buildMesh(); ask mars, problems
//copyBodyInfos(*robot.gui.ors , robot.ctrl.ors);

