#define MT_IMPLEMENTATION

#include <signal.h>
#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/robot.h>
#include <MT/soc.h>

int nIter2 = 20;//so many iterations for backproject
int nLoc = 6;//so many local models 10
int nLoc2 = 15;//so many points in local model, was 50 before
double gamaP1 = 0.001;
int nPow = 1;
#include "../../../nikolay/code/RegressCalibrate/nikLWR.h"
#include "../../../nikolay/code/RegressCalibrate/SwedishMethods.h"

ofstream pos_file("onlinePos.dat");  

OpenGL *globalGL=NULL;
struct SimpleTrack{
	ors::Vector v;//speed
	ors::Vector p;//last position
	double t ;//last time
	int Smooth;//smooth type

	void addSample(ors::Vector pos, double time){
		cout << " timeee " << (time-t) << endl;
		v = (pos-p)/(time-t);
		p = pos;
		t = time;
	}
	ors::Vector predict(double time){
		return p + v*(time-t);
	}
	arr R1,R2,R12,xk,Pk;
	arr C,Ct,y,Phi,Phit;
	arr x,P;//x,p |tt

	void Init(){
		MT::getParameter(Smooth,"Smooth");
		R1 = arr(6,6);R1.setDiag(0.02);
		for(uint i = 0; i < 3; i++){
			R1(i,i+3) = 0.01;//smaller value for better pos
			R1(i+3,i) = 0.01;//cov between speed and pos
		}
		R2 = arr(4,4);R2.setDiag(0.6);
		arr cov;
		cov <<FILE("COV");cov = cov*1.0;//created from measured data
		//R1 = 5.0*cov.sub(0,5,0,5);
		//R2 = 2.0*cov.sub(6,9,6,9);
		R12 = arr(6,4);R12.setZero();
		// R12 = 2.5*cov.sub(0,5,6,9);//R12(0,0)= 0.1;R12(2,1)= 0.1;R12(0,2)= 0.1;R12(2,3)= 0.1;


		Phi = arr(6,6);Phi.setDiag(1.0);//setdiag has setzseo inside
		Pk = arr(6,6); Pk.setDiag(.6);//initial covariance
		xk = arr(6);xk.setZero();xk(0) = 0.2; xk(1) = -0.5; xk(2) = 1;
		t = 0;
	}

	void PrepareObservation(const arr& vision, const arr & Pl, const arr & Pr, double time){
		for(uint i = 0; i < 3; i++)
			Phi(i,i+3)=time-t; //double t = 0.26;//find real time interval, .26 or .14, or 0
		transpose(Phit,Phi);
		t = time;

		C = arr(4,6);C.setZero();
		arr PLt;transpose(PLt,Pl);
		arr PRt;transpose(PRt,Pr);

		arr l(2,3);l.setZero();
		l(0,0) = -1;l(1,1) = -1;
		l(0,2) = vision(0);l(1,2) = vision(1);//left pixels
		arr lt;
		transpose(lt,l);
		arr piL = PLt*lt;    if(vision(0)+vision(1) == 0)         {piL = 0.0;cout << "left 0" << endl; }   //more robust hopefully
		l(0,2) = vision(2);l(1,2) = vision(3);  //right pixels
		transpose(lt,l);
		arr piR = PRt*lt;if(vision(2)+vision(3) == 0)         {piR = 0.0;cout << "right 0" << endl; }
		arr pi(4,4);
		for(uint i = 0; i < 4; i++){
			pi(i,0) = piL(i,0);
			pi(i,1) = piL(i,1);
			pi(i,2) = piR(i,0);
			pi(i,3) = piR(i,1);
		}
		for(uint i = 0; i < 4; i++)
			for(uint j = 0; j < 3; j++)
				C(i,j) = pi(j,i);//careful transposes...
		transpose(Ct,C);
		y = pi.sub(3,3,0,3);y.resize(4);
		y = y*-1.0;
	}

	void addSample(const arr& vision, const arr & Pl, const arr & Pr, double time ){
		PrepareObservation(vision,Pl,Pr,time);

		arr t1 = inverse(R2 + C*Pk*Ct);
		arr K = (Phi*Pk*Ct+R12)*t1;
		arr Kf = Pk*Ct*t1;

		arr oldx = x;
		arr oldP = P;
		arr oldPk = Pk;
		arr oldxk = xk;

		x = xk + Kf*(y-C*xk);
		P = Pk - Kf*C*Pk;//transpose or not ??//arr Pkt; transpose(Pkt,Pk);

		//arr t2;transpose(t2,Phi*Pk*Ct+R12);
		//Pk= Phi*Pk*Phit + R1 - K*t2;error in ppaper equations, no convergence...
		//xk = Phi*xk + K*(y-C*xk);
		xk = Phi*x;//wikipedia notation
		Pk = Phi*P*Phit + R1 ;

		if(oldx.N > 0 && Smooth == 1){ //smooth if data exists, it improves std on positions...
			arr Pi= inverse(oldPk);
			arr Pstar = oldP*Phit*Pi;//cout << "#" << Pstar << "#" << endl;
			x = oldx + Pstar*(x - Phi*oldx);
		}
		if (oldx.N > 0 && Smooth == 2){//RTS filter
			arr Pi= inverse(oldPk);
			arr Phii = inverse(Phi);
			arr Kn = Phii*R1*Pi;
			arr id(6,6);id.setDiag(1.0);
			arr F = Phii*(id - R1*Pi);
			x = F*x + Kn*oldxk;
		}
		if (oldx.N > 0 && Smooth == 3)
			x = 0.5*(oldx +x);//simple average smoother..

		p = ors::Vector(x(0),x(1),x(2));
		v = ors::Vector(x(3),x(4),x(5));
		x = oldx;//to avoid smoothing self effect..
	}
};

struct ReceedingHorizon:public StepThread{
	soc::AICO aico;
	soc::SocSystem_Ors *sys;
	TaskAbstraction *task;
	const soc::SocSystem_Ors *sys_parent;

	//OUTPUT
	arr bwdMsg_v,bwdMsg_Vinv;
	uint bwdMsg_count;

	//INPUT
	arr q0,v0;

	void init(const soc::SocSystem_Ors& _sys,TaskAbstraction *_task){
		sys_parent = &_sys;
		task=_task;
	}

	void open(){
		sys=sys_parent->newClone(true);
		sys->os = &cout;
		aico.init(*sys,.7,.0,.0,10,0,NULL);
		bwdMsg_count=0;
	}

	void step(){
		if(bwdMsg_count>10){
			sys->s->q0=q0;
			sys->s->v0=v0;
			aico.shiftMessages(-bwdMsg_count); //shift everything 10 steps forward...
			shiftTargets(sys->vars,-bwdMsg_count);
			bwdMsg_count-=bwdMsg_count; //grep messages starting from 1 again...
		}else
		{
			sys->s->q0=q0;
			sys->s->v0=v0;
		}
		aico.stepDynamic();
		bwdMsg_v=aico.v;
		bwdMsg_Vinv=aico.Vinv;
	}

	void close(){
	}

	void setReachGoals(const char* objShape){
		uint T=384;
		sys->setTimeInterval(4.,T);
		sys->setq0AsCurrent();
		aico.initMessages();
		activateAll(sys->vars,false);
		//activate collision testing with target shape - no collisions for me
		ors::Body *obj = sys->ors->getBodyByName(objShape);
		/*obj->cont=true;  sys->swift->initActivations(*sys->ors);*/
		TaskVariable *V;
		double midPrec,endPrec;
		MT::getParameter(midPrec,"reachPlanMidPrec");
		MT::getParameter(endPrec,"reachPlanEndPrec");
		arr xtarget;
		xtarget.setCarray(obj->X.p.v,3);
		V=listGetByName(sys->vars,"posNew");
		// V->updateState();
		V->y_target = xtarget;
		V->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);

		V=listGetByName(sys->vars,"collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T,MT::getParameter<double>("reachPlanColPrec"),0.);
		V=listGetByName(sys->vars,"limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T,MT::getParameter<double>("reachPlanLimPrec"),0.);
		V=listGetByName(sys->vars,"qlinear");    V->v=0.;  V->v_target=0.;  V->setInterpolatedTargetsEndPrecisions(T,0.,0.,MT::getParameter<double>("reachPlanEndVelPrec"),MT::getParameter<double>("reachPlanEndVelPrec"));
	}

};

struct RHV:public TaskAbstraction {
	uint visStep;
	TaskVariable * TV_fNew;
	ors::Body * obst,*obstV,*future,*target;
	SimpleTrack track, Kal1, Kal2;
	bool started_track, bGoTarget;
	arr Pl,Pr;
	void findObstacle(ControllerModule *ctrl);
	virtual void updateTaskVariables(ControllerModule *ctrl); //overloading the virtual
	virtual void initTaskVariables(ControllerModule *ctrl);
	void init(RobotProcessGroup *robot);
	ors::Vector CameraLocation(const arr & p);
	ReceedingHorizon recho;
};

//from camera parameters to camera center, cf. Zisserman
ors::Vector RHV::CameraLocation(const arr & p){
	double X,Y,Z,T;
	arr a(3,3);

	a = p.sub(0,2,1,3);
	X =  determinant(a);
	a = p.sub(0,2,0,2);
	T = -determinant(a);

	for (uint i = 0; i < 3; i++)
		for (uint j = 0; j < 3; j++)if(j == 0)
			a(i,j) = p(i,j);
		else
			a(i,j)= p(i,j+1);
	Y = -determinant(a);

	for (uint i = 0; i < 3; i++)
		for (uint j = 0; j < 3; j++)if(j == 2)
			a(i,j) = p(i,j+1);
		else
			a(i,j)= p(i,j);
	Z = determinant(a);
	ors::Vector ans(X/T,Y/T,Z/T);
	return ans;
}

void RHV::init(RobotProcessGroup *robot){
	cout << "init TV_q = "<<TV_q->y << endl;
	cout << "init TV_x->x="<<TV_eff->y << endl;
	MT::IOraw = true;
	started_track = false;

	arr p2;
	p2 <<FILE("regparams");
	Pl = p2.sub(0,2,0,3);
	Pr = p2.sub(3,5,0,3);

	robot->gui.gl->camera.setPosition(-0.5,-7,1);
	robot->gui.gl->camera.focus(0., -0.5, 1.);

	//find camera location
	ors::Body * b = new ors::Body(robot->gui.ors->bodies);
	ors::Shape * s=new ors::Shape(robot->gui.ors->shapes,b);
	s->type=1;
	s->size[0]=.0; s->size[1]=.0; s->size[2]=0.; s->size[3]=.02;
	s->color[0]=.5; s->color[1]=.2; s->color[2]=.8;
	b->X.p = CameraLocation(Pl);

	ors::Body * br = new ors::Body(robot->gui.ors->bodies);
	ors::Shape * sr=new ors::Shape(robot->gui.ors->shapes,br);
	sr->type=1;
	sr->size[0]=.0; sr->size[1]=.0; sr->size[2]=0.; sr->size[3]=.02;
	sr->color[0]=.5; sr->color[1]=.7; sr->color[2]=.8;
	br->X.p = CameraLocation(Pr);cout << "camera " << b->X.p << " " << br->X.p << endl;

	arr Rot;
	Rot <<FILE("RotationMatrix");arr r2; transpose(r2,Rot);Rot = r2;
	Rot = Rot*-1.0;//hack to get positive trace and determinant 1
	ors::Quaternion q;
	q.setMatrix(Rot.p);
	cout <<"Rot = " <<Rot <<"Q=" <<q <<endl;
	ors::Vector z; ors::Quaternion q2;
	q.getZ(z);q2.setRad(PI,z);
	q = q2*q;
	/*robot->gui.gl->camera.X->r = q;
	robot->gui.gl->camera.X->p = b->X.p;
	robot->gui.gl->camera.setHeightAngle(50);
	robot->gui.gl->camera.setZRange(.05,10.);*/
	robot->gui.ors->getBodyByName("camera")->X.p = (b->X.p+br->X.p)/2;
	robot->gui.ors->getBodyByName("camera")->X.r = q;

	Kal1.Init();
	Kal2.Init();
}

void RHV::initTaskVariables(ControllerModule *ctrl){
	ors::KinematicWorld &ors=ctrl->ors;
	TV_fNew   = new TaskVariable("posNew",ors,posTVT,"m9","<t( .02   .022 -.366)>",0,0,0);
	TV_fNew->targetType=directTT;
	TVall.append(TV_fNew);
	TaskAbstraction::initTaskVariables(ctrl);
	TV_col->params=ARR(.15); //change the margin for the collision variable
}

void RHV::findObstacle(RobotProcessGroup *robot){
	double time = MT::realTime();
	if(visStep != robot->evis.steps && robot->evis.hsvCenters.N){//should manuallz remove 0 observations vision.min = 0
		visStep = robot->evis.steps;
		arr vision1(4),vision2(4);
		robot->evis.hsvCenters = robot->evis.hsvCenters*(float)pow(2.0,robot->evis.downScale);
		for(uint i = 0; i < 4; i++){
			vision1(i) = 	robot->evis.hsvCenters(i);
			vision2(i) = 	robot->evis.hsvCenters(i+4);
		}
		arr val1a =   Find3dPoint(Pl,Pr,vision1);ors::Vector val1(val1a(0),val1a(1),val1a(2));
		arr val2a =   Find3dPoint(Pl,Pr,vision2);ors::Vector val2(val2a(0),val2a(1),val2a(2));

		//track.addSample(val1,time);
		Kal1.addSample(vision1,Pl,Pr,time);
		Kal2.addSample(vision2,Pl,Pr,time);

		//val1 = (Kal1.p+val1)*0.5;val2 = (Kal2.p+val2)*0.5;
		val1 = Kal1.p;val2 = Kal2.p;
		pos_file << val1 << " " << val2 << " " << Kal1.v << " " << Kal2.v << " " << vision1 << " " << vision2 << endl;
		//pos_file << val1 << " " << Kal1.p << " " << track.v << " " << Kal1.v << endl;

		ors::Vector oriVal = val2-val1;oriVal = oriVal/oriVal.length();
		ors::Vector v = val2 + oriVal*0.2;//green marker 0.2 from center
		obst->X.p =  v;
		ors::Frame f;
		f.r.setDiff(ors::Vector(0,0,1),oriVal);
		obst->X.r = f.r;
		obstV->X =  obst->X;//is there a better way = 2 ors stucts for vision and collision...
		future->X = obst->X;
		future->X.p += 0.3*(Kal1.v+Kal2.v)/2;//prdict 0.3 second ahead
	}	
}


void RHV::updateTaskVariables(ControllerModule *ctrl){
	activateAll(TVall,false); //deactivate all variables
	ctrl->useBwdMsg=false;
	TV_lim->active=true;
	TV_lim->y_prec = 1e1*0.5;
	TV_col->active=true;
	TV_col->y_prec=1e1;//are precisions here necessary??
	TV_col->y_target = 0;

	TV_q->active = true;
	TV_q->y_prec = 0.;
	TV_fNew->active = true;
	TV_fNew->y_prec = 0.;

	if (!started_track && maxDiff(ctrl->q_home, TV_q->y)<0.001) {
		cout<<"Starting track."<<endl;
		started_track = true;
	}
	if (started_track) {
		findObstacle(ctrl);
		arr x = arr(target->X.p.v,3) - TV_fNew->y;
		//ctrl->gui->gl->text.clear() << "dist to target " << norm(x) << " pos " << TV_fNew->y << " targ" << target->X.p;
		if(recho.bwdMsg_v.d1==2*ctrl->q_reference.N){
			ctrl->useBwdMsg=true;
			ctrl->bwdMsg_v   .referToSubDim(recho.bwdMsg_v,   (uint)recho.bwdMsg_count);
			ctrl->bwdMsg_Vinv.referToSubDim(recho.bwdMsg_Vinv,(uint)recho.bwdMsg_count);
			cout <<"bwdMsg#"<<recho.bwdMsg_count <<flush;
			recho.bwdMsg_count++;
		}else{
			MT_MSG("no bwd message!");
		}
	}
	else {
		cout<<"Starting home "<< maxDiff(ctrl->q_home, TV_q->y)<< endl;
		TV_q->v_prec = 1.;
		TV_q->v_target = ctrl->q_home - TV_q->y;
		double vmax = .3, v=norm(TV_q->v_target);
		if (v>vmax) TV_q->v_target *= vmax/v;
	}
	recho.q0=ctrl->q_reference;
	recho.v0=ctrl->v_reference;
}



int main(int argc,char** argv){
	MT::IOraw = true;
	MT::initCmdLine(argc,argv);
	signal(SIGINT,RobotProcessGroup::signalStopCallback);
	RobotProcessGroup robot;
	RHV demo;
	demo.recho.init(robot.ctrl.sys,&demo);
	robot.ctrl.task=&demo;

	robot.evis.downScale = 1;//2 times smaller resolution
	robot.open();
	globalGL = robot.gui.gl;

	robot.gui.ors->getBodyByName("OBJECTS")->X.p(0) = 100;
	demo.obst = robot.ctrl.ors.getBodyByName("obstacle");
	demo.obstV = robot.gui.ors->getBodyByName("obstacle");
	demo.future = robot.gui.ors->getBodyByName("obstacleF");
	demo.init(&robot);

	arr atarget; MT::getParameter(atarget,"target");
	demo.target = robot.gui.ors->getBodyByName("target");
	demo.target->X.p =  ors::Vector(atarget(0),atarget(1),atarget(2));
	robot.ctrl.ors.getBodyByName("target")->X.p = demo.target->X.p ;//planenr thread uses this actually, not GL body !!!

	demo.recho.threadOpen();
	demo.recho.setReachGoals("target");

	for(;!robot.signalStop;){ //catches the ^C key
		robot.step();
		demo.recho.threadStepOrSkip(100);//why, what is the meaning....
		if(robot.gamepad.state(0)==16 || robot.gamepad.state(0)==32) break;
	}
	robot.close();
	return 0;
}


