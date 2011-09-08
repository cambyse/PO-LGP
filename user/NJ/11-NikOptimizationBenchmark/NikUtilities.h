/*
 * NikUtilities.h
 *
 *  Created on: Feb 22, 2011
 *      Author: nikolay
 */

#ifndef NIKUTILITIES_H_
#define NIKUTILITIES_H_


#endif /* NIKUTILITIES_H_ */

ofstream costfile("costP.dat");
arr Clusters;
int T;
soc::SocSystem_Ors sys;
OpenGL* gl;
double midPrec, endPrec, limPrec, colPrec, protoPrec;

arr inerpolateDatabaseMotion(int nInit, uint time)  {
	double nDATAT = 400;
	arr clust = Clusters.sub((nInit) * 400, (nInit) * 400 + 399, 0, 6);
	if(time == nDATAT)
		return clust;

	arr a(time, clust.d1);
	double fratio = nDATAT / (time - 1);
	for(uint i = 0;i < time - 1;i++){
		int l = floor((i) * fratio);
		int h = l + 1;
		a[i] = 0.5*clust[l] + 0.5*clust[h];  //clust[l] + (h - i * fratio) * (clust[h] - clust[l]);
	}
	a[time - 1] = clust[nDATAT - 1];
	return a;
}

void setTargetAndQ(const arr & qhome,const ors::Vector target){
	sys.ors->setJointState(qhome);
	sys.ors->calcBodyFramesFromJoints();
	ors::Body *obj = sys.ors->getBodyByName("target");
	obj->X.pos = target;
}

void setKReachGoals(const arr & clust = NULL)
{
	sys.setq0AsCurrent();
	arr q0 = sys.WS->q0;
	activateAll(sys.vars, false);
	TaskVariable *V;
	arr xtarget;
	xtarget.setCarray(sys.ors->getBodyByName("target")->X.pos.p, 3);
	V = listGetByName(sys.vars, "posNew");  V->updateState();
	V->y_target = xtarget;
	V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
	V = listGetByName(sys.vars, "limits");
	V->y = 0.;
	V->y_target = 0.;
	V->setInterpolatedTargetsConstPrecisions(T, limPrec, 0.);
	V = listGetByName(sys.vars, "collision");
	V->y = 0.;
	V->y_target = 0.;
	V->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
	V->y_prec_trajectory(0) = 0;V->y_prec_trajectory(1) = 0;V->y_prec_trajectory(2) = 0;V->y_prec_trajectory(3) = 0;
	if(clust.N)// if(true) !bInited
	{
		V = listGetByName(sys.vars, "qitself");
		V->setInterpolatedTargetsConstPrecisions(T, protoPrec, 0.); //tricky method,  destroys my targets !!
		V->y_trajectory = clust;
		double Tratio = 0.1;
	if(true)	for(uint i = 0;i <= T;i++)//aico and ilqg surprisingly sensitive to continuous movements
			if(i < Tratio * T){
				V->y_prec_trajectory(i) = protoPrec * (i+0) / (Tratio * T);
			}
			else{
				if(i < (1 - Tratio) * T)
					V->y_prec_trajectory(i) = protoPrec;
				else
					V->y_prec_trajectory(i) = protoPrec * (T-i) / (Tratio * T)* (T-i) / (Tratio * T) ;//protoPrec;//
			}///NOTE: aico works better when not going to target and staying continuous, ilqg works better when final step closer to target
	//V->y_prec_trajectory(T) = 1e4;//to offset larget endeffctor values
	}
}

arr endeffTraj(const arr & q){
	arr x(q.d0,3);
	TaskVariable * V = listGetByName(sys.vars, "posNew");
	for(uint i = 0; i < q.d0; i++){
		sys.ors->setJointState(q[i]);
		sys.ors->calcBodyFramesFromJoints();
		V->updateState();
		x[i] = V->y;
	}
	sys.ors->setJointState(q[0]);
	sys.ors->calcBodyFramesFromJoints();
	return x;
}

arr IKTrajWithInit(const arr & qRef){
	arr ans;
	setKReachGoals(qRef);
	bool bDyn =  sys.dynamic;
	sys.dynamic = false;
	soc::bayesianIKTrajectory(sys,ans,-10);
	sys.dynamic = bDyn;
	return ans;
}

arr planTrajectoryWithInit(arr & qInit){
	int mode=MT::getParameter<int>("planmode");
	int iter=MT::getParameter<int>("planiter");int iter2 = iter;
	arr ans;
	soc::LQG lqg;
	lqg.init(sys, .8, 4, 0);
	lqg.q = qInit;
	/////aico stuff
	soc::AICO aico;
	aico.init(sys,.8,.0,.0,4,0);
	aico.q = qInit;
	aico.initMessagesWithReferenceQ(qInit);

	aico.damping = 0.2;
	soc::getPhaseTrajectory(aico.dampingReference,qInit,sys.getTau());
	///plan with corresponding method
	switch(mode){
	case 1:

		while(iter-- > 0){
			lqg.stepGeneral();
			plotClear();
			plotLine(endeffTraj(lqg.q));
			costfile << iter2 - iter << " " << lqg.cost << endl;
		}
		ans = lqg.q;
		break;
	case 2:
		while(iter-- > 0){
			aico.stepDynamic();
			plotClear();
			plotLine(endeffTraj(aico.q));
			costfile << iter2 - iter << " " << aico.cost << endl;
		}
		ans = aico.q;
		break;
	case 3:
		while(iter-- > 0){
			aico.stepGaussNewton();
			plotClear();
			plotLine(endeffTraj(aico.q));
			costfile << iter2 - iter << " " << aico.cost << endl;
		}
		ans = aico.q;
		break;
	default: NIY;
	}

	return ans;
}

arr planTrajectoryWithCondition(arr & qInit){
	int mode=MT::getParameter<int>("planmode");
	int iter=MT::getParameter<int>("planiter");int iter2 = iter;
	arr ans;
	soc::LQG lqg;
	lqg.init(sys, .8, 4, 0);
	/////aico stuff
	soc::AICO aico;
	aico.init(sys,.8,.0,.0,4,0);
	aico.damping = 0.5;
	double fConditionDamp = MT::getParameter<double>("conditiondamp");//how to decrease condition variable inportrance
	///plan with corresponding method
	switch(mode){
	case 1:

		while(iter-- > 0){
			lqg.stepGeneral();
			plotClear();
			plotLine(endeffTraj(lqg.q));
			costfile << iter2 - iter << " " << lqg.cost << endl;
			TaskVariable * V = listGetByName(sys.vars, "qitself");//use in some interface....damping does not work...
			V->y_prec_trajectory*= fConditionDamp;//decrease importance
		}
		ans = lqg.q;
		break;
	case 2:
		while(iter-- > 0){
			aico.stepDynamic();
			plotClear();
			plotLine(endeffTraj(aico.q));
			costfile << iter2 - iter << " " << aico.cost << endl;
			TaskVariable * V = listGetByName(sys.vars, "qitself");
			V->y_prec_trajectory *= fConditionDamp;//decrease importance
		}
		ans = aico.q;
		break;
	case 3:
		while(iter-- > 0){
			aico.stepGaussNewton();
			plotClear();
			plotLine(endeffTraj(aico.q));
			costfile << iter2 - iter << " " << aico.cost << endl;
			TaskVariable * V = listGetByName(sys.vars, "qitself");
			V->y_prec_trajectory *= fConditionDamp;//decrease importance
		}
		ans = aico.q;
		break;
	default: NIY;
	}

	return ans;
}

void init(){
	gl = new OpenGL("robot",800,600);
	//gl->setViewPort(0,.0,1,0,1.);
	//	gl->views(0).camera.setPosition(-0.1,-1.2,7.);
	//	gl->views(0).camera.focus(-0.1, -0.6, 0);
	//	gl->addView(0,glStandardLight,NULL);
	//	gl->addView(0,glDrawPlot,&plotModule);
	gl->add(glStandardLight,NULL);
	gl->add(glDrawPlot,&plotModule);
	gl->camera.setPosition(-0.1,-1.2,7.);
	gl->camera.focus(-0.1, -1.2, 1);
	gl->update();
	MT::load(Clusters, "qD.dat");
	T = MT::getParameter<int>("Tplan");
	double timeReal= MT::getParameter<int>("timereal");
	sys.initBasics(NULL,NULL,gl,T,timeReal,true,NULL);
	sys.os = &cout;

	//now add all control variables
	TaskVariable * TV_fNew   = new TaskVariable("posNew",*sys.ors,posTVT,"m9","<t( -.0000031   .000002 -.357)>",0,0,0);
	TV_fNew->targetType=directTT;
	//now add standard stuff - qlimit, collision, qitself
	double margin; MT::getParameter(margin,"swiftMargin");
	TaskVariable *  TV_col  = new TaskVariable("collision", *sys.ors, collTVT,0,0,0,0,ARR(margin));
	arr limits;
	limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
			-1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
	TaskVariable * TV_lim = new TaskVariable("limits", *sys.ors, qLimitsTVT,0,0,0,0,limits);
	TaskVariable * TV_q    = new TaskVariable("qitself", *sys.ors, qItselfTVT,0,0,0,0,0);
	sys.setTaskVariables(TUPLE(TV_fNew, TV_col,TV_lim,TV_q));

	MT::getParameter(midPrec, "reachPlanMidPrec");
	MT::getParameter(endPrec, "reachPlanEndPrec"); //load here to avoid tthread problems with CFG file
	MT::getParameter(limPrec, "reachPlanLimPrec");
	MT::getParameter(colPrec, "reachPlanColPrec");
	MT::getParameter(protoPrec, "reachPlanProtoPrec");

}
