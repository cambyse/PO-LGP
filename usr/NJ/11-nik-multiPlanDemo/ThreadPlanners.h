//for debugging
ofstream file("costK.dat");
ofstream costfile("costP.dat");
ofstream qfile("q.dat");
ofstream xfile("x.dat");
ofstream pfile("plot.dat");
ofstream tfile("target.dat");
bool bClustSet = true;//which set of prototypes
bool bDATAMODE = false;//1 is data gathering, 0 is use of the prototypes
//#define NIKAICO //inviolates shift operations
//#include <MT/specialTaskVariables.h>
#include <MT/threads.h>



real totalControlCost2(const arr& q,soc::SocSystemAbstraction * sys){
	uint t,T=sys->nTime(),i,m=sys->nTasks();
	CHECK(q.nd==2 && q.d0==T+1 && q.d1==sys->qDim(),"");
	arr W,H,M,F;
	real tau=sys->getTau();
	real tau_1 = 1./tau, tau_2 = tau_1*tau_1;
	arr phi_qhat,x,v,Jqd,u;
	real taskCsum=0., ctrlCsum=0.;
	arr ctrlC(T+1);  ctrlC.setZero();
	for(t=0;t<=T;t++){
		if(!sys->dynamic || !t) sys->setq(q[t]);
		else sys->setqv(q[t],tau_1*(q[t]-q[t-1]));

		if(!sys->dynamic){
			sys->getW(W,t);
			if(t>0) ctrlC(t) = sqrDistance(W,q[t-1],q[t]);
		}else{
			sys->getH(H,t);
			sys->getMF(M,F,t);
			if(t<T && t>0)
				ctrlC(t) = sqrDistance(H,tau_2*M*(q[t+1]+q[t-1]-(real)2.*q[t]),F);
			if(t==0)
				ctrlC(t) = sqrDistance(H,tau_2*M*(q[t+1]-q[t]),F);
		}
		ctrlCsum += ctrlC(t);
	}
	double max = 0;int maxt = -1;
	for(t = 1; t < T; t++)
		if(ctrlC(t) > max){
			max = ctrlC(t);
			maxt = t;
		}
	costfile << " sinners " << maxt << " " << max << endl << q[maxt-1] << endl << q[maxt] << endl << q[maxt+1] << endl;
	//  costfile << " ctrlC array " << ctrlC << endl << " total " << ctrlCsum << endl; //   HALT(" test finished");
	return ctrlCsum;
}

real totalControlCost(const arr& q,soc::SocSystemAbstraction * sys){
	uint t,T=sys->nTime();
	CHECK(q.nd==2 && q.d0==T+1 && q.d1==sys->qDim(),"q has wrong dimension: "<<q.getDim());
	arr W,H,M,Mt,F;
	real tau=sys->getTau();
	real tau_1 = 1./tau, tau_2 = tau_1*tau_1;
	real ctrlCsum=0.;
	arr ctrlC(T+1);  ctrlC.setZero();
	for(t=0;t<=T;t++){
		countSetq++;
		if(!sys->dynamic || !t) sys->setq(q[t]);
		else sys->setqv(q[t],tau_1*(q[t]-q[t-1]));
		if(!sys->dynamic){
			sys->getW(W,t);
			if(t>0) ctrlC(t) = sqrDistance(W,q[t-1],q[t]);
		}else{
			sys->getH(H,t);
			sys->getMF(M,F,t);
			if(t<T && t>0)
				ctrlC(t) = sqrDistance(H,tau_2*M*(q[t+1]+q[t-1]-(real)2.*q[t]),F);
			if(t==0)
				ctrlC(t) = sqrDistance(H,tau_2*M*(q[t+1]-q[t]),F);
		}
		ctrlCsum += ctrlC(t);
	}
	return ctrlCsum;
}

ors::Vector NextRandom(ors::Vector posObst,ors::Vector last){
	ors::Vector a,a0;
	ors::Vector pos0 = posObst;pos0(2) = 0;//care only about radius of obstacle
	bool bGood = false;
	while(!bGood){
		a = ors::Vector((rnd.uni()-0.5)*0.7,rnd.uni()*-0.25 - 0.6,rnd.uni()*0.3 + 0.85);
		a0 = a; a0(2) = 0;
		if ((a0 - pos0).length() > 0.2 && (last - a).length() > 0.1)
			bGood = true;
	}
	return a;
}

//find such speed such to avoid harming robot motor
double FindSpeed(const arr & q){
	q.writeRaw(qfile);qfile<< endl;
	double a = 0;
	for(uint i = 0; i < q.d0-1; i++){
		//arr tmp = q[i]-q[i+1];
		double d = maxDiff(q[i],q[i+1]);
		if (fabs(d) > a)
			a = d;
	}
	a = 1.4*a/0.01;//how much do we exceed 0.01 limit of joint change
	if(a< 2.5)
		a = 2.5;
	return a;
}

//should have a large time resolution, find empirically exact values
struct SecKinPlanner:public StepThread{
#ifdef NIKAICO
	soc::AICO aico;
#else
	soc::LQG aico;
#endif
	soc::SocSystem_Ors *sys;
	TaskAbstraction *task;
	const soc::SocSystem_Ors *sys_parent;
	uint T; //tiem steps
	char *target; //which shape is goal
	uint nMod; //grasp or rreach
	bool showTrajectory;
	bool bInited;
	int nInit; //type of initial path
	uint nBest;//know which is best from above thread
	arr Clusters, xdesc;
	double midPrec, endPrec, limPrec, colPrec, protoPrec;
	bool bReady; //what is proper way to do this with threads??
	//OUTPUT
	//arr bwdMsg_v, bwdMsg_Vinv;
	arr QDisplayGL2,XDisplayGL2;
	double costHack,controlcost;
	//INPUT from dyn planner/hardware
	arr q0, v0;
	ors::Vector initTarget;
	uint bwdMsg_count;
	SecKinPlanner(char *name, uint _T)
	:StepThread(name)
	{
		T = _T;
	}

	;
	void init(const soc::SocSystem_Ors & _sys, TaskAbstraction *_task, char *name, int nmod, uint _nInit = 0)  {
		sys_parent = &_sys;
		task = _task;
		bInited = false;
		nInit = _nInit;
		if(bDATAMODE || !bClustSet )
			Clusters <<FILE("ClustQ4.txt");
		else{
			Clusters <<FILE("qD.dat");
			xdesc <<FILE("xD.dat");
		}
		MT::getParameter(midPrec, "reachPlanMidPrec");
		MT::getParameter(endPrec, "reachPlanEndPrec"); //load here to avoid tthread problems with CFG file
		MT::getParameter(limPrec, "reachPlanLimPrec");
		MT::getParameter(colPrec, "reachPlanColPrec");
		MT::getParameter(protoPrec, "reachPlanProtoPrec");
		nMod = nmod;
		target = name;
		showTrajectory = true;
		bwdMsg_count = 0;
	}

	void open()
	{
		sys = sys_parent->newClone(true);
		sys->setTimeInterval(8, T);
		sys->dynamic = true;
		sys->os = &cout;
#ifdef NIKAICO
		aico.init(*sys,.8,.0,.0,10,0);
		aico.damping = 0.1;
#else
		if(nInit > 0)
			aico.init(*sys, .8, 0, 0);//the 3rd param is display
		else
			aico.init(*sys, .8, 0, 0);
#endif
	}

	arr EndEffTraj(const arr & q){
		arr x(q.d0,3);
		TaskVariable * V = listGetByName(sys->vars, "posNew");
		for(uint i = 0; i < q.d0; i++){
			// cout << i << endl;
			sys->ors->setJointState(q[i]);
			sys->ors->calcBodyFramesFromJoints();
			V->updateState();
			// ors::Vector temp = EndEffPos(sys->ors,q[i]);
			// x(i,0) = temp(0);    x(i,1) = temp(1);    x(i,2) = temp(2);
			x[i] = V->y;
		}
		sys->ors->setJointState(q[0]);
		sys->ors->calcBodyFramesFromJoints();
		//cout << " endx " << x[100] << endl;
		//cout << " endq " << q[100] << endl;
		// q.writeRaw(pfile);pfile << endl;
		return x;
	}

	ors::Vector EndEffPos(ors::KinematicWorld *C, const arr & q)
	{
		C->setJointState(q);
		C->calcBodyFramesFromJoints();
		ors::Transformation f = C->getBodyByName("m9")->X;
		f.appendTransformation(ors::Transformation().setText("<t( .0   .0 -.357)>"));
		return f.pos;
	}

	void LameShift(uint nStep)
	{
		// file << " shift " << nStep << endl << aico.q << endl;
		for(uint i = 0;i < aico.q.d0 - nStep;i++)
			aico.q[i] = aico.q[i + nStep]; //copy forward
		for(uint i = aico.q.d0 - nStep ;i < aico.q.d0;i++)
			aico.q[i] = aico.q[aico.q.d0-1]; //last set to constant value
		//    file << " const " << aico.q.d0 - nStep << " " << aico.q[aico.q.d0 - 1] << endl;
		SetGoals(false); //some early values were 0, now will have precisions again after shift
		//  file << " after " << endl << aico.q << endl;
	}

	void SetQ0Stuff()  {
		file << " q0 stuff " << q0 << endl;
		sys->s->q0 = q0;
		sys->s->v0 = v0; //here or in everz case ??
		sys->setqv(q0,v0); //only in conjunction with set targets ??, otherwise s->q is enough, aico reads it
		sys->ors->setJointState(q0,v0);
		sys->ors->calcBodyFramesFromJoints();
	}

	void MakeConsistent()
	{
		if(bwdMsg_count < 5)
			return;
		bool bR = !bDATAMODE; //data gathering or real demo
		if(bR )
			SetQ0Stuff();
		SetGoals(false);
		if(bR){
			TaskVariable * V = listGetByName(sys->vars, "qitself");//set current as potenmtial
			V->y_target = q0;
			V->setInterpolatedTargetsConstPrecisions(T, 1e7, 0.); //tricky method,  is it nullified by monotonicity
			for(uint i = 0; i < V->y_prec_trajectory.d0; i++){
				V->y_trajectory[i] = q0;
				if(i != bwdMsg_count )
					V->y_prec_trajectory(i) = 0;
			}
		}

		if(bR )
			for(uint j = 0;j < sys->vars.N;j++)
				if(sys->vars(j)->y_prec_trajectory.N)
					for(uint i = 0;i <= bwdMsg_count;i++)
						sys->vars(j)->y_prec_trajectory(i) = 0;
		return; //problem: changing this forces abrupt acceleration at change point
		/*if(bR )
			if(aico.q.N)
				for(uint i = 0;i < bwdMsg_count;i++)
					aico.q[i] = 1.0*qset; //now we are in this point! q0 should be  aico.q[bwdMsg_count]
		 */
		// for(uint j = 0; j <= 3; j+=3)
		//  if(sys->vars(j)->y_prec_trajectory.N)
		//   file << " var " << j << "prec: " << sys->vars(j)->y_prec_trajectory(bwdMsg_count) << " val " << sys->vars(j)->y
		//  << " targ  " << sys->vars(j)->y_trajectory[bwdMsg_count]  <<  "count " << bwdMsg_count <<endl;
	}

	void step()
	{
		bReady = false;
		if(bInited == false){
			SetQ0Stuff();
		}
		if(bInited){
			if(nInit == nBest){//dispcounter == nInit
				aico.display = 10;
				file << " printing " << nInit << " " << nBest << endl;
			}
			else
				aico.display = 0;

			//	file << " q bef step " << aico.q.sub(bwdMsg_count+1,bwdMsg_count+1,0,0) << " ";
			MakeConsistent();
#ifdef NIKAICO
			aico.stepDynamic();// aico.stepGaussNewton();
#else
			if(sys->dynamic)
				aico.stepGeneral();
			else
				aico.stepKinematic();
#endif
			//	double ans = sys->analyzeTrajectory(aico.q,false);
			//		double ans2 = sys->analyzeTrajectory(aico.q,false);
			controlcost = totalControlCost(aico.q,sys);
			//	double controlcost2 = totalControlCost2(aico.q,sys);
			//aico.cost = sys->totalCost(NULL, aico.q, false);
			//if(ans > aico.cost && controlcost > aico.cost){
			//	costfile << " analyze " << ans << " " << ans2 << " orig " << aico.cost << " control " << controlcost << " " << controlcost2  << endl;
			//	HALT(" bad costs");
			//}
			XDisplayGL2 = EndEffTraj(aico.q);
			QDisplayGL2 = aico.q;
			double L = 0,L0,Q0;
			for(uint i = 0; i < XDisplayGL2.d0-1;i++)
				L += norm(XDisplayGL2[i] - XDisplayGL2[i+1]);
			L0 = norm(XDisplayGL2[0] - XDisplayGL2[XDisplayGL2.d0-1]);//final displacement
			Q0 = norm(QDisplayGL2[0] - QDisplayGL2[XDisplayGL2.d0-1]);
			file << " L NORM " << L << " " << L0 << " " << Q0 << " " << (0.2*Q0 +L*0.3 + L0*0.4) << " for " << nInit << endl;
			costHack =  controlcost*(0.2*Q0 +L*0.2 + L0*0.3);//long trajecs should not be penalized for joint length
			if(costHack > 0.3*controlcost) costHack =  0.3*controlcost;
			if(costHack > aico.cost ) costHack = 0;
		}else{
			aico.display = 0;
			aico.convergenceRate = 0.8;
			//stuff when initing
			aico.v = NULL;
			aico.Vinv = NULL;
			aico.q = NULL;
			aico.cost = 1e6;
			SetGoals(true);
			if(nInit > 0){
				//  soc::straightTaskTrajectory(*sys, aico.q, 5); //5 is q joint task
				bool bDyn =  sys->dynamic;
				sys->dynamic = false;
				soc::bayesianIKTrajectory(*sys,aico.q,-10);
				sys->dynamic = bDyn;
				QDisplayGL2 = aico.q;
			}
			else{
				soc::straightTaskTrajectory(*sys, aico.q, 0); //reach task
				QDisplayGL2 = aico.q;
				file << " first states after init" << QDisplayGL2[0] << endl << QDisplayGL2[1] << endl;
			}
			sys->ors->setJointState(q0);sys->ors->calcBodyFramesFromJoints();
			SetGoals(false);
			XDisplayGL2 = EndEffTraj(QDisplayGL2);
			bInited = true;
		}
		//aico.cost = sys->totalCost(NULL, aico.q, false);
		if (aico.cost > 10000)
			cout << aico.cost;
		file << " plan " << nInit << " bIn " << bInited << " cost " << aico.cost << endl; // " q " << aico.v[100] << endl;
		file.flush();
		bReady = true;
		initTarget = sys->ors->getShapeByName(target)->X.pos;
	}

	void close()
	{
	}

	//interpolate time steps, from bigger to smaller always
	arr ClusterV(int nInit, uint time)  {
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

	int BestCluster(int nRank)  {
		arr bestInd(nRank + 1);
		arr feature(10);
		for(uint i = 0;i < 7;i++)
			feature(i) = q0(i);
		feature(7) = sys->ors->getShapeByName(target)->X.pos(0);
		feature(8) = sys->ors->getShapeByName(target)->X.pos(1);
		feature(9) = sys->ors->getShapeByName(target)->X.pos(2);
		arr copyDesc = xdesc * 1.0;
		for(int i = 0;i < nRank + 1;i++){
			//so many min indexex
			double min = 10000;
			for(uint j = 0;j < copyDesc.d0;j++){
				arr desc =    copyDesc[j];
				double val = sumOfSqr(feature.sub(0,6) - desc.sub(0,6)) + 5.5*sumOfSqr(feature.sub(7,9) - desc.sub(7,9));
				for(int iold = 0; iold < i; iold++)//penalize being too close to previous choices
					val -= sumOfSqr(ClusterV(j,400)-ClusterV(bestInd(iold),400))/400 * 0.000003;
				if(i == 0 && nRank == 0)file << " clustval " << j << " " << val << endl;
				if(val < min){
					min = val;
					bestInd(i) = j;
				}
			}
			//if(i == 0)      bestInd(i) = 10;
			copyDesc[bestInd(i)] = copyDesc[bestInd(i)] + 10000000.0; //eliminate
		}
		file << " selected clusters " << bestInd << " for " << feature << endl;
		return bestInd(nRank);
	}

	void setKReachGoals(const char *objShape, bool bTrajPred)
	{
		sys->setq0AsCurrent();
		file << " q0 set" << endl;
		// aico.initMessages(); !!!! thi hacks my intiializations
		activateAll(sys->vars, false);
		ors::Shape *obj = sys->ors->getShapeByName(objShape);
		TaskVariable *V;
		arr xtarget;
		xtarget.setCarray(obj->X.pos.p, 3);
		file << " target " << xtarget << endl;
		V = listGetByName(sys->vars, "posNew");  V->updateState();
		V->y_target = xtarget;
		V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
		V = listGetByName(sys->vars, "limits");
		V->y = 0.;
		V->y_target = 0.;
		V->setInterpolatedTargetsConstPrecisions(T, limPrec, 0.);
		V = listGetByName(sys->vars, "collision");
		V->y = 0.;
		V->y_target = 0.;
		V->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
		V->y_prec_trajectory(0) = 0;V->y_prec_trajectory(1) = 0;V->y_prec_trajectory(2) = 0;V->y_prec_trajectory(3) = 0;
		if(nInit > 0 && bTrajPred)// if(true) !bInited
		{
			V = listGetByName(sys->vars, "qitself");
			arr currentClust;
			if(bDATAMODE || !bClustSet)
				currentClust = ClusterV((nInit - 1), T + 1);
			else
				currentClust = ClusterV(BestCluster(nInit - 1), T + 1);
			// XDisplayGL2 = EndEffTraj(currentClust);
			V->setInterpolatedTargetsConstPrecisions(T, protoPrec, 0.); //tricky method,  destroys my targets !!
			V->y_trajectory = currentClust;
			double Tratio = 0.1;
			for(uint i = 0;i <= T;i++)
				if(i < Tratio * T){
					V->y_prec_trajectory(i) = protoPrec * i / (Tratio * T);
					V->y_trajectory[i] = (V->y_trajectory[i] * (double)i + (Tratio * T - i) * q0) / (Tratio * T);
				}else{
					if(i < (1 - Tratio) * T)
						V->y_prec_trajectory(i) = protoPrec;

					else
						V->y_prec_trajectory(i) = protoPrec * (i - (1 - Tratio) * T) / (Tratio * T);
					//   V->y_trajectory[i] = (V->y_trajectory[i]*(400.0-i) + (i-350.0)*q0)/50.0;
				}

			V = listGetByName(sys->vars, "posNew");
			V->y_prec_trajectory /= 10000.0;
			V = listGetByName(sys->vars, "collision");
			V->y_prec_trajectory *= 10.0;


			//      V = listGetByName(sys->vars, "posNew");
			//    for(uint i = 0;i < T; i++)
			//    V->y_prec_trajectory(i) = 0;//no more mid precisions
		}
	}

	/*void setKGraspGoals(const char* objShape){
		sys->setTimeInterval(4.,T);
		sys->setq0AsCurrent();
		activateAll(sys->vars,false);
		setGraspGoals(*sys,T,objShape);
		target = (char*)objShape;
	}*/
	void SetGoals(bool bTrajPred)
	{
		if(nMod == 1)
			setKReachGoals(target, bTrajPred);
		//		if (nMod == 2)
		//			setKGraspGoals(target);
	}
	void UpdateExtState(ors::Body *b)
	{
		int ind = b->index;
		sys->ors->bodies(ind)->X.rot = b->X.rot; //thus give orientation as well, pos is given to external state
		sys->ors->bodies(ind)->X.pos = b->X.pos; //actually, X.pos is read in goals without ext state, so change pos as well
		arr state(T + 1, 4); //50 is number of time steps in planner, later make smarter
		for(uint i = 0;i < state.d0;i++){
			state(i, 0) = ind;
			for(uint j = 0;j < 3;j++)
				state(i, j + 1) = b->X.pos(j) + b->X.vel(j) * 4.0 * i / (double)((T + 1));//estimated position, this planner stands for 4s in t steps
		}
		sys->s->q_external = state;
	}



};

struct ThreadPlanner:public StepThread{
	//  TaskAbstraction *task;
	uint T;//time steps
	int nPlan;//how many pklanners totally
	uint nBest;//best sub planner

	MT::Array<SecKinPlanner*> helpers;
	double lastCost;//judge planners progress and whether costs worsening ,diffCost
	// bool bStop;

	//OUTPUT
	arr bwdMsg_v,bwdMsg_Vinv;
	double bwdMsg_count;

	//INPUT from simulator
	arr q0,v0;


	bool bUnsetInit;
	bool bShiftAll;
	bool bReady;

	ThreadPlanner():StepThread("dynamicplanner"){};

	//current position and velocity in m/s of body
	void UpdateExtState(ors::Body * b){
		for(uint i = 0; i < helpers.N; i++)
			helpers(i)-> UpdateExtState(b);
	}


	void UnsetInit(){
		cout << " unsetinit " << endl << endl;file  << " unsetinit " << endl << endl;costfile << endl;
		MT::IOraw = true;
		if(bDATAMODE){
			for(uint i = 0; i < helpers.N; i++){
				if(helpers(i)->aico.q.N>0 && helpers(i)->aico.cost < 0.4){ //save data
					arr xEndeff =   helpers(i)->EndEffTraj(  helpers(i)->aico.q);
					if(norm(xEndeff[0] - xEndeff[xEndeff.d0-1]) > 0.3){//only long trajecs are itneresting

						xEndeff.writeRaw(xfile);    xfile << endl;
						helpers(i)->aico.q.writeRaw(qfile);    qfile << endl;
						//tfile << helpers(i)->aico.cost << " ";
						helpers(i)->aico.q[0].writeRaw(tfile);
						tfile << " " << helpers(i)->initTarget << endl;
					}
				}
				file <<  helpers(i)->aico.cost  << " ";
			}
			file << endl;
		}
		for(uint i = 0; i < helpers.N; i++)
			helpers(i)->bInited = false;

		/*
    if(nBest != 0 && nBest < nPlan){
      helpers(0)->aico.q = helpers(nBest)->aico.q;
#ifdef NIKAICO
      helpers(0)->aico.v = helpers(nBest)->aico.v;//this stores best so far
      helpers(0)->aico.qhat = helpers(nBest)->aico.qhat;
      helpers(0)->aico.Vinv = helpers(nBest)->aico.Vinv;
#endif
    }
    if ( bwdMsg_count > 50 && bwdMsg_count < T-50 && false){//so there is sense to preserve solution
      helpers(0)->LameShift(bwdMsg_count);   //aico.shiftSolution(bwdMsg_count);
      file << " shift " << endl;
    }
    else
      helpers(0)->bInited = false;*/
		lastCost = 1000000;
		nBest =0;
		bwdMsg_v = arr(0);//helpers(nBest)->aico.v;//v or qhat ??  //bwdMsg_Vinv = helpers(nBest)->aico.Vinv;     //dfile << "bwd " << bwdMsg_v[0] << endl << " orig " <<  helpers(nBest)->aico.v[bwdMsg_count];
		bwdMsg_count = 0;
	}

	void ShiftAll(){
		if ( bwdMsg_count > T*0.75){
			int N =  T*0.35; //bwdMsg_count;
			file << " shift all" << N << " " << bwdMsg_count << endl;
			bwdMsg_count -=N;
			for(uint i = 0; i < helpers.N; i++){
				file << helpers(i)->aico.q[bwdMsg_count] << endl;
				helpers(i)->LameShift(N);   //->aico.shiftSolution(N);
				file << helpers(i)->aico.q[0] << endl << " q0 " << helpers(i)->q0;
				helpers(i)->SetQ0Stuff();
			}
		}
		bwdMsg_v = helpers(nBest)->aico.q;//crucial, otherwise bad targets will come in main thread
	}

	void init(const soc::SocSystem_Ors& _sys,TaskAbstraction *_task, char * name, int nMod, uint _T){
		T = _T;
		MT::getParameter(nPlan,"nPlan");
		for(int i = 0; i < nPlan; i++){
			MT::String name = MT::String("plan") + i;
			helpers.append(new SecKinPlanner(name,T));
		}
		for(uint i = 0; i < helpers.N; i++)
			helpers(i)->init(_sys,_task,name,nMod,i);//i-1 0 and -1 threated specially
		lastCost = 1000000;
		//diffCost = 0;
		nBest =666;
		bwdMsg_count=0;
		bUnsetInit = 0;bShiftAll = 0;
	}

	void open(){
		bReady = true;
		for(uint i = 0; i < helpers.N; i++)
			helpers(i)->threadOpen();
	}

	void step(){
		bReady = false;
		if(bUnsetInit || bShiftAll)
			for (uint i = 0; i < helpers.N; i++){
				helpers(i)->aico.convergenceRate = 0.9;
				if( bShiftAll) helpers(i)->MakeConsistent();
			}
		if(bUnsetInit){
			lastCost = 100000;
			UnsetInit();
			bUnsetInit = false;
		}
		if(bShiftAll){
			// lastCost = 100000;
			ShiftAll();
			bShiftAll = false;
		}

		for (uint i = 0; i < helpers.N; i++){
			helpers(i)->bwdMsg_count = bwdMsg_count;
			helpers(i)->threadStep();
		}

		for (uint i = 0; i < helpers.N; i++)
			helpers(i)->threadWait();

		if(bwdMsg_count < 5 || false){//dont change best once we have started, ok with new consistency
			for (uint i = 0; i < helpers.N; i++){
				if(helpers(i)->aico.cost - helpers(i)->costHack < lastCost ){//&& norm(q0-helpers(i)->aico.q[bwdMsg_count+1]) < 0.3 second condition ensures we don't jump to bad trajects, consistency should tdo this, but it is bugged...
					lastCost = helpers(i)->aico.cost - helpers(i)->costHack;
					nBest = i;
				}
				costfile << helpers(i)->aico.cost - helpers(i)->costHack<< " ";
			}
		}costfile << endl;
		file << " best " << nBest << " " << lastCost << endl;
		for (uint i = 0; i < helpers.N; i++)
			helpers(i)->nBest = nBest;
#ifdef NIKAICO
		bwdMsg_v = helpers(nBest)->aico.qhat.sub(0,helpers(nBest)->aico.qhat.d0-1,0,6);//v or qhat ?? or q
		//bwdMsg_Vinv = helpers(nBest)->aico.Vinv;
#else
		bwdMsg_v = helpers(nBest)->aico.q;
#endif
		lastCost = helpers(nBest)->aico.cost - helpers(nBest)->costHack;
		if(lastCost < 0)
			HALT(" bad costs");
		cout << endl << endl << " BESTTT " << nBest << " cost: " << lastCost << endl;
		bReady = true;
		//file << " best msg " << bwdMsg_v[bwdMsg_count+1] << endl;
	}

	void close(){
		for(uint i = 0; i < helpers.N; i++)
			delete helpers(i);
	}

	void SetQV0(const arr & q, const arr & v){
		q0 = q;
		v0 = v;
		for(uint i = 0; i < helpers.N; i++){
			helpers(i)->q0 =q;
			helpers(i)->v0 = v;
			helpers(i)->bwdMsg_count = bwdMsg_count;			//   helpers(i)->aico.initMessages();
		}
	}
};

