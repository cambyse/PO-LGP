/*
 * EnergyRoutines.h
 *
 *  Created on: Nov 22, 2010
 *      Author: nikolay
 */
#ifndef ENERGYROUTINES_H_
#define ENERGYROUTINES_H_
#endif /* ENERGYROUTINES_H_ */

arr meaN;
arr prN;
arr w1,w2,w3;
arr lastCenter;//for object positons in train data create
int bDynamical;
MT::String sDiscriminant;

MT::Array<ors::Shape*> GetLandmarks(ors::Graph * g){
	ors::Shape * s1 = g->getShapeByName("tipNormal1");
	ors::Shape * s2 = g->getShapeByName("tipNormal2");
	ors::Shape * s3 = g->getShapeByName("tipNormal3");
	ors::Shape * s4 = g->getShapeByName("fingNormal1");
	ors::Shape * s5 = g->getShapeByName("fingNormal2");
	ors::Shape * s6 = g->getShapeByName("fingNormal3");
	ors::Shape * o1 = g->getBodyByName("o1")->shapes(0);
	ors::Shape * palm = g->getShapeByName("palmCenter");
	MT::Array<ors::Shape*> landmarks(8);
	landmarks(1) = s1;landmarks(2) = s2;landmarks(3) = s3;
	landmarks(4) = s4;landmarks(5) = s5;landmarks(6) = s6;
	landmarks(0) = o1;
	landmarks(7) = palm;
	return landmarks;
}

ors::Vector GetRelPos(ors::Shape* a,ors::Shape* b){
	ors::Vector pi,pj,c;
	pi = a->body->X.pos + a->body->X.rot * a->rel.pos;
	pj = b->body->X.pos + b->body->X.rot *b->rel.pos;
	c = b->X.rot / (pi-pj);
	return c;
}

arr GetRelJacobian(ors::Shape* a,ors::Shape* b,ors::Graph * G){
	arr J,Ji,Jj,JRj;
	ors::Vector pi,pj,c,vi,vj,r,jk;
	uint k;
	pi = a->body->X.pos + a->body->X.rot * a->rel.pos;
	pj = b->body->X.pos + b->body->X.rot * b->rel.pos;
	G->jacobianPos(Ji,a->body->index,&a->rel);
	G->jacobianPos(Jj,b->body->index,&b->rel);
	G->jacobianR(JRj,b->body->index);
	J.resize(3,Jj.d1);
	for(k=0;k<Jj.d1;k++){
		vi.set(Ji (0,k),Ji (1,k),Ji (2,k));
		vj.set(Jj (0,k),Jj (1,k),Jj (2,k));
		r .set(JRj(0,k),JRj(1,k),JRj(2,k));
		jk =  b->X.rot / (vi-vj);
		jk -= b->X.rot / (r^(pi-pj));
		J(0,k)=jk(0); J(1,k)=jk(1); J(2,k)=jk(2);
	}
	return J;
}

arr GetRawFeaturesJ(const MT::Array<ors::Shape*> & landmarks,ors::Graph * G,arr & grad){
	uint br = 0;
	uint M = 4;
	arr ans(landmarks.N*(landmarks.N-1)*M);
	arr gradT(landmarks.N*(landmarks.N-1)*M,G->getJointStateDimension());
	for(uint j = 0; j < landmarks.N; j++)
		for(uint i = 0; i < landmarks.N; i++)
			if (i!=j)
			{
				ors::Vector f = GetRelPos(landmarks(j),landmarks(i));
				if(f.length()==0)f(0)+=0.00000001;
				ans(br*M) =   f(0);
				ans(br*M+1) = f(1);
				ans(br*M+2) = f(2);
				ans(br*M+3) = f.length();
				//	ans(br*M+4) = acos(f(2)/f.length());
				//ans(br*6+5) = atan2(f(1),f(0));
				arr Jt = GetRelJacobian(landmarks(j),landmarks(i),G);
				gradT[br*M] = Jt[0];//just coords
				gradT[br*M+1] = Jt[1];
				gradT[br*M+2] = Jt[2];
				arr mult = ans.sub(br*M,br*M+2)/ f.length();
				gradT[br*M+3] =  mult*Jt;//distance grad
				/*	mult(0) = f(2)*-1/(f.length()*f.length())*f(0)/f.length();
				mult(1) = f(2)*-1/(f.length()*f.length())*f(1)/f.length();
				mult(2) = (f.length()-f(2)*f(2)/f.length())/(f.length()*f.length());*(r - z*z/r) /r^2;
				mult *= (-1/sqrt(1- (f(2)/f.length())*(f(2)/f.length()) ));
				gradT[br*M+4] =  mult*Jt;//acos grad*/
				/*mult(0) = 1/(1+(f(1)/f(0))*(f(1)/f(0)))*-f(1)/(f(0)*f(0));
				mult(1) = 1/(1+(f(1)/f(0))*(f(1)/f(0)))/f(0);
				mult(2) = 0;
				gradT[br*6+5] =  mult*Jt;//atan grad*/
				br++;
			}
	transpose(grad,gradT);
	return ans;
}

arr GetRawFeatures(const MT::Array<ors::Shape*> & landmarks){
	uint br = 0;
	uint M = 4;
	arr ans(landmarks.N*(landmarks.N-1)*M);
	for(uint j = 0; j < landmarks.N; j++)
		for(uint i = 0; i < landmarks.N; i++)
			if (i!=j)
			{
				ors::Vector f = GetRelPos(landmarks(j),landmarks(i));
				if(f.length()==0)f(0)+=0.00000001;
				ans(br*M) =   f(0);
				ans(br*M+1) = f(1);
				ans(br*M+2) = f(2);
				ans(br*M+3) = f.length();
				//ans(br*M+4) = acos(f(2)/f.length());
				//		ans(br*5+4) = acos(f(2)/f.length());
				br++;
			}
	return ans;
}

arr GetFeaturesDyn(const MT::Array<ors::Shape*> & landmarks,const arr & q, const arr & lastRaw, const arr & lastJoint,ors::Graph * G,arr & grad){
	arr gradR;
	arr raw = GetRawFeaturesJ(landmarks,G,gradR);
	uint nOffset = raw.N + q.N;
	arr TD(2*nOffset);
	for(uint i =0 ; i < raw.N; i++){
		TD(i) = lastRaw(i);//hack to see how important *0.0
		TD(i+nOffset) = raw(i);
	}
	for(uint i =0 ; i < q.N; i++){
		TD(i+raw.N) = lastJoint(i);
		TD(i+raw.N + nOffset) = q(i);
	}

	arr gradF(q.N,TD.N);gradF = 0;

	for(uint i = 0; i < q.N; i++ )
		gradF(i,raw.N+nOffset + i) = 1;//just joint vector gradients on diagonal

	for(uint i = 0; i < q.N; i++ )
		for(uint j = 0; j < raw.N; j++ )
			gradF(i,j+nOffset) = gradR(i,j);

	TD -=meaN;//mean and scale with projection PCA
	TD = TD*prN;
	grad = gradF*prN;
	TD = TD-0.5;
	//for(uint i =0 ; i < TD.d1; i++)
	//		TD(i+nOffset) = TD(i+nOffset)- TD(i);//try really U signals
	return TD;
}

arr GetFeatures(const MT::Array<ors::Shape*> & landmarks,const arr & q, ors::Graph * G,arr & grad){
	arr gradR;
	arr raw = GetRawFeaturesJ(landmarks,G,gradR);
	arr TD(raw.N + q.N);
	for(uint i =0 ; i < raw.N; i++)
		TD(i) = raw(i);
	for(uint i =0 ; i < q.N; i++)
		TD(i+raw.N) = q(i);

	arr gradF(q.N,TD.N);gradF = 0;

	for(uint i = 0; i < q.N; i++ ){
		gradF(i,raw.N+i) = 1;//just joint vector gradients on diagonal
	}
	for(uint i = 0; i < q.N; i++ )
		for(uint j = 0; j < raw.N; j++ ){
			gradF(i,j) = gradR(i,j);
		}
	TD -=meaN;//mean and scale with projection PCA
	TD = TD*prN;
	grad = gradF*prN;
	return TD-0.5;
}

arr GetFeatures(const MT::Array<ors::Shape*> & landmarks,const arr & q, ors::Graph * G){
	arr temp;
	return GetFeatures(landmarks,q,G,temp);
}
double MLPsc(const arr & feat,const arr & w1,const arr & w2, arr & grad){
	arr l0 = feat*w1.sub(0,w1.d0-2,0,w1.d1-1);
	l0 += w1.sub(w1.d0-1,w1.d0-1,0,w1.d1-1);
	arr sigm = 1./(1.0 + exp(-l0));
	double sc = scalarProduct(w2,sigm) + scalarProduct(w3,feat);//no bias terms ghere
	arr gradS(sigm.d0,sigm.d0);gradS = 0;
	for (uint i = 0; i < gradS.d0; i++)
		gradS(i,i) = sigm(i)*(1.0-sigm(i));
	grad = w1.sub(0,w1.d0-2,0,w1.d1-1)*gradS*w2 + w3;
	return sc;
}

double MLPsc(const arr & feat,const arr & w1,const arr & w2){
	arr tmp;
	return MLPsc(feat,w1,w2,tmp);
}

double MLP(const arr & feat){
	return MLPsc(feat,w1,w2);
}

double MLP(const arr & feat, arr & grad){
	return MLPsc(feat,w1,w2,grad);
}


arr MLPa(const arr & feat,const arr & w1,const arr & w2, arr & grad){
	arr l0 = feat*w1.sub(0,w1.d0-2,0,w1.d1-2);//pruned or unpruned
	l0 += w1.sub(w1.d0-1,w1.d0-1,0,w1.d1-2);

	arr sigm = 1./(1.0 + exp(-l0));
	arr sc = sigm*w2.sub(0,w2.d0-2,0,w2.d1-1);
	sc = sc + w2.sub(w2.d0-1,w2.d0-1,0,w2.d1-1);//linear comb + bias
	//return sc;

	arr gradS(sigm.d0,sigm.d0);gradS = 0;
	for (uint i = 0; i < gradS.d0; i++)
		gradS(i,i) = sigm(i)*(1.0-sigm(i));

	grad = w1.sub(0,w1.d0-2,0,w1.d1-2)*gradS*w2.sub(0,w2.d0-2,0,w2.d1-1);
	return sc;
}

arr MLPa(const arr & feat,const arr & w1,const arr & w2){
	arr tmp;
	return MLPa(feat,w1,w2,tmp);
}


arr MLPa(const arr & feat){
	return MLPa(feat,w1,w2);
}

//deviation from linear prediction
double DLP(const arr & feat, const arr & lastF, arr & grad){
	arr feat1(feat.N+1);feat1 = 1.0;
	for(uint i = 0; i < feat.N; i++)
		feat1(i) = lastF(i);//for bias consdt term

	arr diff = feat - (feat1*w1 + lastF);
	double ans = sumOfSqr(diff);
	grad = 2.0*diff;
	return ans;
}

double DLPml(const arr & feat, const arr & lastF, arr & grad){
	arr predJ = MLPa(lastF);//just joint space prediction
//	cout << " joint predict " << predJ << endl << " real joint " << feat.sub(feat.N-14,feat.N-1);
	arr pred = feat*1.0;//hack to make other task spaces don't matter
	for(uint i = 0; i < 14; i++)
		pred(i+feat.N-14) = predJ(i);
	arr diff = feat - (pred + lastF);
	for(uint i = 0; i < feat.N-14; i++)
		diff(i) = 0;//hack to make other task spaces don't matter
	double ans = sumOfSqr(diff);
	grad = 2.0*diff;
	return ans;
}

void PrintJointsNoise(RobotProcessGroup & robot, const arr& joints, arr & lastJoint){
	MT::Array<ors::Shape*> landmarks = GetLandmarks(&robot.ctrl.ors);
	cout << " noise creation " << endl;
	uint noi = 60;
	ofstream f("noise.txt",ofstream::app);
	ofstream fJ("Jacobians.txt",ofstream::app);
	ofstream ff("pos.txt",ofstream::app);
	ff << lastCenter << endl;
	//generate noisy samples, collision free!!!
	for(uint i = 0; i < joints.d0; i+=10){//limit to less steps
		double fNorm;//try fixed norm noise
		fNorm = norm(joints[i]-lastJoint);//cout << endl << lastJoint << endl << joints[i] << endl;
		cout << i << endl;
		for(uint n = 0; n < noi; n++)
		{
			arr noise = joints[i]*1.0;//careful, without the 1.0 it will change reference !!!
			int br = 0;//max checks for no coll
			while(br++ < 100){//repeat until no collisions
				noise = joints[i];
				if (n > 0){
					arr nA(joints.d1);nA = 0;
					while(norm(nA) < 0.006)
					{
						rndGauss(nA,0.07);
						if (n  < 10){//single joint change
							uint g = rnd.uni()*noise.N;
							for(uint h = 0; h < noise.N; h++)
								if(h!=g)nA(h) = 0;
						}
						else if (i > joints.d0-5 && n >= 40)//just move fingers, it is most surely converged
							for(uint h = 0; h < 7; h++)
								nA(h)= 0;
					}
					if(false)noise = lastJoint + nA/norm(nA)*fNorm;//fixed norm noise or not
					else
						noise = lastJoint + nA;
				}
				robot.ctrl.ors.setJointState(noise);
				robot.ctrl.ors.calcBodyFramesFromJoints();
				robot.ctrl.swift.computeProxies(robot.ctrl.ors,false);
				bool bNoCollision = true;
				for(uint c = 0; c < robot.ctrl.ors.proxies.N; c++)
					if (robot.ctrl.ors.proxies(c)->d < 0.001){
						//robot.ctrl.ors.reportProxies();
						bNoCollision = false; break;
					}
				if(bNoCollision)
					break;
			}
			f << i << " " << n << " " << GetRawFeatures(landmarks) << " " << noise << endl;
			if(n == 0) {
				arr J;
				GetRawFeaturesJ(landmarks, &robot.ctrl.ors, J);
				fJ << J << endl;
			}
			if (false){
				robot.gui.gl->text.clear()<< i << endl;
				//robot.gui.q_reference = noise;
				MT::wait(0.1);
				robot.gui.threadStep();
			}
		}
		lastJoint = joints[i];//		cout << endl << lastJoint << endl << joints[i] << endl;
	}
}

//move by collision free sampling
void SampleTrajectory(RobotProcessGroup & robot){
	MT::Array<ors::Shape*> landmarks = GetLandmarks(&robot.ctrl.ors);
	cout << " sampling creation " << endl;
	int nSa = MT::getParameter<int>("iterations");
	uint T = MT::getParameter<int>("T");
	arr lastJoint = robot.ctrl.q_reference;
	arr lastFeature,bestF;
	lastFeature = GetRawFeatures(landmarks);//GetRawFeatures(landmarks);
	arr best(T,lastJoint.N);
	best[0] = lastJoint;
	for(uint t = 1; t < T; t++){//limit time steps{
		double fMin = 10000;
		for(uint n = 0; n < nSa; n++)
		{
			arr noise;
			while(true){//repeat until no collisions
				bool bNoCollision = true;
				noise = lastJoint;
				if (n > 0){
					arr nA(lastJoint.N);rndGauss(nA,0.004);
					noise = noise + nA;
				}
				robot.ctrl.ors.setJointState(noise);
				robot.ctrl.ors.calcBodyFramesFromJoints();
				robot.ctrl.swift.computeProxies(robot.ctrl.ors,false);
				for(uint c = 0; c < robot.ctrl.ors.proxies.N; c++)
					if (robot.ctrl.ors.proxies(c)->d < 0.006){
						bNoCollision = false; break;
					}
				if(bNoCollision || true)//turn off collisions
					break;
			}
			arr gf;
			arr feat;

			if(bDynamical > 0)
				feat = GetFeaturesDyn(landmarks,noise,lastFeature,lastJoint,&robot.ctrl.ors,gf);
			else
				feat	= GetFeatures(landmarks,noise,&robot.ctrl.ors,gf);

			double Energy = MLP(feat);
			if (Energy < fMin ){
				fMin = Energy;
				best[t] = noise;
				//bestF = feat;
				bestF = GetRawFeatures(landmarks);
			}
		}
		robot.ctrl.ors.setJointState(best[t]);//
		robot.ctrl.ors.calcBodyFramesFromJoints();
		lastFeature = bestF;
		lastJoint = best[t];
		if (true){//display best sample
			robot.ctrl.q_reference = best[t];
			robot.gui.gl->text.clear()<< t << " best " << fMin << endl;
			robot.step();
		}
	}
	//now loop undendlessly
	for(uint c = 0; c < 10; c++)
		for(uint t = 0; t < T; t++){
			MT::wait(0.15);
			robot.ctrl.q_reference = best[t];
			robot.gui.gl->text.clear()<< t<< endl;
			robot.step();
		}
}

void GradTrajectory(RobotProcessGroup & robot){
	MT::Array<ors::Shape*> landmarks = GetLandmarks(&robot.ctrl.ors);
	cout << " sampling creation " << endl;
	int nSa = MT::getParameter<int>("iterations");
	uint T = MT::getParameter<int>("T");
	arr lastJoint = robot.ctrl.q_reference;
	arr lastFeature,bestF;
	if(bDynamical == 1)
		lastFeature = GetRawFeatures(landmarks);//GetRawFeatures(landmarks);
	else
		lastFeature = GetFeatures(landmarks,lastJoint,&robot.ctrl.ors);
	arr best(T,lastJoint.N);
	best[0] = lastJoint;

	for(uint t = 1; t < T; t++){//limit time steps{
		cout << endl << " Time" << t << endl;
		arr noise = lastJoint;
		for(uint n = 0; n < nSa; n++)
		{
			arr gE,gf,feat;
			if(bDynamical == 1)
				feat = GetFeaturesDyn(landmarks,noise,lastFeature,lastJoint,&robot.ctrl.ors,gf);
			else
				feat	= GetFeatures(landmarks,noise,&robot.ctrl.ors,gf);
			double Energy;
			if(bDynamical <2) Energy= MLP(feat,gE);
			if(bDynamical == 2) Energy= DLP(feat,lastFeature,gE);
			cout << Energy << endl;
			arr gTotal = gf*gE;
			noise -= 0.0009*gTotal;
			best[t] = noise;
			robot.ctrl.ors.setJointState(best[t]);//
			robot.ctrl.ors.calcBodyFramesFromJoints();
			//robot.ctrl.swift.computeProxies(robot.ctrl.ors,false);
			//robot.ctrl.ors.reportProxies();
		}

		if(bDynamical == 1)
			bestF = GetRawFeatures(landmarks);
		else
			bestF = GetFeatures(landmarks,lastJoint,&robot.ctrl.ors);
		lastFeature = bestF;
		lastJoint = best[t];
		if (true){//display best sample
			robot.ctrl.q_reference = best[t];
			robot.gui.gl->text.clear()<< t << endl;
			robot.step();
		}
	}
	//now loop undendlessly
	for(uint c = 0; c < 0; c++)
		for(uint t = 0; t < T; t++){
			MT::wait(0.15);
			robot.ctrl.q_reference = best[t];
			robot.gui.gl->text.clear()<< t<< endl;
			robot.step();
		}
}


namespace NJCH {

MT::Array<ors::Shape*> landmarks;
ors::Graph * G;

void f(arr &y,const arr &x,void*){
	arr a(39,8); a = 1;
	arr b(8); b = 1;
	double yy = MLPsc(x,a,b);
	y = arr(1);
	y(0) = yy;
}

void  df(arr &J,const arr &x,void*){
	arr a(39,8); a = 1;
	arr b(8); b = 1;

	MLPsc(x,a,b,J);

}

/*
void f2(arr &y,const arr &x,void*){
	arr J;
	double yy = MLPMix(x,J);
	y = arr(1);
	y(0) = yy;
}

void  df2(arr &J,const arr &x,void*){
	MLPMix(x,J);

}*/

void f5(arr &y,const arr &x,void*){
	G->setJointState(x);
	G->calcBodyFramesFromJoints();
	y = GetRawFeatures(landmarks);
	//y = y.sub(50,100);
	//cout << " y " << y << endl;
	/*ors::Shape * s1 = G->getBodyByName("m3")->shapes(0);
	ors::Shape * s2 = G->getBodyByName("m5")->shapes(0);
	ors::Vector d = GetRelPos(s1,s2);
	y = arr(d.p,3);
	return;*/
	/*arr lastQ(14);
	arr lastFeat(168);
	arr grad;
	y = GetDynFeatures(landmarks, x, lastFeat*0.0 ,lastQ*0.0, G,grad);*/
}

void  df5(arr &J,const arr &x,void*){
	G->setJointState(x);
	G->calcBodyFramesFromJoints();
	/*ors::Shape * s1 = G->getBodyByName("m3")->shapes(0);
	ors::Shape * s2 = G->getBodyByName("m5")->shapes(0);
	arr Jt =GetRelJacobian(s1,s2,G);
	transpose(J,Jt);
	return;*/
	/*arr lastQ(14);
	arr lastFeat(168);
	GetDynFeatures(landmarks, x, lastFeat*0.0 ,lastQ*0.0,G,J);*/
	GetRawFeaturesJ(landmarks,G,J);
	//J = J.sub(0,13,0,10);
}

/*
void f6(arr &y,const arr &x,void*){
	G->setJointState(x);
	G->calcBodyFramesFromJoints();
	arr lastQ(14);
	arr lastFeat(288);
	arr grad;
	y = GetDynFeatures(landmarks, x, lastFeat*0.0 ,lastQ*0.0, G,grad);
	double e = MLP(y);
	y = arr(1); y(0) = e;
}

void  df6(arr &J,const arr &x,void*){
	G->setJointState(x);
	G->calcBodyFramesFromJoints();
	arr lastQ(14);
	arr lastFeat(288);
	arr grad;
	arr y = GetDynFeatures(landmarks, x, lastFeat*0.0 ,lastQ*0.0,G,grad);
	double e = MLP(y,J);
	J = grad*J;
}*/

void
checkGradient(void (*f)(arr&, const arr&,void*),
		void (*df)(arr&,const arr&,void*),
		void *data,
		const arr& x,real tolerance){
	arr y,J,dx,dy,JJ;
	f(y,x,data);
	df(J,x,data);

	JJ.resize(y.N,x.N);
	real eps=CHECK_EPS*1e0;
	uint i,k;
	for(i=0;i<x.N;i++){
		dx=x;
		dx.elem(i) += eps;
		f(dy,dx,data);
		dy = (dy-y)/eps;
		for(k=0;k<y.N;k++) JJ(k,i)=dy.elem(k);
	}
	//cout << JJ;//JJ.reshapeAs(J);
	arr JJt;transpose(JJt,JJ);JJ = JJt;
	real md=maxDiff(J,JJ,&i);
	if(md>tolerance){
		MT_MSG("checkGradient -- FAILURE -- \nmax diff=" <<md);
		cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
		arr diff = JJ- J;
		for(i = 0; i < JJ.d0; i++)
			for(k = 0; k < JJ.d1; k++)
				if(diff(i,k) < tolerance)
					diff(i,k)=0;
		cout << "\n diff " << diff << endl;
		//HALT("checkGradient -- FAILURE -- \n measured grad=" <<JJ <<"\ncomputed grad=" <<J <<"\nmax diff=" <<md);
	}else{
		cout <<"checkGradient -- SUCCESS (max diff error=" <<md <<")" <<endl;
	}
}


void CheckManifold(const arr & q){
	arr lastQ = q;
	arr lastFeat(288);lastFeat = GetRawFeatures(landmarks);
	arr grad,J;
	arr y;// = GetDynFeatures(landmarks, q, lastFeat ,lastQ,G,grad);
	//double e = MLP(y,J);
	arr Jt = grad*J;//grad by q of energy
	transpose(J,Jt);

	double fMin = 1000;
	uint nSa = 10000;
	arr best;
	for(uint n = 0; n < nSa; n++)
	{
		arr noise;
		while(true){//repeat until no collisions
			bool bNoCollision = true;
			noise = lastQ;
			if (n > 0){
				arr nA(lastQ.N);rndGauss(nA,0.05);
				noise = noise + nA;
			}
			/*robot.ctrl.ors.setJointState(noise);
			robot.ctrl.ors.calcBodyFramesFromJoints();
			robot.ctrl.swift.computeProxies(robot.ctrl.ors,false);
			for(uint c = 0; c < robot.ctrl.ors.proxies.N; c++)
				if (robot.ctrl.ors.proxies(c)->d < 0.006){
					bNoCollision = false; break;
				}*/
			if(bNoCollision)
				break;

		}
		G->setJointState(noise);
		G->calcBodyFramesFromJoints();
		arr gf;
		arr feat;// = GetDynFeatures(landmarks,noise,lastFeat,lastQ,G,gf);
		double Energy = MLP(feat);
		//cout << Energy << " " << fMin << endl;
		if (Energy < fMin ){
			fMin = Energy;
			best = noise;
			cout << " best " << best << endl << " diff " << best - lastQ << endl << " grad " << -J << endl << endl;
		}
	}
}

void NJCheck(ors::Graph * C){
	G = C;
	landmarks = GetLandmarks(G);
	arr q;
	G->getJointState(q);

	CheckManifold(q);return;

	for(uint i = 0; i < 3; i++){
		rndGauss(q,0.01,true);
		//	checkGradient(f6,df6,NULL,q,0.03);
	}
}
}

