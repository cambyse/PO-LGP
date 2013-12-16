/*
 * ORSImitate.cpp
 *
 *  Created on: Nov 30, 2010
 *      Author: nikolay
 */

//#include "ORSImitate.h"


void ImitateTask::updateTaskVariables(ControllerModule *ctrl){
	activateAll(TVall,false); //deactivate all variables
	ctrl->useBwdMsg=false;             //deactivate use of bwd messages (from planning)
    int nh = 1;
	TV_lim->active            = true;
	TV_lim->y_prec = 1e1;
	TV_col->active            = true;
	TV_col->y_prec = 1e0 + 400*nh;//1e0
	TV_q->active = true;
	TV_q->v_prec= 1e0 + 3*nh; //damp velocities
	TV_q->v_target.setZero();

	TVImitate->active = true;
	if(bDynamical >= 2){//DPL case
		TVImitate->y_prec = 1e5;
		TVImitate->y_target = 0;
		TV_col->y_prec = 1e-1;TV_lim->y_prec  = 1e-1;TV_q->v_prec= 1e0;
	}
	if(bDynamical == 1){//structured output case
		TVImitate->y_prec = 1e3;
		TVImitate->y_target = -1;
	}
	if(bDynamical == 0){//minimize cost
		TVImitate->v_prec = 1e3 - 100*nh;//1e3
		TVImitate->v_target = -0.03 + 0.01*nh;//-0.09
	}
	cout << " collision " <<  TV_col->y << " ";
}

void ImitateTask::initTaskVariables(ControllerModule *ctrl){
	TVImitate = new ImitateTaskVariable("grasp imitate",ctrl->ors);
	TVall.append(TVImitate);

	TaskAbstraction::initTaskVariables(ctrl);
}


ImitateTaskVariable::ImitateTaskVariable(const char* _name, ors::KinematicWorld& _ors){
	landmarks = GetLandmarks(&_ors);
	nCounter = 1;//change time for dynamic case
	nIterations = MT::getParameter<int>("iterations");

	set(_name, _ors, userTVT, -1, Transformation_Id, -1, Transformation_Id, ARR());


	userUpdate();//initial values...
	targetType = directTT;

}

void ImitateTaskVariable::userUpdate(){
	//y and J set
	arr q;
	ors->getJointState(q);
	if(lastJoint.N == 0){//start from rest
		lastJoint = q;
		if(bDynamical >= 2) lastFeature = GetFeatures(landmarks,q,ors);
		if(bDynamical == 1)lastFeature = GetRawFeatures(landmarks);
	}
	arr gf,feat;
	if(bDynamical ==1 )
		feat = GetFeaturesDyn(landmarks,q,lastFeature,lastJoint,ors,gf);
	else
		feat = GetFeatures(landmarks,q,ors,gf);
	arr gE;
	double Energy;
	if(bDynamical == 0 || bDynamical ==1) Energy= MLP(feat,gE);
	if(bDynamical == 2) Energy= DLP(feat,lastFeature,gE);
	if(bDynamical == 3) Energy= DLPml(feat,lastFeature,gE);
	arr gTotal = gf*gE;

	y = arr(1);y(0)= Energy;
	transpose(J, gTotal);//strange, q in row dimension, d1,,, should have been in d0 ?
	cout << " energy " <<  y << endl;

	if((nCounter++)%nIterations == 0){
		lastJoint = q;
		if(bDynamical >= 2) lastFeature = GetFeatures(landmarks,q,ors);
		if(bDynamical == 1) lastFeature = GetRawFeatures(landmarks);
	}

}
