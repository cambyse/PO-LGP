/*
 * NikolayRoutines.h
 *
 *  Created on: Jan 4, 2012
 *      Author: nikolay
 */

#ifndef NIKOLAYROUTINES_H_
#define NIKOLAYROUTINES_H_


#endif /* NIKOLAYROUTINES_H_ */




void ResetObjects(ors::Graph * C,const uintA & objects){
	ors::Body * table = C->getBodyByName("table");
	cout << " table " << table->X.pos << endl;
	//	for(uint z = 0; z < 1000; z++)		cout << rnd.uni() << endl;
	for(uint i = 0; i < objects.N-1; i++){
		ors::Body * temp = C->bodies(objects(i));
		temp->X.pos(0) = table->X.pos(0) + (rnd.uni()-0.5)*0.8;
		while( (temp->X.pos(0) > -0.15 && temp->X.pos(0) < 0.15)){
			double r = rnd.uni();
			//cout << " r " << r << " ";
			temp->X.pos(0) = table->X.pos(0) + (r-0.5)*0.8;
		}

		temp->X.pos(1) = table->X.pos(1) + (rnd.uni()-0.5)*0.3 +0.2;//closer to robot
		temp->X.pos(2) = table->X.pos(2) + 0.3;
		cout << temp->X.pos << endl;
	}
	sim.simulate(50);
	cout << endl << " resetting done " << endl << endl;
	for(uint i = 0; i < objects.N-1; i++){
		ors::Body * temp = C->bodies(objects(i));
		cout << temp->X.pos << endl;
	}
}

//print objects sorting by object acted on
void PrintData(const uintA & objs, const uint & nTarget, ors::Graph * C, ofstream & f, ofstream & f2){
	MT::IOraw = true;
	MT::Array< arr > objects_data;
	relational::getFeatureVectors(objects_data, *C, objs);
	for(uint i = 0; i < objects_data.N; i++){
		arr oneobj = objects_data(i);
		for(uint j = 0; j < oneobj.N; j++)
			f << oneobj(j) << " ";
	}
	f << endl;
	for(uint i = 0; i < objs.N; i++)
		if(i == nTarget)
			f2 << " 1 ";
		else f2 << " 0 ";
	f2 << endl;
}

MT::Array< arr > tracedata;
arr rewards;
int nDescr;//lenght of descriptor
int nObj;//cosntant how many objects
MT::String str;
uintA idxes;//permutes to irig data...

uintA RandPerm(int N){
	uintA r(N);
	for(uint i = 0; i < N; i++)
		r(i) = i;

	r.setRandomPerm(N);
	//cout << r << endl;
	return r;
}

//first N+N^2 permuted, target swappped, action and reward same
arr ReturnPermutedState(const arr & orig, const uintA& perm){
	arr p = orig*0.0;
	p(nDescr-3) = orig(nDescr-3);//action
	p(nDescr-1) = orig(nDescr-1);//reward

	int nOff = 65;//hacked, can be also read from similator probably
	//cout << " target " << orig(nDescr-2);
	p(nDescr-2) = perm(orig(nDescr-2)-nOff)+nOff;
	for (uint i = 0; i < nObj; i++)//unary
		p(i) = orig(perm(i));

	for (uint i = 0; i < nObj; i++)//binary
		for (uint j = 0; j < nObj; j++){
			uint idNew = i*nObj+j;
			uint idOld = perm(i)*nObj + perm(j);
			p(idNew) = orig(idOld);
		}
	//cout << p << endl << orig << endl;
	return p;
}

double ExpDifference(const arr & a, const arr & b){
	//	arr dif = a.sub(0,nDescr-3) - b.sub(0,nDescr-3) ;//difference between actions and rleational description
	double dif =0;
	if ( a(nDescr-1) == b(nDescr-1))//reward
			dif = 0;
		else
			dif = 1;
		if (dif ==1)
			return dif;

	if ( a(nDescr-2) == b(nDescr-2))//action
		dif = 0;
	else
		dif = 1;
	if (dif ==1)
		return dif;
	for(int i = nDescr-3; i >=0 ; i--){
		dif = fabs(a(i)-b(i));
		if (dif ==1)
			return dif;
	}
	return 0;
}

void EnhanceDataset(){
	MT::Array< arr > tracedata2(0);
	arr rewards2;
	int nS = 4000;//how many samples and also how far back to look
	for(uint i =0; i < tracedata.N; i++){
		//add originals
		//cout << endl << tracedata(i) << endl;
		tracedata2.append(tracedata(i));
		rewards2.append(rewards(i));
		for(uint j = 0; j < nS; j++){
			arr tmp = ReturnPermutedState(tracedata(i),RandPerm(nObj));
			bool bGood = true;
			if (ExpDifference(tmp,tracedata(i)) < 1)
				bGood = false;

			if(tracedata2.N>0)
				for(int z = tracedata2.N-1; z >=0 &&  tracedata2.N -z < 2*nS ;z--){
					if (ExpDifference(tmp,tracedata2(z))  < 1)
						bGood = false;
					if (!bGood)
						break;
				}

			if(bGood){//new and unique, add
				tracedata2.append(tmp);
				rewards2.append(rewards(i));
				idxes.append(i);
			}
		}
		cout << " after permutes " << tracedata2.N << " bef " << tracedata.N << endl;
	}
	cout << " after permutes " << tracedata2.N << " bef " << tracedata.N << endl;
	tracedata = tracedata2;
	rewards =rewards2;
}

TL::Atom* PredictTrace(const TL::SymbolicState& ss, const char * file){
	if (tracedata.N == 0 || str != MT::String(file)){
		double gamma = 0.8;
		tracedata =  init_getVector__for_the_nikolay(file);
		nDescr = tracedata(0).N;
		nObj = 10;//constant for now !!!
		str = MT::String(file);
		rewards = arr(tracedata.N);
		for(uint i =0 ; i < tracedata.N; i++){
			rewards(i) = 0;//tracedata(i)(nDescr-1);TOBIAS has false pre reward
			cout <<  " rew " << tracedata(i)(nDescr-1) << endl;
			for(uint j = 1; j < 3; j++)
				if (i +j < tracedata.N &
					i/10 == (i+j)/10	)//so in same episode...
					rewards(i)  = rewards(i) + pow(gamma,j)*tracedata(i+j)(nDescr-1);
		}
		EnhanceDataset();
		cout << endl << endl << " reward and states ready " << endl;
	}
	//cout << rewards;
	//just find the nearest neighbour inside experiences
	//TODO make permutation invariant
	//TODO calc discounted rewards for multiple episodes
	arr current;//stores current descriptor...
	getVector_StateLiterals(current,ss);
	double fmin = 1000000;
	int best = -1;
	for(uint i = 0; i < tracedata.N; i++){
		arr tmpexp = tracedata(i);
		arr tmpdesc = tmpexp.sub(0,nDescr-4);//last 3 are action targ reward
		double v = norm(tmpdesc-current);
		if (v <= fmin && (best == -1 || rewards(i) >= rewards(best)) ){
			fmin = v;
			best= i;
		}
	}
	arr bestexp = tracedata(best);
	int bestact = bestexp(nDescr-3);
	int besttarg = bestexp(nDescr-2);
	cout << endl << " best action " << best << "; dist: " << fmin << "; reward: " << rewards(best) << "; values: " << bestact << " " << besttarg << endl;
	cout << endl    		<< bestexp << endl;
	cout << endl << idxes(best) << endl;
	uintA args;  args.append(besttarg);
	TL::Predicate* p_GRAB = TL::logicObjectManager::getPredicate(MT::String("grab"));
	TL::Predicate* p_PUTON = TL::logicObjectManager::getPredicate(MT::String("puton"));
	TL::Atom* action = NULL;
	if (bestact == 0) {
		action = TL::logicObjectManager::getAtom(p_GRAB, args);
	}
	else
		action = TL::logicObjectManager::getAtom(p_PUTON, args);
	return action;
}

void NikGenerateData(){
	//	GRAB_UNCLEAR_OBJ_FAILURE_PROB = 0;
	MT::String sim_file;
	bool bSphere = true;
	if(!bSphere)
     sim_file = MT::String("situationNik.ors");
	else
		sim_file =MT::String("situationNikSphere.ors");

	initSimulator(sim_file, false);
	sim.simulate(50);
	TL::logicObjectManager::init("language.dat");

	uint num_reset = 30;//sequence length; if lessd than 40 - throw away
	uint num_actions = 20*num_reset;//mor than 1000...

	cout<<"GENERAL SIMULATOR INFORMATION:"<<endl;
	uintA objects;
	sim.getObjects(objects);
	if(bSphere)
	objects.append(67);//for sphere case
	objects.append(sim.C->getBodyByName("fing1c")->index);

	cout<<"Objects: "<<objects<<endl;
	objects.remove(0);//no table....
	cout<<"Objects: "<<objects<<endl;
	TL::logicObjectManager::setConstants(objects);

	ofstream f("X.dat");
	ofstream f2("T.dat");

	uint target_object = -4;
	uint k;
	TL::Predicate* p_action;
	TL::Predicate* p_GRAB = TL::logicObjectManager::getPredicate(MT::String("grab"));
	TL::Predicate* p_PUTON = TL::logicObjectManager::getPredicate(MT::String("puton"));
	uintA sa;
	sa.resize(1);
	for (k=0; k<num_actions; k++) {
		if (k % num_reset == 0)//better when no object in hand
			ResetObjects(sim.C,objects);
		uint old = target_object;
		do{	//change code so that always putting on diff block,avoid duplicate target; ok when grasping an object
			uint tmpnum = rnd.num(objects.N-1);
			target_object = objects(tmpnum);//finger is lat object, NEVER give it as target!!!
			sa(0) = target_object;
		}
		while(old == target_object);// && (k%2 == 1 || k == 0 )

		PrintData(objects,target_object-objects(0),sim.C,f,f2);
		if (k%2 == 0) { // grab
			cout<< k << " Grabbing "<<target_object<<"."<<endl;
			p_action = p_GRAB;
		}
		else { // put on
			cout<<"Putting on "<<target_object<<  " | " << old << "."<<endl;
			p_action = p_PUTON;
		}
		TL::Atom*  action = TL::logicObjectManager::getAtom(p_action, sa);
		TL::RobotManipulationDomain::performAction(action, &sim, 100);

		sim.relaxPosition();//note, this leads to inhand being unpredictable.... small noise...
		cout << " simN " << k << endl ;
		sim.simulate(60);//faster than 100
		PrintData(objects,target_object-objects(0),sim.C,f,f2);
	}

	cout<<"Please press button to continue."<<endl;
	sim.watch();

	sim.shutdownAll();



}
