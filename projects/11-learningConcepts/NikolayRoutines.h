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
	for(uint i = 0; i < objects.N; i++){
		ors::Body * temp = C->bodies(objects(i));
		temp->X.pos(0) = table->X.pos(0) + (rnd.uni()-0.5)*0.8;
		temp->X.pos(1) = table->X.pos(1) + (rnd.uni()-0.5)*0.2 +0.15;//closer to robot
		temp->X.pos(2) = table->X.pos(2) + 0.2;
	}
	sim.simulate(50);
	cout << endl << " resetting done " << endl << endl;
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

void NikGenerateData(){
	MT::String sim_file = MT::String("situationNik.ors");
	initSimulator(sim_file, false);
	sim.simulate(50);
	  TL::logicObjectManager::init("language.dat");

	uint num_reset = 30;//sequence length; if lessd than 40 - throw away
	uint num_actions = 60*num_reset;

	cout<<"GENERAL SIMULATOR INFORMATION:"<<endl;
	uintA objects;
	sim.getObjects(objects);
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
		if (k % num_reset== 0 )//better when no object in hand
			ResetObjects(sim.C,objects);
		uint old = target_object;
		do{	//change code so that always putting on diff block,avoid duplicate target; ok when grasping an object
			uint tmpnum = rnd.num(objects.N-1);
			target_object = objects(tmpnum);//finger is lat object, NEVER give it as target!!!
			sa(0) = target_object;
		}
		while(old == target_object && (k%2 == 1 || k == 0 ));
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
		sim.simulate(100);
		PrintData(objects,target_object-objects(0),sim.C,f,f2);
	}

	cout<<"Please press button to continue."<<endl;
	sim.watch();

	sim.shutdownAll();



}
