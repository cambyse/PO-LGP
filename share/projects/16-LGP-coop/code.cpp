
#include "code.h"

Coop::Coop() : poseView("pose"), seqView("sequence", 1., -0), pathView("path", .1, -1){}

void Coop::prepareKin(){
  kin.init("LGP-coop-kin.g");
  //  kin.watch();
  computeMeshNormals(kin.shapes);

  tableC = kin.getBodyByName("tableC");
  tableL = kin.getBodyByName("tableL");
  tableR = kin.getBodyByName("tableR");

  { //grab desired final configuration & create initial configuration, placing objects far on the table

    for(ors::Body *b:kin.bodies) if(b->name.startsWith("/toolbox")) box.append(b);

    targetAbs.resize(box.N);
    targetRel.resize(box.N, box.N);
    for(uint i=0;i<box.N;i++){
      targetAbs(i) = box(i)->X;
      for(uint j=0;j<i;j++) targetRel(i,j).setDifference(box(j)->X, box(i)->X);
    }

    double xpos = -.6;
    for(ors::Body *b:box){
      ors::Joint *j = b->inLinks.scalar();
      tableC->outLinks.removeValue(j);
      j->from = tableL;
      tableL->outLinks.append(j);
      kin.checkConsistency();

      j->B.setZero();
      j->B.addRelativeTranslation(xpos, 0,0);
      j->B.addRelativeRotationDeg(90,0,0,1);
      xpos += .15;
    }
    kin.calc_fwdPropagateFrames();
  }
//  kin.watch(/*true*/);
}

void Coop::prepareFol(){
//  fol.verbose = 5;
  fol.init(FILE("LGP-coop-fol.g"));
  //-- prepare logic world
//  for(ors::Body *b:box) fol.addObject(b->name);
  fol.addObject("/toolbox/handle");
  fol.addObject("screwdriverHandle");
  fol.addObject("screwbox");
  fol.addFact({"table","tableC"});
  fol.addFact({"table","tableL"});
  fol.addFact({"table","tableR"});
  fol.addAgent("baxterL");
  fol.addAgent("baxterR");
  fol.addAgent("handL");
  fol.addAgent("handR");

//  fol.addFact({"INFEASIBLE","activate_grasping","handR","screwdriverHandle"});

  fol.reset_state();
  FILE("z.start.fol") <<fol;

}

void Coop::prepareTree(){
  root = new ManipulationTree_Node(kin, fol);
  node = root;
}

void Coop::prepareDisplay(){
  threadOpenModules(true);
}

void Coop::updateDisplay(){
  if(node->poseProblem && node->poseProblem->MP->configurations.N)
    poseView.modelWorld.set() = *node->poseProblem->MP->configurations.last();
  if(node->seqProblem && node->seqProblem->MP->configurations.N)
    seqView.setConfigurations(node->seqProblem->MP->configurations);
  else seqView.clear();
  if(node->pathProblem && node->pathProblem->MP->configurations.N)
    pathView.setConfigurations(node->pathProblem->MP->configurations);
  else pathView.clear();


  for(auto& n:seqFringe) n->inFringe1=true;
  for(auto& n:pathFringe) n->inFringe2=true;

  Graph dot=root->getGraph();
  dot.writeDot(FILE("z.dot"), false, false, 0, node->graphIndex);
  system("dot -Tpdf z.dot > z.pdf");
}

void Coop::printChoices(){
  //-- query UI
  cout <<"********************" <<endl;
  cout <<"NODE:\n" <<*node <<endl;
  cout <<"--------------------" <<endl;
  cout <<"\nCHOICES:" <<endl;
  cout <<"(q) quit" <<endl;
  cout <<"(u) up" <<endl;
  cout <<"(p) pose problem" <<endl;
  cout <<"(s) sequence problem" <<endl;
  cout <<"(x) path problem" <<endl;
  cout <<"(m) MC planning" <<endl;
  uint c=0;
  for(ManipulationTree_Node* a:node->children){
    cout <<"(" <<c++ <<") DECISION: " <<*a->decision <<endl;
  }
}

mlr::String Coop::queryForChoice(){
  mlr::String cmd;
  std::string tmp;
  getline(std::cin, tmp);
  cmd=tmp.c_str();
  return cmd;
}

bool Coop::execRandomChoice(){
  mlr::String cmd;
  if(mlr::rnd.uni()<.5){
    switch(mlr::rnd.num(5)){
      case 0: cmd="u"; break;
      case 1: cmd="p"; break;
      case 2: cmd="s"; break;
      case 3: cmd="x"; break;
      case 4: cmd="m"; break;
    }
  }else{
    cmd <<rnd(node->children.N);
  }
  return execChoice(cmd);
}

bool Coop::execChoice(mlr::String cmd){
  cout <<"COMMAND: '" <<cmd <<"'" <<endl;

  if(cmd=="q") return false;
  else if(cmd=="u"){ if(node->parent) node = node->parent; }
  else if(cmd=="p") node->solvePoseProblem();
  else if(cmd=="s") node->solveSeqProblem();
  else if(cmd=="x") node->solvePathProblem(20);
  else if(cmd=="m") node->addMCRollouts(100,10);
  else{
    int choice;
    cmd >>choice;
    cout <<"CHOICE=" <<choice <<endl;
    if(choice>=(int)node->children.N){
      cout <<"--- there is no such choice" <<endl;
    }else{
      node = node->children(choice);
      if(!node->isExpanded){
        node->expand();
        if(autoCompute){
          node->solvePoseProblem();
          //          node->solveSeqProblem();
          //          node->solvePathProblem(20);
        }
      }
    }
  }
  return true;
}
