/* PIP(T=25) Task: Reach the goal in exactly 25 time steps (not earlier). The
displayed policy is only valid for one time step (the next time step
is is recalculated from the $\b$'s.) For states close to the goal, the
policy is to move towards the save center (away from the trap-walls)
as a waiting location. For states farer away, the policy is very
goal-directed; in the extreme cast even at the cost of walking all the
way close to the trap-walls.

There is not tuning of the gamma parameters such that DP could create
such policies. DP($\g=1$) creates a savest possible policy, first
moving away from the trap-walls. DP($\g=.9$) is wuite greedily
goal-directed. */
void onTime();

/* PIP(0\le T\le 25) Task: Reach the goal within $0<T<25$ time
   steps. PIP creates a policy that is save (keeping away from walls)
   for short paths (since enough time is allowed) but greedy for long
   paths (where one has to hurry to reach the goal within 25 time
   steps.

   DP can either be save or greedy. No parameter tuning of $\g$ can
   create such policies */
void timeWindow();

/* Task: Analyze different possibilities/trade-offs between security
   and time. Start: bottom left, goal: bottom right. Three possible
   trespasses. Bottom one very dangerous (to get trapped), middle one
   medium, top one secure. To select between these three options using
   DP, one has to fiddle around with $\g$ and rerunning the algorithm
   several times. PIP(0\le T) generates a multimodal time posteriors.
*/
void threeHoles();

void largeMaze();

void paper(){
  //onTime();
  //timeWindow();
  //threeHoles();
  largeMaze();
}

void savePiFig(Planner& p,Planner::node to,const char* file){
  uint i; Planner::edge e; Planner::node n;
  std::cout <<"exporting pi FIG file";
  forNodes(n,p.G) if(n!=to && n->Nout==1){
    n->disp().hide=false;
    n->disp().shape=1;
    n->disp().size=mazeDX;
    n->disp().col.setGray(0);
  }else n->disp().hide=true;
  forEdges(i,e,p.G) if(e->ppi<.5 || e->from==e->to) e->disp().hide=true; else e->disp().col.setGray(0);
  for_out_edges(e,to) e->disp().hide=true;
  to->disp().hide=false; to->disp().size=.5*mazeDX; to->disp().col.setGray(0);
  p.G.saveFIG(file);
  std::cout <<" -- done" <<std::endl;
}
void saveGvisFig(Planner& p,Planner::node to,const char* file){
  uint i; Planner::edge e; Planner::node n;
  std::cout <<"exporting Gvis FIG file"; std::cout.flush();
  forNodes(n,p.G) if(n!=to && n->Nout==1){
    n->disp().hide=false;
    n->disp().shape=1;
    n->disp().size=mazeDX;
    n->disp().col.setGray(0);
  }else{
    if(n->Gvis>.01){
      n->disp().hide=false;
      n->disp().size=.9*mazeDX*n->Gvis;
      n->disp().col.setGray(0);
    }else{
      n->disp().hide=true;
    }
  }
  forEdges(i,e,p.G) e->disp().hide=true;
  p.G.saveFIG(file);
  std::cout <<" -- done" <<std::endl;
}
void unhideAll(Planner& p){
  uint i; Planner::edge e; Planner::node n;
  forNodes(n,p.G) n->disp().hide=false;
  forEdges(i,e,p.G) e->disp().hide=false;
}

void onTime(){
  Planner p;
  Planner::node a,b;
  OpenGL gl; gl.add(p.G);

  imageToGraph("mazes/20x10empty.png",p.G);

  a=p.G.nodes(25); b=p.G.nodes(38);
  p.G.del_out_edges(b); p.G.new_edge(b,p.G.first); //let goal lead to trap
  p.normalizeNoisePi();
  
  //my planning
  p.timePriorType='w'; p.timeWindowL=20; p.timeWindowH=20;
  p.iteratedPlan(5,a,b,false);
  system(STRING("cp T data/T-onTime"));
  system(STRING("cp Z data/Z-onTime"));
  savePiFig(p,b,STRING("data/onTime.fig"));

  gl.text.clr() <<"onTime my planning";
  do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  unhideAll(p);

  //dynamic programming
  p.discount=1.;
  p.installRandomPolicy();
  p.valueIteration(a,b);
  p.installMaxQPolicy(); p.layoutPolicy();
  savePiFig(p,b,STRING("data/onTime-DP-1.fig"));

  gl.text.clr() <<"onTime DP, gamma=1";
  do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  unhideAll(p);

  //dynamic programming
  p.discount=.9;
  p.installRandomPolicy();
  p.valueIteration(a,b);
  p.installMaxQPolicy();
  savePiFig(p,b,STRING("data/onTime-DP-09.fig"));

  gl.text.clr() <<"onTime DP, gamma=.9";
  do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  unhideAll(p);

  //my planning
  p.timePriorType='d'; p.discount=1.;
  p.iteratedPlan(5,a,b,false);
  savePiFig(p,b,STRING("data/onTime-PIP-1.fig"));

  gl.text.clr() <<"onTime my planning";
  do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  unhideAll(p);

  //my planning
  p.timePriorType='d'; p.discount=.9;
  p.iteratedPlan(5,a,b,false);
  savePiFig(p,b,STRING("data/onTime-PIP-09.fig"));

  gl.text.clr() <<"onTime my planning";
  do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  unhideAll(p);
}

void timeWindow(){
  Planner p;
  Planner::node a,b;
  OpenGL gl; gl.add(p.G);

  imageToGraph("mazes/20x10empty.png",p.G);

  a=p.G.nodes(25); b=p.G.nodes(38);
  p.G.del_out_edges(b); p.G.new_edge(b,p.G.first); //let goal lead to trap
  p.normalizeNoisePi();
  
  //my planning
  p.timePriorType='w'; p.discount=1.; p.timeWindowL=0; p.timeWindowH=20;
  p.iteratedPlan(5,a,b,false);
  system(STRING("cp T data/T-window"));
  system(STRING("cp Z data/Z-window"));
  savePiFig(p,b,STRING("data/window.fig"));

  gl.text.clr() <<"window my planning";
  do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  unhideAll(p);
}

void threeHoles(){
  Planner p;
  Planner::node a,b;
  OpenGL gl; gl.add(p.G);
  uint k;

  imageToGraph("mazes/20x15holes.png",p.G);

  a=p.G.nodes(241); b=p.G.nodes(41);
  p.G.del_out_edges(b); p.G.new_edge(b,p.G.first); //let goal lead to trap
  p.normalizeNoisePi();

  //my planning
  p.timePriorType='w'; p.discount=1.; p.timeWindowL=0; p.timeWindowH=200;

  p.installRandomPolicy();
  for(k=0;k<10;k++){
    p.plan(a,b,false); 
    system(STRING("cp T data/T-holes-"<<k));
    system(STRING("cp Z data/Z-holes-"<<k));
    p.calcGvis(); p.layout(-1,0,0,1); gl.text.clr() <<"browse all gamma";
    do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);

    p.installPostPolicyFromPrior(0);
    p.makeMaxPolicy();
    p.makePostToPriorPolicy();
    saveGvisFig(p,b,STRING("data/holes-"<<k<<".fig"));

    gl.text.clr() <<"holes - " <<k;
    do{ gl.display(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
    unhideAll(p);
  }

  p.installRandomPolicy();
  for(k=0;k<10;k++){
    p.plan(a,b,false); 
    system(STRING("cp T data/T-holesX-"<<k));
    system(STRING("cp Z data/Z-holesX-"<<k));
    p.installPostPolicyFromPrior(0);
    p.makeMaxPolicy();
    p.makePostToPriorPolicy();
    savePiFig(p,b,STRING("data/holesX-"<<k<<".fig"));

    gl.text.clr() <<"holes - " <<k;
    do{ gl.display(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
    unhideAll(p);
  }

  //dynamic programming
  p.discount=1.;
  p.installRandomPolicy();
  p.valueIteration(a,b);
  system(STRING("cp DP data/DP-1-holes"));
  p.installMaxQPolicy(); p.layoutPolicy();
  savePiFig(p,b,STRING("data/holes-DP-1.fig"));

  gl.text.clr() <<"holes DP, gamma=1";
  do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  unhideAll(p);

  //dynamic programming
  p.discount=.95;
  p.installRandomPolicy();
  p.valueIteration(a,b);
  system(STRING("cp DP data/DP-095-holes"));
  p.installMaxQPolicy(); p.layoutPolicy();
  savePiFig(p,b,STRING("data/holes-DP-095.fig"));

  gl.text.clr() <<"holes DP, gamma=.95";
  do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  unhideAll(p);
}



void largeMaze(){
  Planner p;
  Planner::node a,b;
  OpenGL gl; gl.add(p.G);

  imageToGraph("mazes/100x100.png",p.G);

  //a=p.G.rndNode(); while(a->Nout<5) a=p.G.rndNode();
  //b=p.G.rndNode(); while(b->Nout<5) b=p.G.rndNode();
  a=p.G.nodes(1540); b=p.G.nodes(7040);
  p.G.del_out_edges(b); p.G.new_edge(b,p.G.first); //let goal lead to trap
  p.normalizeNoisePi();

  p.timePriorType='d'; p.discount=1.; p.timeWindowL=0; p.timeWindowH=200;

#if 0  //my planning
  p.iteratedPlan(6,a,b,true);
  system(STRING("cp PIP data/PIP-large-prune."<<Parameter<uint>("seed")()));

  //p.iteratedPlan(6,a,b,false);
  //system(STRING("cp PIP data/PIP-large-full."<<Parameter<uint>("seed")()));

  //p.calcGvis();
  //saveGvisFig(p,b,STRING("data/PIP-large."<<Parameter<uint>("seed")()<<".fig"));

  //p.layout(-1,0,0,1); gl.text.clr() <<"browse all gamma";
  //do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
#endif

  //dynamic programming
  p.valueIteration(a,b);
  system(STRING("cp VI data/VI-large."<<Parameter<uint>("seed")()));

  p.policyIteration(a,b);
  system(STRING("cp PI data/PI-large."<<Parameter<uint>("seed")()));
  //p.prioritizedSweeping(a,b);
  //p.installMaxQPolicy();
  //savePiFig(p,b,STRING("data/onTime-DP-09.fig"));

  //gl.text.clr() <<"onTime DP, gamma=.9";
  //do{ gl.watch(); if(gl.selected) p.G.writeSelected(gl.selected); }while(gl.selected);
  //unhideAll(p);
}
