#include <Core/util.h>
#include <Gui/plot.h>
#include <Algo/ann.h>

void TEST(RRT) {
  uint dim=2;

  arr q_start(dim);
  arr q_goal(dim);
  arr q(dim),qnew,d;
  bool collision;
  double stepsize = 0.03;
  double beta = 0; //0.1;

  rndUniform(q_start,-1.,1.,false); //random start point
  rndUniform(q_goal , 1.,1.,false); //random goal  point

  q_start = ARR(-.5,-.5);
  q_goal  = ARR( 1., 1.);
  
  ANN ann[2];      //ann[fb] also stores all points added to the tree in ann[fb].X
  uintA parent[2]; //for each point we also store the index of the parent node

  ann   [0].append(q_start); //append q as the root of the fwd tree
  parent[0].append(0);       //q has itself as parent
  
  ann   [1].append(q_goal);  //append q as the root of the bwd tree
  parent[1].append(0);       //q has itself as parent
  
  plotOpengl();
  plotModule.colors=false;
  //plotGnuplot();

  MT::timerStart();
  for(uint i=0;i<10000;i++){
    //draw random configuration
    if(rnd.uni()<beta) q = q_goal;
    else rndUniform(q,-1.,1.,false);

    for(int fb=0;fb<2;fb++){ //fwd and bwd growth
      //find NN
      uint k=ann[fb].getNN(q);

      //compute little step
      d = q - ann[fb].X[k]; //difference vector between q and nearest neighbor
      qnew = ann[fb].X[k] + stepsize/length(d) * d;

      //check collision (here: simple sphere)
      if(length(qnew)<.5) collision=true;
      else collision=false;

      //perhaps add to tree
      if(!collision){
	//if(length(qnew-q_goal)<stepsize){ plot(true); return; }
        ann[fb].append(qnew);
        parent[fb].append(k);
      }

      //plot helper
      if(!collision){
        arr line;
        line.append(ann[fb].X[k]); line.reshape(1,line.N);
        line.append(qnew);
        plotLine(line); //add a line to the plot
      }

      //output
      if(i<20 || !(i%100)){
        MT::String str;
        str
	  <<"i=" <<i
	  <<"  fwd-tree#=" <<ann[0].X.d0
	  <<"  bwd-tree#=" <<ann[1].X.d0
	  <<endl;
	plot(false,str); //update the plot
	//MT::wait(.1);
      }
    }
  }
}


int MAIN(int argc,char** argv){

  testRRT();

  return 0;
}
