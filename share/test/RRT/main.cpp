#include <MT/algos.h>
#include <MT/plot.h>
#include <MT/ann.h>

void testRRT(){
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

  ann   [0].append(q_start); //append q as the root of the tree
  parent[0].append(0);    //q has itself as parent
  
  ann   [1].append(q_goal); //append q as the root of the tree
  parent[1].append(0);   //q has itself as parent
  
  plotOpengl();
  plotModule.colors=false;
  //plotGnuplot();

  MT::timerStart();
  for(uint i=0;i<10000;i++){
    //draw random configuration
    if(rnd.uni()<beta) q = q_goal;
    else rndUniform(q,-1.,1.,false);

    for(int fb=0;fb<2;fb++){
      //find NN
      uint k=ann[fb].getNN(q);
      cout <<"\r avg time to get NN = " <<MT::timerRead(false)/(i+1) <<"  points = " <<ann[fb].X.d0;

      //compute little step
      d = q - ann[fb].X[k]; //difference vector between q and nearest neighbor
      qnew = ann[fb].X[k] + stepsize/norm(d) * d;

      //check collision (here: simple shere)
      if(norm(qnew)<.5) collision=true;
      else collision=false;

      //perhaps add to tree
      if(!collision){
	//if(norm(qnew-q_goal)<stepsize){ plot(true); return; }
        ann[fb].append(qnew);
        parent[fb].append(k);
      }

      //plot helper
      if(!collision){
        arr line;
        line.append(ann[fb].X[k]); line.reshape(1,line.N);
        line.append(qnew);
        plotLine(line); //add a line to the plot
        if(!fb && !(i%100))
          plot(false); //update the plot
      }
    }
  }
}


int main(int argn,char** argv){

  testRRT();

  return 0;
}
