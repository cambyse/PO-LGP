#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/ann.h>
#include <MT/plot.h>



void RTTplan(){
  //Simulator S("arm7.ors");
  Simulator S("../02-pinInAHole/pin_in_a_hole.ors");

  arr q;
  S.getJointAngles(q);

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr q_start = q;
  arr qnew,d;
  bool collision;
  double stepsize = 0.1;

  ANN ann;      //ann also stores all points added to the tree in ann.X
  uintA parent; //for each point we also store the index of the parent node

  ann   .append(q_start); //append q as the root of the tree
  parent.append(0);    //q has itself as parent
  
  plotModule.colors=false;

  for(uint i=0;i<100000;i++){
    //draw random target configuration
    rndUniform(q,-1.,1.,false);

    //find NN
    uint k=ann.getNN(q);

    //compute little step
    d = q - ann.X[k]; //difference vector between q and nearest neighbor
    qnew = ann.X[k] + stepsize/norm(d) * d;

    //check collision
    S.setJointAngles(qnew,false);
    arr y_col;
    S.kinematicsContacts(y_col);
    if(y_col(0)>.5) collision=true; else collision=false;

    //perhaps add to tree
    if(!collision){
      ann.append(qnew);
      parent.append(k);
    }

    //plot helper (makes it slow...)
    if(!collision){
      //I can't draw the edge in the 7-dim joint space!
      //But I can draw a projected edge in 3D endeffector position space:
      arr y_from,y_to;
      arr line;
      S.setJointAngles(ann.X[k],false);  S.kinematicsPos(y_from,"pin");
      S.setJointAngles(qnew    ,false);  S.kinematicsPos(y_to  ,"pin");
      line.append(y_from); line.reshape(1,line.N);
      line.append(y_to);
      plotLine(line); //add a line to the plot
    }

    //some output
    if(!(i%1000)) S.setJointAngles(qnew); //updates diplay (makes it slow)
    cout <<"\rRRT size = " <<ann.X.d0 <<std::flush;
  }
}

int main(int argc,char **argv){
  uint mode=1;
  if(argc>1) mode = atoi(argv[1]);
  switch(mode){
  case 1:  RTTplan();  break;
  }

  return 0;
}
