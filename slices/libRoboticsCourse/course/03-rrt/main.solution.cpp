#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/ann.h>
#include <MT/plot.h>

uint MODE=2;

void testRRT(){
  uint dim=2;

  arr q_start(dim);
  arr q(dim),qnew,d;
  bool collision;
  double stepsize = 0.03;

  rndUniform(q_start,-1.,1.,false); //random start point

  q_start = ARR(-.5,-.5);
  
  ANN ann;      //ann also stores all points added to the tree in ann.X
  uintA parent; //for each point we also store the index of the parent node

  ann   .append(q_start); //append q as the root of the tree
  parent.append(0);       //q has itself as parent
  
  plotOpengl();
  plotModule.colors=false;
  //plotGnuplot();

  for(uint i=0;i<100000;i++){
    //draw random configuration
    rndUniform(q,-1.,1.,false);

    //find NN
    uint k=ann.getNN(q);

    //compute little step
    d = q - ann.X[k]; //difference vector between q and nearest neighbor
    qnew = ann.X[k] + stepsize/norm(d) * d;

    //check collision (here: simple shere)
    if(norm(qnew)<.5) collision=true;
    else collision=false;
    //TIP: here you would somehow have to check collision:
    // -- use S.setJointAngles(qnew) to set the configuration (or S.setJointAngles(qnew,false) without display..)
    // -- use S.kinematicsContacts()>.5 check on collision

    //perhaps add to tree
    if(!collision){
      ann.append(qnew);
      parent.append(k);
    }

    //plot helper
    if(!collision){
      arr line;
      line.append(ann.X[k]); line.reshape(1,line.N);
      line.append(qnew);
      plotLine(line); //add a line to the plot
      if(!(i%100)) plot(true); //update the plot
      /*
	TIP: if you want to plot within the simulator window, plot only the endeffector line segments:
arr y_from,y_to;
      arr line;
      S.setJointAngles(ann.X[k],false);  S.kinematicsPos(y_from,"arm7");
      S.setJointAngles(qnew    ,false);  S.kinematicsPos(y_to  ,"arm7");
      line.append(y_from); line.reshape(1,line.N);
      line.append(y_to);
      plotLine(line); //add a line to the plot
      */
    }

    //some output
    cout <<"\rRRT size = " <<ann.X.d0 <<std::flush;

  }
}

void RTTplan(){
  Simulator S("arm7.ors");
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
    //draw random configuration
    rndUniform(q,-1.,1.,false);

    //find NN
    uint k=ann.getNN(q);

    //compute little step
    d = q - ann.X[k]; //difference vector between q and nearest neighbor
    qnew = ann.X[k] + stepsize/norm(d) * d;

    //check collision (here: simple shere)
    S.setJointAngles(qnew,false);
    double y_col = S.kinematicsContacts();
    if(y_col>.5) collision=true; else collision=false;

    //perhaps add to tree
    if(!collision){
      ann.append(qnew);
      parent.append(k);
    }

    //plot helper (makes it slow...)
    if(!collision){
      arr y_from,y_to;
      arr line;
      S.setJointAngles(ann.X[k],false);  S.kinematicsPos(y_from,"arm7");
      S.setJointAngles(qnew    ,false);  S.kinematicsPos(y_to  ,"arm7");
      line.append(y_from); line.reshape(1,line.N);
      line.append(y_to);
      plotLine(line); //add a line to the plot
      //if(!(i%100)) plot(true); //update the plot
    }

    //some output
    if(!(i%100)) S.setJointAngles(qnew); //updates diplay
    cout <<"\rRRT size = " <<ann.X.d0 <<std::flush;
  }
}

int main(int argc,char **argv){
  if(argc>1) MODE = atoi(argv[1]);

  switch(MODE){
  case 1:  testRRT();  break;
  case 2:  RTTplan();  break;
  }

  return 0;
}
