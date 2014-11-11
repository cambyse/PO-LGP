#include <Ors/roboticsCourse.h>


void simpleArrayOperations(){
  arr x;
  x = ARR(.1, .2, .3);         //directly setting the array
  cout <<"x = " <<x <<endl;

  x += ARR(2., 2., 2.);        //adding to an array
  cout <<"x = " <<x <<endl;

  x *= 1.;
  cout <<"x = " <<x <<endl;

  arr y = ARR(-.3,-.2,-.1);
  y.append(x);                 //appending a vector to a vector
  cout <<"y = " <<y <<endl;

  arr M(4,3);                  //some 3 x 4 matrix
  M[0] = ARR(1, 0, 0);         //setting the first row
  M[1] = ARR(0, 1, 0);
  M[2] = ARR(0, 0, 1);
  M[3] = ARR(1, 0, 0);

  cout <<"M =\n" <<M <<endl;

  cout <<"transpose M =\n" <<~M <<endl;  //matrix transpose

  cout <<"M*x = " <<M*x <<endl;          //matrix-vector product

  cout <<"M^-1 =\n" <<inverse(M) <<endl; //matrix inverse

  M.append(M);                 //appending a matrix to a matrix
  cout <<"M =\n" <<M <<endl;
}


void openingSimulator(){
  Simulator S("man.ors");
  cout <<"joint dimensions = " <<S.getJointDimension() <<endl;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr q;
  S.getJointAngles(q);

  q(0) += 0.1;                 //change the first entry of q-vector
  S.setJointAngles(q);
  S.watch();
  
  q = 0.;                      //set q-vector equal zero
  S.setJointAngles(q);
  S.watch();
}

////////////////////////////////////////////////////////////////////////////////

void reach_problem(){
  Simulator S("man.ors");
  arr q,W;
  uint n = S.getJointDimension();
  S.getJointAngles(q);
  double w = MT::getParameter("w",1e-4);
  W.setDiag(w,n);  //W is equal the Id_n matrix times scalar w

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr y_target,y,J;
  for(uint i=0;i<10;i++){
    //1st task:
    y_target = {-0.2, -0.4, 1.1}; 
    S.kinematicsPos(y,"handR");  //"handR" is the name of the right hand ("handL" for the left hand)
    S.jacobianPos  (J,"handR");

    //compute joint updates
    q += inverse(~J*J + W)*~J*(y_target - y); 
    //NOTATION: ~J is the transpose of J
    
    //sets joint angles AND computes all frames AND updates display
    S.setJointAngles(q);

    //optional: pause and watch OpenGL
    S.watch();
  }
}

////////////////////////////////////////////////////////////////////////////////

void reach_sol(){
  Simulator S("man.ors");
  arr q,W;
  uint n = S.getJointDimension();
  S.getJointAngles(q);
  double w = MT::getParameter("w",1e-4);
  W.setDiag(w,n);  //W is equal the Id_n matrix times scalar w

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr y_target,y,y0,yT,J;
  yT = {-0.2, -0.4, 2.1};
  S.kinematicsPos(y0,"handR");
  uint T=10;
  for(uint t=0;t<=T+10;t++){
    //1st task:
    if(t<T) y_target = y0 + ((double)t/T)*(yT-y0); 
    else    y_target = yT;
    S.kinematicsPos(y,"handR");  //"handR" is the name of the right hand ("handL" for the left hand)
    S.jacobianPos  (J,"handR");

    //report on error in the first task
    cout <<t <<" current eff pos = " <<y <<"  current error = " <<length(y_target-y) <<endl;

    //compute joint updates
    q += inverse(~J*J + W)*~J*(y_target - y); 
    //NOTATION: ~J is the transpose of J
    
    //sets joint angles AND computes all frames AND updates display
    S.setJointAngles(q);

    //optional: pause and watch OpenGL
    S.watch();
  }
}

////////////////////////////////////////////////////////////////////////////////

void circle(){
  Simulator S("man.ors");
  arr q,W;
  uint n = S.getJointDimension();
  S.getJointAngles(q);
  W.setDiag(1.,n);

  arr y_target,y,J;
  for(uint i=0;i<1000;i++){
    y_target = ARR(-0.2, -0.4, 1.1); 
    y_target += .2 * ARR(cos((double)i/20), 0, sin((double)i/20)); 
    S.kinematicsPos(y,"handR");
    S.jacobianPos  (J,"handR");
    cout <<i <<" current eff pos = " <<y <<"  current error = " <<length(y_target-y) <<endl;;
    q += inverse(~J*J + 1e-4*W)*~J*(y_target - y); 
    S.setJointAngles(q);
  }
  S.watch();
}

////////////////////////////////////////////////////////////////////////////////

void circle2(){
  Simulator S("man.ors");
  arr q,q_home,y_target,y,J,W,Phi,PhiJ;
  uint n = S.getJointDimension();
  S.getJointAngles(q);
  W.setDiag(1.,n); //we define W as identity matrix

  q_home = q;
  for(uint i=0;i<10000;i++){
    Phi.clear();
    PhiJ.clear();

    //1st task: track with right hand
    y_target = ARR(-0.2, -0.4, 1.1);
    y_target += .2 * ARR(cos((double)i/20), 0, sin((double)i/20));
    S.kinematicsPos(y,"handR");
    S.jacobianPos (J,"handR");
    Phi .append( 1e2 * (y - y_target) ); //rho = 1e4 (cp. slide 02:45)
    PhiJ.append( 1e2 * J );

    //add the "stay close to home" task here

    //add another task for the left hand here, if you like

    //compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi; //(cp. slide 02:46)
    S.setJointAngles(q);
  }
}

////////////////////////////////////////////////////////////////////////////////

void multiTask(){
  Simulator S("man.ors");
  arr q,y_target,y,J,W,Phi,PhiJ;
  uint n = S.getJointDimension();

  S.getJointAngles(q);
  W.setDiag(1.,n);

  for(uint i=0;i<10000;i++){
    //1st task: track with right hand
    y_target = ARR(-0.2, -0.4, 1.1); 
    y_target += .2 * ARR(cos((double)i/20), 0, sin((double)i/20)); 
    S.kinematicsPos(y,"handR");
    S.jacobianPos  (J,"handR");

    Phi  = (y - y_target)/1e-2;
    PhiJ = J / 1e-2;

    //2nd task: joint should stay close to zero
    y = q;
    y_target.resizeAs(q);
    y_target.setZero();
    J.setDiag(1.,n);

    Phi .append((y - y_target)/1e0);
    PhiJ.append( J / 1e0 );

    //3rd task: track with left hand
    y_target = ARR( 0.2, -0.4, 1.1); 
    y_target += .2 * ARR(cos((double)i/30), 0, sin((double)i/30)); 
    S.kinematicsPos(y,"handL");
    S.jacobianPos  (J,"handL");

    Phi .append((y - y_target)/1e-2);
    PhiJ.append( J / 1e-2 );

    //compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    S.setJointAngles(q);
  }
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  switch(MT::getParameter<int>("mode",2)){
  case 0:  simpleArrayOperations();  break;
  case 1:  openingSimulator();  break;
  case 2:  reach_problem();  break;
  case 3:  reach_sol();  break;
  case 4:  circle2();  break;
  case 5:  multiTask();  break;
  }

  return 0;
}
