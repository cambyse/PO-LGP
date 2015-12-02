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


void reach(){
  Simulator S("man.ors");
  arr q,W;
  uint n = S.getJointDimension();
  S.getJointAngles(q);
  double sigma = 1e-2;
  W.setDiag(sigma*sigma,n);  //W is equal the Id_n matrix times scalar w

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr y_target,y,J;

  uint numberOfSteps = 100;

  arr e;
  double alpha = 0.1;
  for(uint i=0; i < numberOfSteps; i++) {
    //1st task:
    y_target = {-0.2, -0.4, 1.1};
    S.kinematicsPos(y,"handR");  // gets the feature vector where "handR" is the name of the right hand
    S.jacobianPos  (J,"handR");  // gets the corresponding Jacobian matrix

    e = (y_target - y);
    cout << sqrt(~e*e) << endl;

    //compute joint updates
    q += alpha*inverse(~J*J + W)*~J*e;   //NOTATION: ~J is the transpose of J

    //sets joint angles AND computes all frames AND updates display
    S.setJointAngles(q);

    //optional: pause and watch OpenGL
    //S.watch();
    mlr::wait(0.01);
  }
}

void reach2(){
  Simulator S("man.ors");
  arr q,W;
  uint n = S.getJointDimension();
  S.getJointAngles(q);
  double sigma = 1e-2;
  W.setDiag(sigma*sigma,n);  //W is equal the Id_n matrix times scalar w

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr y_target,y,J;

  uint numberOfSteps = 100;

  arr e, yIntermediate, y0;
  S.kinematicsPos(y0,"handR");
  yIntermediate = y0;

  for(uint i=0; i < numberOfSteps; i++) {
    //1st task:
    y_target = {-0.2, -0.4, 1.1};
    S.kinematicsPos(y,"handR");  // gets the feature vector where "handR" is the name of the right hand
    S.jacobianPos  (J,"handR");  // gets the corresponding Jacobian matrix

    yIntermediate += 1.0/numberOfSteps*(y_target - y0);

    e = (yIntermediate - y);
    cout << sqrt(~e*e) << endl;

    //compute joint updates
    q += inverse(~J*J + W)*~J*e;   //NOTATION: ~J is the transpose of J

    //sets joint angles AND computes all frames AND updates display
    S.setJointAngles(q);

    //optional: pause and watch OpenGL
    //S.watch();
    mlr::wait(0.01);
  }
}

arr generateCircleTrajectory(uint numberOfPoints, uint numberOfQuarterCircles, arr middlePoint, double radius) {
  arr trajectory = zeros(numberOfPoints*numberOfQuarterCircles,3);
  for(uint i = 0; i < numberOfPoints*numberOfQuarterCircles; i++) {
    trajectory[i] = radius * ARR(cos((double)i/numberOfPoints), 0, sin((double)i/numberOfPoints)) + middlePoint;
  }
  return trajectory;
}

/* takes a trajectory in task space, interpolates linearly between them
 * and does the inverse kinematics
 */
void executeTrajectory(arr trajectory, uint numberOfSteps) {
  Simulator S("man.ors");
  arr q,W;
  uint n = S.getJointDimension();
  S.getJointAngles(q);
  double sigma = 1e-2;
  W.setDiag(sigma*sigma,n);  //W is equal the Id_n matrix times scalar w

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr y_target,y,J;

  arr e, yIntermediate, y0;

  for(uint k = 0; k < trajectory.d0-1; k++) {
    y0 = trajectory[k];
    yIntermediate = y0;
    for(uint i=0; i < numberOfSteps; i++) {
      y_target = trajectory[k+1];

      S.kinematicsPos(y,"handR");  // gets the feature vector where "handR" is the name of the right hand
      S.jacobianPos  (J,"handR");  // gets the corresponding Jacobian matrix

      yIntermediate += 1.0/numberOfSteps*(y_target - y0);

      e = (yIntermediate - y);
      cout << sqrt(~e*e) << endl;

      //compute joint updates
      q += inverse(~J*J + W)*~J*e;   //NOTATION: ~J is the transpose of J

      //sets joint angles AND computes all frames AND updates display
      S.setJointAngles(q);

      //optional: pause and watch OpenGL
      //S.watch();
      //mlr::wait(0.01);
    }
  }
}

void circle() {
  arr trajectory = generateCircleTrajectory(100,10,ARR(-0.2,-0.4,1.1),0.2);
  executeTrajectory(trajectory,1);
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  switch(mlr::getParameter<int>("mode",4)){
  case 0:  simpleArrayOperations();  break;
  case 1:  openingSimulator();  break;
  case 2:  reach();  break;
  case 3:  reach2();  break;
  case 4:  circle();  break;
  }

  return 0;
}
