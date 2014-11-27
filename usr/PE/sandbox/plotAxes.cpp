void drawFrame(void* classP){
  arr *p = (arr*)classP;
//  arr p = *p;
  glColor((*p)(2,0));
  glPointSize(4.0f);
  glLineWidth(6);
  glBegin(GL_LINES);
  cout << (*p)(0,0) << endl;
  glVertex3f((*p)(0,0),(*p)(0,1),(*p)(0,2));
  glVertex3f((*p)(1,0),(*p)(1,1),(*p)(1,2));
  glEnd();
  glLineWidth(1);
}



 ors::Vector p = world.getShapeByName("r_ft_sensor")->X.pos;
  ors::Vector xV = ors::Vector(1.,0.,0.); //world.getShapeByName("r_ft_sensor")->X.rot.getX();
  ors::Vector yV = ors::Vector(0.,1.,0.); //world.getShapeByName("r_ft_sensor")->X.rot.getY();
  ors::Vector zV = ors::Vector(0.,0.,1.); //world.getShapeByName("r_ft_sensor")->X.rot.getZ();

//  world.kinematicsVec(x,NoArr,world.getShapeByName("r_ft_sensor")->body,&xV);
//  world.kinematicsVec(y,NoArr,world.getShapeByName("r_ft_sensor")->body,&yV);
//  world.kinematicsVec(z,NoArr,world.getShapeByName("r_ft_sensor")->body,&zV);
  ors::Vector xx = world.getShapeByName("r_ft_sensor")->X.rot.getX();
  ors::Vector yy = world.getShapeByName("r_ft_sensor")->X.rot.getY();
  ors::Vector zz = world.getShapeByName("r_ft_sensor")->X.rot.getZ();

  cout << "x Vector: " << x << endl;
  cout << "y Vector: " << y << endl;
  cout << "z Vector: " << z << endl;
  // X,Y,Z - R,G,B
  arr draw1 = cat(~ARRAY(p),~ARRAY(p+0.3*xx),~ARRAY(2.,0.,0.));
  arr draw2 = cat(~ARRAY(p),~ARRAY(p+0.3*yy),~ARRAY(5.,0.,0.));
  arr draw3 = cat(~ARRAY(p),~ARRAY(p+0.3*zz),~ARRAY(0.,0.,0.));
  cout << draw1 << endl;
  world.gl().add(drawFrame,&draw1);
  world.gl().add(drawFrame,&draw2);
  world.gl().add(drawFrame,&draw3);


  ors::Vector p2 = world.getBodyByName("r_wrist_roll_link")->X.pos;

  world.kinematicsVec(x,NoArr,world.getBodyByName("r_wrist_roll_link"),&xV);
  world.kinematicsVec(y,NoArr,world.getBodyByName("r_wrist_roll_link"),&yV);
  world.kinematicsVec(z,NoArr,world.getBodyByName("r_wrist_roll_link"),&zV);

  // X,Y,Z - R,G,B
  arr draw4 = cat(~ARRAY(p2),~ARRAY(p2+0.3*x),~ARRAY(2.,0.,0.));
  arr draw5 = cat(~ARRAY(p2),~ARRAY(p2+0.3*y),~ARRAY(5.,0.,0.));
  arr draw6 = cat(~ARRAY(p2),~ARRAY(p2+0.3*z),~ARRAY(0.,0.,0.));
  world.gl().add(drawFrame,&draw4);
  world.gl().add(drawFrame,&draw5);
  world.gl().add(drawFrame,&draw6);
