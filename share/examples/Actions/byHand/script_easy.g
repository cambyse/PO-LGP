cleanAll

Script {
  (Control gazeAt endeffKinect r_gripper_palm_link_0){ PD=[.1, .9, .5, 10.], prec=10 }
  (Control pos endeffR base_footprint){ target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}
  (Control pos endeffL base_footprint){ target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}
  { (conv Control pos endeffR base_footprint) }
  (cleanAll)
  (cleanAll)!

  (HomingActivity)
  { (conv HomingActivity) }
}


Rule {
    X, Y, Z,
    { (cleanAll) (conv Control X Y Z) }
    { (conv Control X Y Z)! }
}

Rule {
    X,
    { (cleanAll) (conv Control X) }
    { (conv Control X)! }
}

Rule {
    X, Y, Z,
    { (cleanAll) (Control X Y Z) }
    { (Control X Y Z)! }
}

Rule {
    X, 
    { (cleanAll) (Control X) }
    { (Control X)! }
}
