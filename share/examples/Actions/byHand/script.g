cleanAll

Script {
  (Control wheels){ target=[0, .3, .2], PD=[.5, .9, .5, 10.]}
  (Control pos endeffR base_footprint){ target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}
  (Control pos endeffL base_footprint){ target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}
  { (conv Control wheels)  (conv Control pos endeffR base_footprint) }
  (cleanAll)
  (cleanAll)!

  (Control wheels){ target=[0, -.3, -.2], PD=[.5, .9, .5, 10.]}
  (Control pos endeffR base_footprint){ target=[.7, -.2, .7], PD=[.5, .9, .5, 10.]}
  (Control pos endeffL base_footprint){ target=[.7, +.2, .7], PD=[.5, .9, .5, 10.]}
  { (conv Control wheels)  (conv Control pos endeffL base_footprint) }
  (cleanAll)
  (cleanAll)!

  (Control wheels){ target=[0, .3, .2], PD=[.5, .9, .5, 10.]}
  (Control pos endeffR base_footprint){ target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}
  (Control pos endeffL base_footprint){ target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}
  { (conv Control pos endeffL base_footprint) }
  (cleanAll)
  (cleanAll)!

  (Control wheels){ target=[0, 0, 0], PD=[.5, .9, .5, 10.]}
  (HomingActivity)
  { (conv HomingActivity) (conv Control wheels) }
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
