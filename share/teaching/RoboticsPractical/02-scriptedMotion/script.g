Script {
  (Control pos endeffL obj1){ target=[0 0 .3], PD=[1., .8, 1., 1.] }
  (Control vecAlign endeffL obj1){ vec1=[1 0 0] vec2=[0 0 -1] target=[1], PD=[1., .8, 1., 1.] }
  (Control qItself){ ref1="l_gripper_l_finger_joint" target=[.1], PD=[1., .8, 1., 1.] }
  wait{ (conv Control pos endeffL obj1) (conv Control vecAlign endeffL obj1) }

  (Control pos endeffL obj1)!, (conv Control pos endeffL obj1)!, (conv Control vecAlign endeffL obj1)!
  (Control pos endeffL obj1){ target=[0 0 .1], PD=[3., 1., 1., 1.] }
  wait{ (conv Control pos endeffL obj1) }

  (Control qItself)!, (conv Control qItself)!
  (Control qItself){ ref1="l_gripper_l_finger_joint" target=[.0], PD=[1., .8, 1., 1.] }
  wait{ (conv Control qItself) }

  (Control pos endeffL obj1)!, (conv Control pos endeffL obj1)!, (conv Control vecAlign endeffL obj1)!
  (Control pos endeffL obj1){ target=[0 0 .3], PD=[3., 1., 1., 1.] }
  wait{ (conv Control pos endeffL obj1) }

  (Control pos endeffL obj1)!, (conv Control pos endeffL obj1)!, (conv Control vecAlign endeffL obj1)!
  (Control pos endeffL obj1){ target=[0 .5 .3], PD=[3., 1., 1., 1.] }
  wait{ (conv Control pos endeffL obj1) }

  (Control qItself)!, (conv Control qItself)!
  (Control qItself){ ref1="l_gripper_l_finger_joint" target=[.1], PD=[1., .8, 1., 1.] }
  wait{ (conv Control qItself) }


#  wait{ (conv Control pos endeffL obj1) }
 # (Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!

  (HomingActivity)
  wait{ (conv HomingActivity) }
}

Script1 {
  (PlayFunnySound)

  (Control pos endeffR base_footprint){ target=[.5, -.5, 1.], PD=[1., .8, 1., 1.] }
  (Control pos endeffL base_footprint){ target=[.5, -.1, 1.], PD=[1., .8, 1., 1.] }
  wait{ (conv Control pos endeffL base_footprint) }
  (Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!

  (PlayFunnySound)!

  (Control pos endeffR base_footprint){ target=[.9, +.1, 1.5], PD=[1., .8, 1., 1.] }
  (Control pos endeffL base_footprint){ target=[.9, +.5, 1.5], PD=[1., .8, 1., 1.] }
  wait{ (conv Control pos endeffR base_footprint) }
  (Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!

  (Control pos endeffR base_footprint){ target=[.5, -.5, 1.], PD=[1., .8, 1., 1.] }
  (Control pos endeffL base_footprint){ target=[.5, -.1, 1.], PD=[1., .8, 1., 1.] }
  wait{ (conv Control pos endeffR base_footprint) }
  (Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!

  (HomingActivity)
  wait{ (conv HomingActivity) }
}
