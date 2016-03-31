Script {
  (PlayFunnySound)

  (Control pos endeffR base_footprint){ target=[.5, -.5, 1.], PD=[1., .8, 1., 1.]}
  (Control pos endeffL base_footprint){ target=[.5, -.1, 1.], PD=[1., .8, 1., 1.]}
  wait{ (conv Control pos endeffL base_footprint) }
  (Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!

  (PlayFunnySound)!

  (Control pos endeffR base_footprint){ target=[.9, +.1, 1.5], PD=[1., .8, 1., 1.]}
  (Control pos endeffL base_footprint){ target=[.9, +.5, 1.5], PD=[1., .8, 1., 1.]}
  wait{ (conv Control pos endeffR base_footprint) }
  (Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!

  (Control pos endeffR base_footprint){ target=[.5, -.5, 1.], PD=[1., .8, 1., 1.]}
  (Control pos endeffL base_footprint){ target=[.5, -.1, 1.], PD=[1., .8, 1., 1.]}
  wait{ (conv Control pos endeffR base_footprint) }
  (Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!

  (Control gazeAt endeffKinect endeffR)!

  (HomingActivity)
  wait{ (conv HomingActivity) }
}
