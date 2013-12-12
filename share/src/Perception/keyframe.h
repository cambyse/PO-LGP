#pragma once
#include <Core/array.h>

struct KeyFrame;
typedef MT::Array<KeyFrame*> KeyFrameL;

enum KeyFrameType {
  KFT_undef = -1,
  KFT_corr = 0,
  KFT_corr_ens,
};

struct KeyFrame {
  struct sKeyFrame;
  sKeyFrame *s;

  KeyFrame(uint fn);
  ~KeyFrame();

  void setBodies(uint b1, uint b2);
  void setCorr(double c, uint wl);
  void setCorr(arr cs, uintA wls);

  uint getFrame();

  friend bool operator<(const KeyFrame &kf1, const KeyFrame &kf2);
  friend std::ostream &operator<<(std::ostream &os, const KeyFrame &kf);
};

