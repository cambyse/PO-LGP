#include <iostream>
#include "keyframe.h"

struct KeyFrame::sKeyFrame {
  uint framenum;
  KeyFrameType type;

  bool hasbodies;
  uint body1, body2;

  double corr;
  uint wlen;

  arr corrs;
  uintA wlens;

  sKeyFrame(uint fn);
  ~sKeyFrame();
};

KeyFrame::sKeyFrame::sKeyFrame(uint fn): framenum(fn),
                                          type(KFT_undef),
                                          hasbodies(false) {}
KeyFrame::sKeyFrame::~sKeyFrame() {}

KeyFrame::KeyFrame(uint fn) {
  s = new sKeyFrame(fn);
}

KeyFrame::~KeyFrame() {
  delete s;
}

void KeyFrame::setBodies(uint b1, uint b2) {
  s->hasbodies = true;
  s->body1 = b1;
  s->body2 = b2;
}

void KeyFrame::setCorr(double c, uint wl) {
  s->type = KFT_corr;
  s->corr = c;
  s->wlen = wl;
}

void KeyFrame::setCorr(arr cs, uintA wls) {
  CHECK(cs.N == wls.N, "Correlations and window lengths not equal?");
  s->type = KFT_corr_ens;
  s->corrs = cs;
  s->wlens = wls;
}

bool operator<(const KeyFrame &kf1, const KeyFrame &kf2) {
  return kf1.s->framenum < kf2.s->framenum;
}

std::ostream &operator<<(std::ostream &os, const KeyFrame &kf) {
  KeyFrame::sKeyFrame *s = kf.s;
  os << "KeyFrame {\n";
  os << "  frame:    " << s->framenum << "\n";
  if(s->hasbodies)
    os << "  bodies:    " << s->body1 << " " << s->body2 << "\n";
  switch(kf.s->type) {
    case KFT_undef:
      os << "  type:     undefined\n";
      break;
    case KFT_corr:
      os << "  type:     correlation\n";
      os << "  corr:     " << s->corr << "\n";
      os << "  wlen:     " << s->wlen << "\n";
      break;
    case KFT_corr_ens:
      os << "  type:     correlation-ensemble\n";
      os << "  tot_corr: " << product(s->corrs) << "\n";
      os << "  corr:     " << s->corrs << "\n";
      os << "  wlen:     " << s->wlens << "\n";
      break;
    default:
      HALT("KeyFrame type unhandles?!");
  }
  os << "}" << endl;
  return os;
}

