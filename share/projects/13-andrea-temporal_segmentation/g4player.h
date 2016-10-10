#pragma once

#include <Perception/g4data.h>

// =============================================================================
// G4Player
//

#include <Ors/ors.h>
struct G4Player {
  struct sG4Player {
    ors::KinematicWorld *kw;

    sG4Player(): kw(nullptr) {}
    ~sG4Player() {}
  };

  sG4Player *s;

  G4Player() { s = new sG4Player(); }
  ~G4Player() { delete s; }
  
  ors::KinematicWorld &kw() {
    if(!s->kw) s->kw = new ors::KinematicWorld();
    return *s->kw;
  }

#define PREC_POS 1e-2
#define PREC_ORI 1e-1
  void updateKW(G4Rec &g4rec, uint f) {
    // thinds I need in here:
    //  * g4id
    //  * g4rec
    //  * kw
    arr sensor_pos, sensor_quat;
    ors::Vector vec_x = Vector_x,
                vec_y = Vector_y,
                vec_z = Vector_z;
    ors::Quaternion quat;
    ors::Body *b;
    ors::Shape *sh;

    arr y, J, yVec, JVec;
    arr Phi, PhiJ, PhiJT;
    arr q, dq;
    arr y_target, W;

    uint n = kw().getJointStateDimension();
    W.setDiag(1e-4, n);
    for(const String &sensor: g4rec.id().unstruct_sensors()) {
      b = kw().getBodyByName(sensor);
      b->X.pos.set(g4rec.query("pos", sensor, f).p);
      b->X.rot.set(g4rec.query("quat", sensor, f).p);
    }
    kw().calc_fwdPropagateShapeFrames();
    if(g4rec.id().struct_sensors().N == 0)
      return;
    for(;;) {
      Phi.resize(0);
      PhiJ.resize(0);
      kw().getJointState(q);

      for(const String &sensor: g4rec.id().struct_sensors()) {
        sh = kw().getShapeByName(STRING("sh:task:" << sensor));

        sensor_pos.referTo(g4rec.query("pos", sensor, f));
        sensor_quat.referTo(g4rec.query("quat", sensor, f));
        quat = ors::Quaternion(sensor_quat);

        kw().kinematicsPos(y, J, sh->body, &sh->rel.pos);
        Phi.append((y - sensor_pos) / PREC_POS);
        PhiJ.append(J / PREC_POS);
          
        kw().kinematicsVec(y, J, sh->body, &vec_x);
        y_target.setCarray((quat * vec_x).p(), 3);
        Phi.append((y - y_target) / PREC_ORI);
        PhiJ.append(J / PREC_ORI);

        kw().kinematicsVec(y, J, sh->body, &vec_y);
        y_target.setCarray((quat * vec_y).p(), 3);
        Phi.append((y - y_target) / PREC_ORI);
        PhiJ.append(J / PREC_ORI);

        kw().kinematicsVec(y, J, sh->body, &vec_z);
        y_target.setCarray((quat * vec_z).p(), 3);
        Phi.append((y - y_target) / PREC_ORI);
        PhiJ.append(J / PREC_ORI);
      }
      PhiJT = ~PhiJ;
      dq = inverse(PhiJT * PhiJ + W) * PhiJT * Phi;

      q -= dq;
      kw().setJointState(q);
      kw().calc_fwdPropagateShapeFrames();
      if(absMax(dq) < 1e-2)
        break;
    }
    // TODO remove computeProxies?
    /* kw().computeProxies(); */
    // if(show) {
    //   kw().gl().text.clear() << "frame " << f << "/" << g4d().numFrames();
    //   kw().gl().update(NULL, true);
    // }
  }

  void play(G4Rec &g4rec) {
    kw() = ors::KinematicWorld(STRING(g4rec.dir << "world.ors"));

    ors::Body *b = new ors::Body(kw());
    ors::Shape *sh = new ors::Shape(kw(), *b);
    sh->type = ors::markerST;
    sh->size[0] = .5;

    uint nframes = g4rec.numFrames();
    for(uint f = 0; f < nframes; f += 2) {
    // for(uint f = 0; f < 1000; f++) {
      updateKW(g4rec, f);
      kw().gl().text.clear() << "frame " << f << "/" << nframes;
      kw().gl().update(NULL);
    }
  }

  void play(G4Rec &g4rec, const char *sens1, const char *sens2, const arr &interaction) {
    // TODO we have a problem here: the output of the model is downsampled..
    // To solve, you pretty much need some method to inflate the model's output
    // so that it gains the length of the original sequence..
    // Probably do this within the model itself..
    // Add some type of argument which selects which output you want, and default it to the restored one.
    // TODO
    // possibly color according to some output

    // TODO think about the player interface..
    // Maybe give it a model directly?
    // probably the best way of action
    double col_on[3] = { 1, 0, 0 };
    double col_off[3] = { 1, 1, 1 };

    kw() = ors::KinematicWorld(STRING(g4rec.dir << "world.ors"));
    uint nframes = g4rec.numFrames();
    for(uint f = 0; f < nframes; f++) {
      for(ors::Shape *sh: kw().shapes)
        memcpy(sh->color, col_off, 3*sizeof(double));
      updateKW(g4rec, f);

      memcpy(kw().getShapeByName(sens1)->color, col_on, 3 * sizeof(double));
      if(interaction(f))
        memcpy(kw().getShapeByName(sens2)->color, col_on, 3 * sizeof(double));

      kw().gl().text.clear() << "frame " << f << "/" << nframes;
      kw().gl().update(NULL, true);
    }
  }
  // TODO previous implementation of play
  // previous impl didn't have inputs.. So it automatically did it for all pairs..
  // which might be a thing we'd like to have as an option
  //
  // double col_on[3] = { 1, 0, 0 };
  // double col_off[3] = { 1, 1, 1 };

  // for(uint f = 0; f < g4d().numFrames(); f++) {
  //   for(ors::Shape *sh: kw().shapes)
  //     memcpy(sh->color, col_off, 3*sizeof(double));
  //   for(uint i1 = 0; i1 < g4d().id().sensors().N; i1++) {
  //     for(uint i2 = i1+1; i2 < g4d().id().sensors().N; i2++) {
  //       String &s1 = g4d().id().sensors().elem(i1);
  //       String &s2 = g4d().id().sensors().elem(i2);
  //       if(annOf(s1, s2).elem(f) == 1) {
  //         for(ors::Shape *sh: kw().getBodyByName(s1)->shapes)
  //           memcpy(sh->color, col_on, 3*sizeof(double));
  //         for(ors::Shape *sh: kw().getBodyByName(s2)->shapes)
  //           memcpy(sh->color, col_on, 3*sizeof(double));
  //       }
  //     }
  //   }
  //   updateOrs(f, true);
  // }
};

