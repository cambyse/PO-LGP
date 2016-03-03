#include <Gui/opengl.h>
#include <Perception/videoEncoder.h>
#include "mocapgui.h"

struct sMocapGui: OpenGL::GLKeyCall, OpenGL::GLClickCall {
  ors::KinematicWorld *kw;

  VideoEncoder_x264_simple *vid;
  bool quit, play, screen, record;
  int dir, speed, f, nframes;
  // uintA kflist;

  Graph labelkvg;

  kw_update_function update_custom;
  kw_init_function init_custom;

  MocapRec &rec;

  sMocapGui(MocapRec &rec): OpenGL::GLKeyCall(), vid(nullptr), rec(rec) {}
  virtual ~sMocapGui() { if(kw) delete kw; }

  void initKW(bool keycall, bool clickcall);

  void init();
  void set_target(Target target);
  void update_step();
  void update_poses();
  void update_alpha();
  void finally();

  bool keyCallback(OpenGL &);
  bool clickCallback(OpenGL &);
};

void sMocapGui::init() {
  quit = false;
  play = true;
  record = false;
  screen = false;
  dir = 1;
  speed = 1;
  // if(&_kflist)
  //   kflist = _kflist;
  // else
  //   kflist.resize(0);
  f = 0;
  nframes = rec.numFrames(THICK);

}

void sMocapGui::initKW(bool keycall, bool clickcall) {
  if(!kw) {
    kw = rec.newKW();
    if(keycall)
      kw->gl().addKeyCall(this);
    if(clickcall)
      kw->gl().addClickCall(this);
  }
}

void sMocapGui::set_target(Target target) {
  if(target == TS) {
    double col_on[3][3] = { { 1, 0, 0 },
                            { 0, 1, 0 },
                            { 0, 0, 1 } };
    double col_off[3] = { 1, 1, 1 };
    uint ncolors = sizeof(col_on) / sizeof(col_on[0][0]);

    const StringA &agents = *rec.agent_targets.getValue<StringA>(Target_to_str(target));
    const StringA &objects = *rec.object_targets.getValue<StringA>(Target_to_str(target));
    cout << "agents: " << agents << endl;
    cout << "objects: " << objects << endl;

    uint nagents = agents.N;
    CHECK(nagents <= ncolors, STRING("Not enough colors for " << nagents << " agents; Add more."))

    StringA objectsensors;
    for(const String &obj: objects)
      objectsensors.append(rec.id().sensorsof(obj));
    uint nobjectsensors = objectsensors.N;

    mlr::Array<StringA> agentshapes(nagents);
    for(uint ai = 0; ai < nagents; ai++)
      for(const String &agentsensor: rec.id().sensorsof(agents(ai)))
        agentshapes(ai).append(STRING("sh" << agentsensor));

    arr interactions;
    // cout << "nagents: " << nagents << endl;
    // cout << "nobjectsensors: " << nobjectsensors << endl;
    for(const String &agent: agents) {
      // cout << "========================" << endl;
      // cout << "agent: " << agent << endl;
      for(const String &objsensor: objectsensors) {
        // cout << "objsensor: " << objsensor << endl;
        // cout << "rec.ann.N: " << rec.ann(target, agent, objsensor).N << endl;
        // interactions.append(rec.ann(target, agent, objsensor));
        // interactions.append(*rec.label().getValue<arr>(STRINGS(Target_to_str(target), agent, objsensor)));
        interactions.append(rec.label().getNode({Target_to_str(target), agent, objsensor})->get<arr>());
     // interactions.(*rec.kvgann.getNode(agent, objsensor)->graph().getValue<arr>("ann"));
      }
    }
    interactions.reshape(nagents, nobjectsensors, interactions.N / (nagents * nobjectsensors));

    init_custom = [&, nagents, agentshapes] (ors::KinematicWorld &kw) {
      for(uint ai = 0; ai < nagents; ai++)
        for(const String &agentshape: agentshapes(ai))
          memcpy(kw.getShapeByName(agentshape)->color, col_on[ai], 3 * sizeof(double));
    };

    update_custom = [&, col_on, col_off, nagents, nobjectsensors, agentshapes, objectsensors, interactions] (ors::KinematicWorld &kw, uint f) {
      for(uint osi = 0; osi < nobjectsensors; osi++) {
        for(ors::Shape *sh: kw.getBodyByName(objectsensors(osi))->shapes)
          memcpy(sh->color, col_off, 3 * sizeof(double));
        for(uint ai = 0; ai < nagents; ai++) {
          if(interactions(ai, osi, f)) {
            for(ors::Shape *sh: kw.getBodyByName(objectsensors(osi))->shapes)
              memcpy(sh->color, col_on[ai], 3 * sizeof(double));
            break;
          }
        }
      }
    };
  }
  else if(target == CE) {
    double col_on[3][3] = { { 1, 0, 0 },
                            { 0, 1, 0 },
                            { 0, 0, 1 } };
    double col_off[3] = { 1, 1, 1 };
    uint ncolors = sizeof(col_on) / sizeof(col_on[0][0]);

    const StringA &objects = rec.id().objects();

    uint nobjects = objects.N;
    CHECK(nobjects <= ncolors, STRING("Not enough colors for " << nobjects << " objects; Add more."))

    StringA objectsensors;
    for(const String &obj: objects)
      objectsensors.append(rec.id().sensorsof(obj));
    uint nobjectsensors = objectsensors.N;

    uint ninteractions = nobjectsensors * (nobjectsensors+1) / 2;
    uint nframes = rec.numFrames(THICK);
    arr interactions(ninteractions, nframes);
    interactions.setZero();
    for(uint os1 = 0; os1 < nobjectsensors; os1++)
      for(uint os2 = 0; os2 < os1; os2++) {
        // arr &&inter = rec.ann(target, objectsensors(os1), objectsensors(os2));
        // arr &inter = *rec.label().getValue<arr>(STRINGS(Target_to_str(target), objectsensors(os1), objectsensors(os2)));
        arr &inter = rec.label().getNode({Target_to_str(target), objectsensors(os1), objectsensors(os2)})->get<arr>();
        if(inter.N)
          interactions[os1*(os1+1)/2 + os2]() = inter;
      }
    interactions.reshape(ninteractions, interactions.N / ninteractions);

    init_custom = nullptr;
    update_custom = [&, col_on, nobjectsensors, objectsensors, interactions] (ors::KinematicWorld &kw, uint f) {
      boolA objectcolor(nobjectsensors);
      objectcolor = false;

      for(uint os1 = 0; os1 < nobjectsensors; os1++)
        for(uint os2 = 0; os2 < os1; os2++)
          if(interactions(os1*(os1+1)/2 + os2, f))
            objectcolor(os1) = objectcolor(os2) = true;

      for(uint os = 0; os < nobjectsensors; os++) {
        const double *color = objectcolor(os)? col_on[0]: col_off;
        for(ors::Shape *sh: kw.getBodyByName(objectsensors(os))->shapes)
          memcpy(sh->color, color, 3 * sizeof(double));
      }
    };
  }
}

void sMocapGui::update_step() {
  if(screen) {
    String fname;
    fname << "screenshot." << mlr::getNowString() << ".png";
    kw->gl().saveEPS(fname);
    // byteA img = kw->gl().captureImage;
    // flip_image(img);

    screen = false;
    cout << "Screenshot done!" << endl;
  }
  if(record) {
    // if(play) {
      if(vid == nullptr)
        vid = new VideoEncoder_x264_simple(STRING("z.output." << mlr::getNowString() << ".264"), 120, 0, true);
      flip_image(kw->gl().captureImage);
      vid->addFrame(kw->gl().captureImage);
    // }
  }
  else if(vid != nullptr) {
    vid->close();
    delete vid;
    vid = NULL;
  }
  if(play) f += dir * speed;
  if(f < 0) { f = 0; play = false; }
  if(f >= nframes) { f = nframes - 1; play = false; }
}

void sMocapGui::finally() {
  if(vid != nullptr) {
    vid->close();
    delete vid;
    vid = NULL;
  }
}

#define PREC_POS 1e-2
#define PREC_ORI 1e-1
void sMocapGui::update_poses() {
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

  uint n = kw->getJointStateDimension();
  W.setDiag(1e-4, n);
  for(const String &sensor: rec.id().sensors_unstruct()) {
    // cout << "sensor: " << sensor << endl;
    // cout << rec.query("pos", sensor, f) << endl;
    b = kw->getBodyByName(sensor);
    b->X.pos.set(rec.query("pos", sensor, f).p);
    b->X.rot.set(rec.query("quat", sensor, f).p);
  }
  kw->calc_fwdPropagateShapeFrames();
  if(rec.id().sensors_struct().N) {
    for(;;) {
      Phi.resize(0);
      PhiJ.resize(0);
      kw->getJointState(q);

      for(const String &sensor: rec.id().sensors_struct()) {
        sh = kw->getShapeByName(STRING("sh/task" << sensor));

        sensor_pos.referTo(rec.query("pos", sensor, f));
        sensor_quat.referTo(rec.query("quat", sensor, f));
        quat = ors::Quaternion(sensor_quat);

        kw->kinematicsPos(y, J, sh->body, sh->rel.pos);
        Phi.append((y - sensor_pos) / PREC_POS);
        PhiJ.append(J / PREC_POS);
          
        kw->kinematicsVec(y, J, sh->body, vec_x);
        y_target.setCarray((quat * vec_x).p(), 3);
        Phi.append((y - y_target) / PREC_ORI);
        PhiJ.append(J / PREC_ORI);

        kw->kinematicsVec(y, J, sh->body, vec_y);
        y_target.setCarray((quat * vec_y).p(), 3);
        Phi.append((y - y_target) / PREC_ORI);
        PhiJ.append(J / PREC_ORI);

        kw->kinematicsVec(y, J, sh->body, vec_z);
        y_target.setCarray((quat * vec_z).p(), 3);
        Phi.append((y - y_target) / PREC_ORI);
        PhiJ.append(J / PREC_ORI);
      }
      PhiJT = ~PhiJ;
      dq = inverse(PhiJT * PhiJ + W) * PhiJT * Phi;

      q -= dq;
      kw->setJointState(q);
      kw->calc_fwdPropagateShapeFrames();
      if(absMax(dq) < 1e-2)
        break;
    }
  }

  kw->gl().text.clear() << "frame " << f << "/" << nframes;
  kw->gl().update(NULL, true);
}

void sMocapGui::update_alpha() {
  for(const String sensor: rec.id().sensors())
    for(ors::Shape *sh: kw->getBodyByName(sensor)->shapes)
      sh->color[3] = rec.query("poseObs", sensor, f).scalar();
}

bool sMocapGui::keyCallback(OpenGL&) {
  int df;
  // TODO kw->gl(), but just gl should be fine too
  switch(kw->gl().pressedkey) {
    case 's':
      screen = true;
      break;
    case 'q':
      quit = true;
      break;
    case 'r':
      record ^= true;
      break;
    case ' ':
      play ^= true;
      break;
    case '0':
      dir = -dir;
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      speed = kw->gl().pressedkey - '0';
      break;
    case 'j':
      df = 10*speed;
      if(f < df) f = 0;
      else f -= df;
      play = false;
      break;
    case 'k':
      df = 10*speed;
      if(f >= nframes - df) f = nframes - 1;
      else f += df;
      play = false;
      break;
    // case 'h':
      // if(kflist.N) {
      //   int kfidx;
      //   for(kfidx = kflist.N - 1; kfidx >= 0 && (int)kflist(kfidx) >= f; kfidx--);
      //   if(kfidx >= 0)
      //     f = kflist(kfidx);
      //   play = false;
      // }
      // break;
    // case 'l':
      // if(kflist.N) {
      //   int kfidx;
      //   for(kfidx = 0; kfidx < (int)kflist.N && (int)kflist(kfidx) <= f; kfidx++);
      //   if(kfidx < (int)kflist.N)
      //     f = kflist(kfidx);
      //   play = false;
      // }
      // break;
    // default:
      // cout << "Key not recognized: '" << (char)kw->gl().pressedkey << "' (" << kw->gl().pressedkey << ")" << endl;
  }
  return true;
}

bool sMocapGui::clickCallback(OpenGL &gl) {
  gl.Select();
  uint i = gl.topSelection->name;
  ors::Body *b = nullptr;
  if((i&3)==1)
    b = kw->shapes(i>>2)->body;
  if(b) {
    cout << "clicked on " << b->name << " @ " << f << endl;
    if(!labelkvg.getNode(b->name)) {
      cout << "new label!" << endl;
      // labelkvg.append(b->name, new String("test"));
      labelkvg.append((char*)b->name, new String("test"));
    }
  }
  return true;
}

MocapGui::MocapGui(MocapRec &rec) { s = new sMocapGui(rec); }
MocapGui::~MocapGui() { delete s; }

ors::KinematicWorld &MocapGui::kw() {
  return *s->kw;
}

kw_update_function &MocapGui::update_custom() {
  return s->update_custom;
}

kw_init_function &MocapGui::init_custom() {
  return s->init_custom;
}

void MocapGui::play(Target target) {
  s->initKW(true, false);
  s->init();
  s->set_target(target);
  if(s->init_custom) s->init_custom(kw());
  while(true) {
    s->update_step();
    if(s->quit)
      break;

    s->update_alpha();
    s->update_poses();
    if(s->update_custom) s->update_custom(kw(), s->f);
  }
  s->finally();
}

void MocapGui::annotate(const String &name, uint cardinality) {
  s->initKW(true, true);
  play();
  // TODO process the stuff and create the labeling file
}

void MocapGui::show() {
  s->init();
  if(s->init_custom) s->init_custom(kw());
  while(!s->quit) s->update_step();
  s->finally();
}
