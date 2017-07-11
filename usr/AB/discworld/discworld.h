#ifndef _DISC_WORLD_
#define _DISC_WORLD_

#include <Kin/kin_physx.h>
#include <Gui/opengl.h>
#include <Plot/plot.h>

#include <Perception/keyframer.h>

class DiscWorld {
  public:
    static float const HEIGHT;
    static float const GOAL_HEIGHT;
    static float const RADIUS;

    mlr::KinematicWorld *ors;
    OpenGL *gl;
    PhysXInterface *physx;

    bool mode;
    int N, T, G;
    mlr::Array<mlr::Vector> bodies;
    mlr::Array<mlr::Vector> goals;
    StringL names;

    KeyFramer kf;

    float speed;
    int lwin;

  public:
    DiscWorld();
    ~DiscWorld();

    void clear();

    void setTMode(int t = 0);
    void setGMode();

    void addBody(int n = 1);
    bool addBody(float x, float y);
    bool addBody(const mlr::Vector &p);

    void addGoal(int n);
    void addGoal(float x, float y);
    void addGoal(const mlr::Vector &p);

    void setSpeed(float s);
    void setNBodies(int n);
    void setLWin(int l);

    void play();
    void replay();

  private:
    void initBodies();
    void resetBodies();

    void clearGui();

    void setGoal(mlr::Body *goal, int g);
    bool move(mlr::Body *agent, mlr::Body *goal);
    void recStep(int t);
    void playStep(int t);
};

#endif // _DISC_WORLD_
