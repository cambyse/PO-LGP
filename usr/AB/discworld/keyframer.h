#ifndef _KEY_FRAMER_
#define _KEY_FRAMER_

#include <Core/array.h>

class KeyFramer {
  private:
    int nbodies, ndofs, agent;
    arr dofs, cumdofs;

    int T;
    arr s;

    int lwin;

    arr err;

  public:
    KeyFramer();
    ~KeyFramer();

    void addBody(int nd);
    int getNBodies();
    int getNDofs(int b);
    int getCumNDofs(int b);
    int agentDofs();

    void clear();
    void clearS();
    void clearT();
    void addState(arr st);
    void addState(arr st, int t);

    int getT();
    arr getState(int t);
    arr getState(int t, int b);
    arr getWindow(int t);
    arr getWindow(int t, int b);

    void setAgent(int a);
    void setLWin(int l);

    void run();
    arr getErr();
    arr getErr(int b);
    void keyframes();

    void test();
};

#endif // _KEY_FRAMER_
