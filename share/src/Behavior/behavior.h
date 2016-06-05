#pragma once

struct Handle;
struct ControlHandle;

struct Behavior{

  //-- generic
  Handle* start(const Graph& specs);
  Handle* modify(Handle* t, const Graph& specs);
  void stop(const HandleL& tasks);

  //-- low-level motion contol
  ControlHandle* control(const Graph& specs);
  ControlHandle* modifyTarget(ControlHandle* t, const arr& target);
  arr getState(ControlHandle* t);
  void waitConv(const ControlHandleL& tasks);

  //-- low-level motion contol




};

