#pragma once

enum ActStatus { AS_preStart=-1, AS_running, AS_done, AS_stalled, AS_converged };

struct Act{
  double startTime;

  Act();
  virtual ~Act(){}

  virtual void start() = 0;
  virtual void stop() = 0;


  virtual ActStatus status() = 0;
  double time();
};

