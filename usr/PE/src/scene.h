#ifndef SCENE_H
#define SCENE_H

#include <Motion/motion.h>
#include "cost_weight.h"

struct Scene {
  /// scene description
  arr x0;
  MotionProblem* MP;
  mlr::KinematicWorld* world;

  /// options
  bool optConstraintsParam;
  bool optNonlinearParam;

  /// variables for the inverse motion problem
  arr PHI, G, lambda;
  arr JxP, JgP, Jx, Jg;
  arr Jg_JgtP, J_Jgt, Jg_Jgt,Jg_JgtI;
  arr dWdx_dPHI_J_G_Jt_dPHI_dWdx, dPHI_J_Jt_dPHI, Jgt_JgJgtI_Jg;

  /// demonstrations
  arr xDem;
  arr lambdaDem;

  void initCosts(bool _optNonlinearParam);
  double compCosts(arr& df, arr& Hf,arr& g, arr& Jg, const arr& w, const arr &dw, const arr &Hw);
};

struct Scenario {
  mlr::Array<Scene > scenes;
  mlr::Array<CostWeight> weights;
  arr paramGT; // ground truth parameters
  double costScale;

  void setParam(arr param,bool reset = false) {
    arr paramNorm = param;
    for (uint i = 0; i<scenes.d0;i++){
      uint pc = 0;
      for (uint c=0;c<scenes(i).MP->tasks.N;c++) {
        if (scenes(i).MP->tasks(c)->map.type == OT_sumOfSqr) {
          arr w;
          weights(c).compWeights(w,NoArr,NoArr,paramNorm.subRange(pc,pc+weights(c).numParam - 1),true);
          if (reset) w = 1.;
          if (weights(c).type==CostWeight::Block){
            scenes(i).MP->tasks(c)->prec.resize(weights(c).fixedParam(1)+1).setZero();
            scenes(i).MP->tasks(c)->prec.subRange(weights(c).fixedParam(0),weights(c).fixedParam(1)) = w;
          }else if (weights(c).type==CostWeight::RBF) {
            scenes(i).MP->tasks(c)->prec = w;
          } else {
            scenes(i).MP->tasks(c)->prec = w;
          }

//          mlr::String name("plots/p");
//          name << c;
//          FILE(name) <<weights(c).type;
//          name << "param";
//          write(LIST<arr>(paramNorm.subRange(pc,pc+weights(c).numParam - 1)),name);
//          name << "fixed";
//          write(LIST<arr>(weights(c).fixedParam),name);
//          cout << scenes(i).MP->tasks(c)->prec << endl;
          pc += weights(c).numParam;

          // make sure all weights are above some threshold
          for (uint k=0;k<scenes(i).MP->tasks(c)->prec.d0;k++){
            if (scenes(i).MP->tasks(c)->prec(k) < 1e-3 && c>0) {
              scenes(i).MP->tasks(c)->prec(k) = 0.;
            }
          }
//          cout << scenes(i).MP->tasks(c)->name << " : " << scenes(i).MP->tasks(c)->prec << endl;
        }
      }
    }
  }
};

#endif // SCENE_H
