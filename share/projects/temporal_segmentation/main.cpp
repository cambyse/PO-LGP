#include <Core/util.h>
#include <Perception/g4data.h>
#include "kf.h"
#include "gui.h"
#include "optim.h"
#include "util.h"
#include "g4player.h"

int main(int argc, char **argv) {
  // BFGS bfgs;
  // bfgs.objf = ObjectiveFunction(3);
  // bfgs.objf.f = [](const arr &x) { return scalarProduct(x, x); };
  // bfgs.objf.jacobian = [](const arr &x) { return 2.*x; };
  // bfgs.objf.hessian = [](const arr &x) { return zeros(x.N, x.N); };

  // arr x = ARR(10, 10, 10);
  // bfgs.loopUntil(x, 1e-2);
  // cout << "SOLUTION: " << x << endl;

  // exit(0);

  G4Data g4d;

  String dset1 = STRING("seg_stack_move_1");
  // String dset2 = STRING("3rdHand/seg_toolbox_1");
  // String dset2 = STRING("seg_stack_move_2");
  g4d.base().clear() << "/home/bais/data/mocaplab/segments/";
#if 1
  // g4d.base().clear() << "/home/bais/data/mocaplab/segments/3rdHand/";
  g4d.load(dset1);
  cout << g4d.rec(dset1) << endl;
#endif

#if 1
  G4FeatSeqL featseqlist;
  auto start = watch::now();
  featseqlist = g4d.rec(dset1).featseqlist(true);
  cout << "featseqlist: " << featseqlist << endl;
  cout << "len: " << featseqlist.N << endl;
  cout << "time: " << watch::from<seconds>(start) << "s." << endl;

  G4FeatSeq featseq;
  // featseq = g4d.rec(dset1).featseq("rh:index", "sbox");

#endif
#if 0
  g4d.load(dset2);
  cout << g4d.rec(dset2) << endl;
#endif
#if 1
  String sens1 = STRING("rh:index");
  // String sens2 = STRING("toolbox:side_right");
  String sens2 = STRING("sbox");
  featseq = g4d.rec(dset1).featseq(sens1, sens2);

  KFLogitCRF model;
  model.params.set("nsplits", 10u);

  // arr thresholds;
  // for(double th = 0; th <= 1; th += .2)
  //   thresholds.append(th);
  // ROCCurve roc = model.ROC(featseqlist, thresholds);
  // COut << "ROC area: " << roc.area() << endl;
  // roc.show("ROC.pdf");

  // model.params.set("thresh", .5);
  // model.train(featseqlist);
  // arr interaction = model.query(featseq);

  // TODO I should take the returning interaction sequence and expand it.
  // Who knows how to expand?
  // -> The G4Rec
  // BUT I can also make it that the necessary information is within the sequence itself.. then the sequence itself will contain the necessary information
  // cout << "interaction: " << interaction << endl;

#endif

  G4Rec &rec = g4d.rec(dset1);
  cout << rec.id() << endl;

  G4Player g4player;
  g4player.play(rec);
  // g4player.play(rec, STRING("sh:" << sens1), STRING("sh:" << sens2), interaction);
  exit(0);

#if 0
  // PhiMachine
  PhiMachine pm;
  pm.set(PhiLin());
  arr data = { 0, 1, 2, 3 };
  cout << "data: " << data << endl;
  cout << "phi(data): " << pm.convert(data) << endl;
#endif
}

