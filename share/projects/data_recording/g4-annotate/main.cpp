#include <Core/array.h>
#include <Core/util.h>
#include <Core/keyValueGraph.h>

#include <fstream>

int main(int argc, char ** argv) {
  MT::initCmdLine(argc, argv);

  String dir;
  ifstream ann_g4;
  KeyValueGraph kvg, *subkvg, *lock;
  Item *item;

  MT::getParameter(dir, "dir");

  MT::open(ann_g4, STRING(dir << "/ann_g4.txt"));
  /* system(STRING("./g4-convert " << dir)); */

  char o1[101], o2[101];
  double *from_frame, *to_frame;
  while(ann_g4.good()) {
    from_frame = new double;
    to_frame = new double;
    *from_frame = -1;
    *to_frame = -1;

    ann_g4 >> o1 >> o2 >> *from_frame >> *to_frame;
    cout << "o1: " << o1 << endl;
    cout << "o2: " << o2 << endl;
    cout << "from_frame: " << *from_frame << endl;
    cout << "to_frame: " << *to_frame << endl;

    if(*from_frame < 0 || *to_frame < 0)
      break;

    lock = new KeyValueGraph();
    lock->append("from", from_frame);
    lock->append("to", to_frame);

    item = kvg.getItem(o1, o2);
    if(item) {
      subkvg = item->getValue<KeyValueGraph>();
      subkvg->append("lock", lock);
    }
    else {
      subkvg = new KeyValueGraph();
      subkvg->append("lock", lock);
      kvg.append(STRINGS(o1, o2), subkvg);
    }
  }

  ann_g4.close();

  FILE(STRING(dir << "/ann_kvg.kvg")) << kvg;

  return 0;
}

