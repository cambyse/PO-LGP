
#include <iostream>

extern "C" {
  #include <getopt.h>
  #include <sys/time.h>
}

#include <NP/nputils.h>
#include <NP/ucm.h>
#include <NP/cvutils.h>
#include <NP/vocutils.h>
#include <NP/desc.h>

static char shortopts[] = "ht:c:b:o:u:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "tree",           required_argument,       NULL,              't' },
  { "class",          required_argument,       NULL,              'c' },
  { "obj",            required_argument,       NULL,              'b' },
  { "output",         required_argument,       NULL,              'o' },
  { "numclasses",     required_argument,       NULL,              'u' },
  { NULL,              0,                      NULL,              0   }
};

int args
(
  charA& tree,
  charA& cls,
  charA& obj,
  charA& output,
  uint& num_classes,
  int argc,
  char** argv
);

int main(int argc, char** argv)
{
  seg::UcmTree tree;
  charA tree_filename, cls_filename, obj_filename, output;
  byteA img_cls, img_obj;
  floatA labels;
  intA cls, obj;
  uint num_classes;
  args(tree_filename, cls_filename, obj_filename, output, num_classes, argc, argv);

  // load the UCM tree
  seg::ucm_load(tree, tree_filename.p);

  // load the class and instance segmentation groundtruths
  np::load_image(img_cls, cls_filename.p);
  np::voc_segmask(cls, img_cls);
  np::load_image(img_obj, obj_filename.p);
  np::voc_segmask(obj, img_obj);

  // determine labels
  np::labels(labels, tree, cls, obj, num_classes);

  // save labels
  np::labels_save(labels, output.p);

  return 0;
}


int args
(
  charA& tree,
  charA& cls,
  charA& obj,
  charA& output,
  uint& num_classes,
  int argc,
  char** argv
)
{
  int status = 0;
  int ch = 1;

  while (ch != -1) 
  {
    ch = getopt_long(argc, argv, shortopts, longopts, NULL);
    switch(ch)
    {
      case 't':
        tree.resize(strlen(optarg)+1);
        tree = '\0';
        memcpy(tree.p, optarg, strlen(optarg));
        status++;
        break;
      case 'o':
        output.resize(strlen(optarg)+1);
        output = '\0';
        memcpy(output.p, optarg, strlen(optarg));
        status++;
        break;
      case 'c':
        cls.resize(strlen(optarg)+1);
        cls = '\0';
        memcpy(cls.p, optarg, strlen(optarg));
        status++;
        break;
      case 'u':
        num_classes = atoi(optarg);
        status++;
        break;
      case 'b':
        obj.resize(strlen(optarg)+1);
        obj = '\0';
        memcpy(obj.p, optarg, strlen(optarg));
        status++;
        break;

      case 0:
      case 1:
      case 2:
      case '?':
      case ':' :
      case 'h':
        std::cout << "Usage:" << std::endl
         << "  " << argv[0] << " [--help|-h]" << std::endl
         << "  [--tree|-t path/to/ucm.tree.array] [--output|-c path/to/ucm.labels.array]"
         << std::endl << "  [--class|-c path/to/class/segmentation.png]" << std::endl
         << "  [--obj|-b path/to/object/instance/segmentation.png]" << std::endl
         << "  [--numclasses|-u nc]"
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }
  
  if (status < 4)
    np::msg_error(HERE, "wrong usage!");

  return (status == 4);
};

