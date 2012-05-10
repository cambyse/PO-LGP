#define MT_IMPLEMENT_TEMPLATES
#include <MT/util.h>
#include <gtk/gtk.h>
#include <graphviz/gvc.h>


graph_t *create_test_graph(void){
#define NUMNODES 5

    Agnode_t *node[NUMNODES];
    Agraph_t *g;
    Agraph_t *sg;
    int j, k;
    char name[10];

    /* Create a new graph */
    aginit();
    //agsetiodisc(NULL, gvfwrite, gvferror);
    g = agopen("new_graph", AGDIGRAPH);

    /* Add nodes */
    for (j = 0; j < NUMNODES; j++) {
	sprintf(name, "%d", j);
	node[j] = agnode(g, name);
    }

    /* Connect nodes */
    for (j = 0; j < NUMNODES; j++) {
	for (k = j + 1; k < NUMNODES; k++) {
	    agedge(g, node[j], node[k]);
	}
    }

    sg = agsubg (g, "cluster1");
    aginsert (sg, node[0]);

    return g;
}

int main(int argc, char **argv){
  gtk_init (&argc, &argv);

  GVC_t *gvc;
  graph_t * G;

  gvc = gvContext();
  G = create_test_graph();
  char *bla[] = {"dot", "-Tgtk", NULL};
  gvParseArgs(gvc, 2, bla);
  gvLayoutJobs(gvc, G);
  gvRenderJobs(gvc, G);
  gtk_main();
  gvFinalize(gvc);
  
  gvFreeContext(gvc);
  return 0;
}

