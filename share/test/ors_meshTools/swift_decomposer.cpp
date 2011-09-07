#include <SWIFT.h>
#include <SWIFT_mesh.h>
#include <SWIFT_mesh_utils.h>
#include <../decomposer/include/convex.h>
#include <../decomposer/include/io.h>

void decompose(ors::Mesh& mesh,const char* filename){
  SWIFT_Scene swift;
  SWIFT_Tri_Mesh smesh;
  smesh.Create(mesh.V.p, (int*)mesh.T.p,
               mesh.V.d0, mesh.T.d0,
               DEFAULT_ORIENTATION,
               DEFAULT_TRANSLATION,
               1.);

  SWIFT_Array<bool> edge_convexities;
  SWIFT_Array<int> piece_ids;
  SWIFT_Array< SWIFT_Array<int> > model_faces;
  SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> > virtual_faces;

  Mesh_Utils_Initialize();
  smesh.Compute_Edge_Convexities( edge_convexities );
  /*cerr << "Jittering with amplitude = " << jampl << endl << endl;
  Jitter( mesh, jampl );
  cerr << "Flipping edges with tolerance = " << edge_flip_tol
       << endl << endl;
  Edge_Flip( mesh, edge_flip_tol );
  */
  Convex_Initialize( &smesh );
  Decompose_Cresting_BFS(&smesh, piece_ids, model_faces, virtual_faces );

  Save_Decomposition_File( filename, &smesh, piece_ids,
                           model_faces, virtual_faces );

}