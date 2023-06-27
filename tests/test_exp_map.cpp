#include "../src/mechanics/manipulators/wholehand/exp_map.h"

int main(int argc, char *argv[]) {

  std::string mesh_file;

  if (argc > 1) {
    mesh_file = argv[1];
  } else {
    mesh_file = std::string(SRC_DIR) + "/data/wholehand/assets/AllegroHand/ManifoldAllegroHand.obj";
  }
  ExpMapMesh expMapMesh(mesh_file);

  double dist = 0.1;
  double angle = 3.1415*45/180;
  int vertex_idx = 0;
  std::cout << "vertex_idx: " << vertex_idx << std::endl;
  std::cout << "vertex pos: " << expMapMesh.mGeometry->inputVertexPositions[vertex_idx] << std::endl; 
  std::cout << "vertex normal: " << expMapMesh.mGeometry->vertexNormals[vertex_idx] << std::endl;

  ContactPoint contactPoint = expMapMesh.exp_map(vertex_idx, dist, angle);
  std::cout << "output pos: " << contactPoint.p.transpose() << std::endl;
  std::cout << "output normal: " << contactPoint.n.transpose() << std::endl;
  return 0;
}