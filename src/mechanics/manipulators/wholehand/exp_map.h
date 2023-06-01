#include "../../external/geometry-central/include/geometrycentral/surface/trace_geodesic.h"
#include "../../external/geometry-central/include/geometrycentral/surface/meshio.h"
#include "../../external/geometry-central/include/geometrycentral/surface/surface_point.h"

#ifndef CONTACTCONSTRAINTS_H
#define CONTACTCONSTRAINTS_H
#include "../../contacts/contact_constraints.h"
#endif

using namespace geometrycentral;
using namespace geometrycentral::surface;

class ExpMapMesh
{
public:
    ExpMapMesh(std::string filename);
    ContactPoint exp_map(int v_idx, double dist, double theta);
    ContactPoint exp_map_uv(int v_idx, double u, double v);
    int find_closes_vertex(Vector3d p);

// private:
    std::unique_ptr<ManifoldSurfaceMesh> mMesh;
    std::unique_ptr<VertexPositionGeometry> mGeometry;
};