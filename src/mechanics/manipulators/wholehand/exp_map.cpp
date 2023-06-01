#include "exp_map.h"

ExpMapMesh::ExpMapMesh(std::string filename)
{
    // load mesh
    std::tie(mMesh, mGeometry) = readManifoldSurfaceMesh(filename);
    mGeometry->requireVertexNormals();
}

ContactPoint ExpMapMesh::exp_map(int v_idx, double dist, double theta)
{
    Vertex v = mMesh->vertex(v_idx);
    SurfacePoint pathEndpoint;
    if (dist == 0)
    {
        pathEndpoint = SurfacePoint(v);
    }
    else
    {
        Vector2 traceVec = dist * Vector2::fromAngle(theta);
        pathEndpoint = traceGeodesic(*mGeometry, SurfacePoint(v), traceVec).endPoint;
    }
    Vector3 p3d = pathEndpoint.interpolate(mGeometry->inputVertexPositions);
    Vector3 n3d = pathEndpoint.interpolate(mGeometry->vertexNormals);
    // contact normal pointing inward to the object
    return ContactPoint(Vector3d(p3d.x, p3d.y, p3d.z), Vector3d(-n3d.x, -n3d.y, -n3d.z));
}

ContactPoint ExpMapMesh::exp_map_uv(int v_idx, double u, double v)
{
    Vertex vert = mMesh->vertex(v_idx);
    SurfacePoint pathEndpoint;
    if (u == 0 && v == 0)
    {
        pathEndpoint = SurfacePoint(vert);
    }
    else
    {
        Vector2 traceVec;
        traceVec.x = u;
        traceVec.y = v;
        pathEndpoint = traceGeodesic(*mGeometry, SurfacePoint(vert), traceVec).endPoint;
    }

    Vector3 p3d = pathEndpoint.interpolate(mGeometry->inputVertexPositions);
    Vector3 n3d = pathEndpoint.interpolate(mGeometry->vertexNormals);
    // contact normal pointing inward to the object
    return ContactPoint(Vector3d(p3d.x, p3d.y, p3d.z), Vector3d(-n3d.x, -n3d.y, -n3d.z));
}

int ExpMapMesh::find_closes_vertex(Vector3d p)
{
    double min_dist = 1e10;
    int min_idx = -1;
    for (int i = 0; i < mGeometry->inputVertexPositions.size(); i++)
    {
        Vector3 v_pos = mGeometry->inputVertexPositions[i];
        Vector3d v_pos_d(v_pos.x, v_pos.y, v_pos.z);
        double dist = (v_pos_d - p).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            min_idx = i;
        }
    }
    return min_idx;
}