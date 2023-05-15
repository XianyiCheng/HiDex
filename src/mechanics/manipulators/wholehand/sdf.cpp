#include "sdf.h"

double BoxSDF(const Vector3d &box_shape, const Vector7d &box_pose, const Vector3d &pt)
{
    // 1. transform the point in to the box frame
    Matrix4d T = SE3Inv(pose2SE3(box_pose));
    Vector3d pt_o = T.block<3, 3>(0, 0) * pt + T.block<3, 1>(0, 3); 

    // 2. sd box 
    // https://iquilezles.org/articles/distfunctions/
    Vector3d q = pt_o.cwiseAbs() - box_shape/2;
    Vector3d q1 = q.cwiseMax(0);
    double d = q1.norm() + std::min(std::max(q[0], std::max(q[1], q[2])), 0.0);

    return d;  
}

double PointsSDF(const std::vector<ContactPoint> &points, const Vector3d &pt)
{
    // points are in the world frame

    // Find closest point to p on the mesh
    int closest_idx = 0;
    double closest_dist = std::numeric_limits<double>::max();
    Vector3d p_closest;
    Vector3d n_closest;

    for (int i = 0; i < points.size(); i++)
    {
        Vector3d p_w = points[i].p;
        Vector3d n_w = points[i].n;

        double d = (p_w - pt).norm();
        if (d < closest_dist)
        {
            closest_dist = d;
            closest_idx = i;
            n_closest = n_w;
            p_closest = p_w;
        }
    }

    // Find the sign
    Vector3d v = pt - p_closest;
    double sign = v.dot(n_closest);
    if (sign > 0)
    {
        return closest_dist;
    }
    else
    {
        return -closest_dist;
    }
}