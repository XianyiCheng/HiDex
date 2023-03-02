#include "utilities.h"
#include "sample.h"
#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace Eigen;

#define PI 3.1415926

Vector3d sample_position(const Vector3d &ub, const Vector3d &lb){
    
    Vector3d t(randd(),randd(),randd());
    Vector3d q;
    q = t.cwiseProduct(ub - lb) + lb;
    return q;
}

Quaterniond generate_unit_quaternion(){
    double q_norm = 2.0;
    double qq[4];
    while (q_norm > 1){
        q_norm = 0;
        for (int i = 0; i<4; i++){
            qq[i] = randd()*2 - 1;
            q_norm += qq[i]*qq[i];
        }
        q_norm = sqrt(q_norm);
    }
    Quaterniond q(qq[0]/q_norm, qq[1]/q_norm, qq[2]/q_norm, qq[3]/q_norm);
    return q;
}

Quaterniond sample_rotation(const Vector3d& axis){
    // set_rand_seed();
    double a = 2*(randd()-0.5);
    AngleAxisd aa(a*PI, axis);
    Quaterniond q(aa);
    return q;
}

Vector3d steer_position(const Vector3d &p0, const Eigen::Vector3d &p1, const double &length){
    Vector3d dp;
    dp = p1 - p0;
    double dp_norm = dp.norm();
    Vector3d p;
    if (dp_norm > length){
        double t = length/dp_norm;
        p = p0 + t*dp;
    }
    else{
        p = p1;
    }
    return p;
    
}

Quaterniond steer_quaternion(const Quaterniond &q0, const Quaterniond &q1, const double &angle){

    double angle_bt_quat = angBTquat(q0, q1);
    Quaterniond q;
    if (angle_bt_quat > angle){
        double t = angle/angle_bt_quat;
        q = q0.slerp(t, q1);
    }
    else{
        q = q1;
    }
    return q;
}

void combinations_recursive(const std::vector<int> &elems, int req_len,
			    std::vector<int> &pos, int depth,
			    int margin, std::vector<std::vector<int>>* elem_combs)
{
	// Have we selected the number of required elements?
	if (depth >= req_len) {
        std::vector<int> ec;
		for (int ii = 0; ii < pos.size(); ++ii){
			ec.push_back(elems[pos[ii]]);
            //cout << elems[pos[ii]];
        }
        elem_combs->push_back(ec);
		//cout << endl;
		return;
	}

	// Are there enough remaining elements to be selected?
	// This test isn't required for the function to be correct, but
	// it can save a good amount of futile function calls.
	if ((elems.size() - margin) < (req_len - depth))
		return;

	// Try to select new elements to the right of the last selected one.
	for (int ii = margin; ii < elems.size(); ++ii) {
		pos[depth] = ii;
		combinations_recursive(elems, req_len, pos, depth + 1, ii + 1, elem_combs);
	}
	return;
}

void combinations(const std::vector<int> &elems, int req_len, std::vector<std::vector<int>>* elem_combs)
{
	std::vector<int> positions(req_len, 0);
	combinations_recursive(elems, req_len, positions, 0, 0, elem_combs);
}

Vector7d steer_config(Vector7d x_near, Vector7d x_rand,
                      double epsilon_translation, double epsilon_angle)
{

  // double epsilon_translation = 1.5;
  // double epsilon_angle = 3.14*100/180;

  Vector3d p_rand = x_rand.head(3);
  Vector3d p_near = x_near.head(3);
  Quaterniond q_near(x_near[6], x_near[3], x_near[4], x_near[5]);
  Quaterniond q_rand(x_rand[6], x_rand[3], x_rand[4], x_rand[5]);
  p_rand = steer_position(p_near, p_rand, epsilon_translation);
  q_rand = steer_quaternion(q_near, q_rand, epsilon_angle);
  Vector7d x_steer;
  x_steer << p_rand[0], p_rand[1], p_rand[2], double(q_rand.x()),
      double(q_rand.y()), double(q_rand.z()), double(q_rand.w());
  return x_steer;
}

void steer_velocity(Vector6d &x, double h, double cl) {

  Vector6d xx = x;
  xx.head(3) = xx.head(3) * cl;
  if (xx.norm() > h) {
    xx = (h / xx.norm()) * xx;
    double xx_norm = xx.tail(3).norm();
    double x_norm = x.tail(3).norm();
    if (xx_norm > 1e-4 && x_norm > 1e-4) {
      x = x * (xx.tail(3).norm() / x.tail(3).norm());
    } else {
      x = xx / cl;
    }
  }
  return;
}