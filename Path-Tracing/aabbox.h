#pragma once

#include "vector.h"
#include "ray.h"

#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Vector3d;

class AABBox
{
private:
	Vector3d maxp, minp; // min, max of x, y, z
public:
	AABBox():
		maxp(Vector3d(-1e8, -1e8, -1e8)), minp(Vector3d(1e8, 1e8, 1e8)) { }

	void expand(std::vector<Vector3d> pts) {
		using std::max; using std::min;

		for (Vector3d v : pts) {
			for (int i = 0; i < 3; i++) {
				maxp[i] = max(maxp[i], v[i]);
				minp[i] = min(minp[i], v[i]);
			}
		}
		/*cout<<"max\n"<<maxp<<endl
			<<"min\n"<<minp<<endl;*/
	}
	double intersect_box(const Ray& ray) {
		using namespace std;

		Vector3d rayori = ray.origin.toeigen();
		const Vec3f& rd = ray.direction;
		Vector3d raydir_inv(1/rd.x, 1/rd.y, 1/rd.z);
		
		double tmin = -1e8, tmax = +1e8;
		
		Vector3d diffmax = maxp - rayori;
		Vector3d diffmin = minp - rayori;

		for (int i = 0; i < 3; i++) {
			diffmax[i] *= raydir_inv[i];
			diffmin[i] *= raydir_inv[i];
			if(diffmax[i] < diffmin[i])
				swap(diffmax[i], diffmin[i]);
			tmax = min(tmax, diffmax[i]);
			tmin = max(tmin, diffmin[i]);
		}
		/*cout<<"ti max\n"<<diffmax<<endl
			<<"ti min\n"<<diffmin<<endl;
		cout<<tmax<<' '<<tmin<<endl;*/
		const double eps = 1e-4;
		if (tmin < tmax) {
			if(tmin > eps)
				return tmin;
			else {
				if(tmax > eps)
					return tmax;
				else
					return 0;
			}
		} else
			return 0;
	}
};