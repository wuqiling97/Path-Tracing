#pragma once

#include "vector.h"
#include "material.h"
#include "util.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using std::cout; using std::endl;

struct ObjectIntersection 
{
	bool hit;	// If there was an intersection
	double t;	// Distance to intersection along ray
	Vec3f n;	// Normal of intersected face
	Vec3f hitp; // hit point
	Material material;	// Material of intersected face

	ObjectIntersection(
		bool hit_, double t_ = 0, Vec3f n_ = Vec3f(), Vec3f hitpoint_ = Vec3f(), Material m_ = Material()
	) : hit(hit_), t(t_), n(n_), hitp(hitpoint_), material(m_) {}
};


class Object 
{
public:
	Vec3f m_pos; // Position
	Object(Vec3f pos) : m_pos(pos) {}
	virtual ObjectIntersection get_intersection(const Ray &r) = 0;
};


class Sphere : public Object 
{
private:
	double m_radius;	// Radius
	Material m_material;	// Material

public:
	Sphere(Vec3f pos_, double radius_, Material material_) :
		Object(pos_), m_radius(radius_), m_material(material_) {}

	double get_radius() { return m_radius; }
	Material get_material() { return m_material; }

	// Check if ray intersects with sphere. Returns ObjectIntersection data structure
	ObjectIntersection get_intersection(const Ray &ray) {
		// Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
		bool hit = false;
		double distance = 0;
		Vec3f n = Vec3f();

		Vec3f l = m_pos - ray.origin;
		double t, eps = 1e-4;
		double tp = l.dot(ray.direction), det = m_radius*m_radius - (l.dot(l) - tp*tp);
		if (det<0) // center, ray distance > r
			return ObjectIntersection(hit);
		else 
			det = sqrt(det);

		bool inside = false;
		if ((t = tp - det) > eps) {
			distance = t;
			inside = false;
		}
		else if ((t = tp + det) > eps) {
			distance = t;
			inside = true;
		}
		else
			distance = 0;

		Vec3f hitpoint;
		if (distance != 0) {
			hit = true;
			hitpoint = ray.origin + ray.direction * distance;
			n = (hitpoint - m_pos).norm();
			if(inside) n = -n;
		}

		return ObjectIntersection(hit, distance, n, hitpoint, m_material);
	}
};


template <int degree> // 次数
class Bezier : public Object
{
private:
	std::vector<Eigen::Vector3d> m_points; // 相对m_pos的坐标
	Material m_material;
public:
	Bezier(Vec3f pos, Eigen::Vector3d points[], Material material) : 
		Object(pos), m_material(material) {
		for(int i=0; i<=degree; i++)
			m_points.push_back(points[i]);
	}
	Eigen::Vector3d _get_point(double t, std::vector<Eigen::Vector3d>& points) {
		//if(points.size()==1)
		//	return points[0];
		//else {
		//	int size = points.size() - 1;
		//	for (int i = 0; i < size; i++) {
		//		points[i] = (1-t)*points[i] + t*points[i+1];
		//	}
		//	points.pop_back();
		//	return _get_point(t, points);
		//}
		int n = points.size() - 1; // n-degree bezier
		for (int i = 1; i <= n; i++) {
			for (int j = 0; j <= n - i; j++) {
				points[j] = (1 - t)*points[j] + t*points[j + 1];
			}
		}
		return points[0];
	}
	Eigen::Vector3d get_point(double t) {
		auto temp = m_points;
		return _get_point(t, temp);
	}
	Eigen::Vector3d get_derivative(double t) {
		auto tmp1 = m_points;
		auto tmp2 = m_points;
		tmp1.erase(tmp1.begin());
		tmp2.pop_back();
		return degree*(_get_point(t, tmp1) - _get_point(t, tmp2));
	}
	Eigen::Matrix3d Jacobi(Eigen::Vector3d pt, const Vec3f& raydir) {
		// t: pt[0], u: pt[1], v: pt[2]
		double &t = pt[0], &u = pt[1], &v = pt[2];
		Eigen::Vector3d deri = get_derivative(u);
		Eigen::Vector3d bezier_pt = get_point(u);
		Eigen::Matrix3d ret; // jacobi matrix
		ret <<
			raydir.x, deri.x()*sin(v), bezier_pt.x()*cos(v), 
			raydir.y, deri.y(),        0, 
			raydir.z, deri.x()*cos(v), bezier_pt.x()*(-sin(v));
		return ret;
	}
	ObjectIntersection get_intersection(const Ray& ray) {
		using namespace Eigen;

		//TODO: t的初值选择与包围盒的交点

		// 平移bezier曲线到原点
		Vector3d rayori = ray.origin.toeigen() - m_pos.toeigen();
		Vector3d raydir = ray.direction.toeigen();

		Vector3d point(0, 0.5, M_PI);
		double &t = point[0], &u = point[1], &v = point[2];
		double tprev = 0;
		double eps = 1e-4;
		Matrix3d Jmat; // jacobi matrix
		Vector3d fvalue; //f(t, u, v) = S(u, v)-C(t)
		bool ishit = false;
		Vec3f normal;

		// 牛迭求解t, u, v
		for (int i = 0; ; i++) {
			Vector3d deri = get_derivative(u);
			Vector3d bezier_pt = get_point(u);
			Jmat <<
				-raydir[0], deri.x()*sin(v), bezier_pt.x()*cos(v),
				-raydir[1], deri.y(), 0,
				-raydir[2], deri.x()*cos(v), bezier_pt.x()*(-sin(v));
			fvalue = Vector3d(bezier_pt.x()*sin(v), bezier_pt.y(), bezier_pt.x() * cos(v))
				- (rayori + t * raydir);

			tprev = t;
			point = point - Jmat.inverse() * fvalue;
			if (abs(tprev - t) < 1e-7) {
				ishit = true;
				//cout<<"iter time = "<<i<<endl;
				// caculate normal
				Vec3f dfdu(Jmat(0,1), Jmat(1,1), Jmat(2,1));
				Vec3f dfdv(Jmat(0,2), Jmat(1,2), Jmat(2,2));
				normal = (dfdu % dfdv).norm();
				if(ray.direction.dot(normal) >= 0)
					normal = -normal;

				break;
			}
			if (i > 20) {
				ishit = false;
				break;
			}
		}
		v = v - floor(v/(2 * M_PI)) * 2 * M_PI;
		myassert(v >= 0 && v <= 2*M_PI);

		if (ishit && t > eps && u >= 0 && u <= 1) {
			Vec3f hitp = Vec3f(point) + m_pos;
			normal = normal + m_pos;
			return ObjectIntersection(true, t, normal, hitp, m_material);
		} else
			return ObjectIntersection(false);
	}
};