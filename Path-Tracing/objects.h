#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

#include "vector.h"
#include "material.h"
#include "util.h"
#include "aabbox.h"
#include "texture.h"

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
protected:
	Texture m_texture;
	Material m_material;	// Material
	Vec3f m_pos; // Position
	// Get color at UV coordinates u,v
	Vec3f get_uvcolor(double u, double v) {
		if(m_texture.isload())
			return m_texture.getcolor(u, v);
		else
			return m_material.color;
	}
public:
	Object(Vec3f pos, Material material, std::string texpath) : 
		m_pos(pos), m_material(material), m_texture(Texture(texpath)){}
	virtual ObjectIntersection get_intersection(const Ray &r) = 0;
};


class Sphere : public Object 
{
private:
	double m_radius;	// Radius

public:
	Sphere(Vec3f pos_, double radius_, Material material_, std::string texpath_ = "") :
		Object(pos_, material_, texpath_), m_radius(radius_) {}

	double get_radius() { return m_radius; }
	Material get_material() { return m_material; }

	// Check if ray intersects with sphere. Returns ObjectIntersection data structure
	ObjectIntersection get_intersection(const Ray &ray) {
		// Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
		double distance = 0;
		Vec3f n = Vec3f();

		Vec3f l = m_pos - ray.origin;
		double t;
		double tp = l.dot(ray.direction), det = m_radius*m_radius - (l.dot(l) - tp*tp);
		if (det<0) // center, ray distance > r
			return ObjectIntersection(false);
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
			bool hit = true;
			hitpoint = ray.origin + ray.direction * distance;
			n = (hitpoint - m_pos).norm();
			if(inside) n = -n;

			if (m_texture.isload()) {
				double u = atan((hitpoint.y - m_pos.y) / (hitpoint.x - m_pos.x)) / M_PI + 0.5;
				double v = asin(clamp((hitpoint.z - m_pos.z) / m_radius)) / M_PI + 0.5;
				m_material.color = get_uvcolor(u, v);
			}

			return ObjectIntersection(hit, distance, n, hitpoint, m_material);
		} else
			return ObjectIntersection(false);

	}
};


template <int degree> // 次数
class Bezier : public Object
{
private:
	std::vector<Eigen::Vector3d> m_points; // 相对m_pos的坐标
	AABBox box;
public:
	Bezier(Vec3f pos, Eigen::Vector3d points[], Material material, std::string texpath = "") : 
		Object(pos, material, texpath) {
		using Eigen::Vector3d;

		std::vector<Vector3d> boxpts;
		Vector3d vpos = pos.toeigen();
		for (int i = 0; i <= degree; i++) {
			m_points.push_back(points[i]);
			Vector3d& p = points[i];
			// 传入控制点的对称位置
			boxpts.push_back(Vector3d(-p[0], p[1], -p[0]) + vpos);
			boxpts.push_back(Vector3d(p[0], p[1], p[0]) + vpos);
		}
		box.expand(boxpts);
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
	ObjectIntersection get_intersection(const Ray& ray) {
		using namespace Eigen;

		double boxt = box.intersect_box(ray);
		if (boxt == 0) {
			return ObjectIntersection(false);
		}
		//cout<<"boxt = "<<boxt<<endl;

		// 平移bezier曲线到原点
		Vector3d rayori = ray.origin.toeigen() - m_pos.toeigen();
		Vector3d raydir = ray.direction.toeigen();

		Vector3d point(0, 0.5, M_PI);
		double &t = point[0], &u = point[1], &v = point[2];
		t = boxt;
		Vector3d boxp = rayori + t*raydir; // 与包围盒的交点
		double &x = boxp[2], &y = boxp[0];
		// 设置v的初值为包围盒交点的角度
		if (x > 0) {
			if (y > 0)
				v = atan(y/x);
			else
				v = atan(y/x) + 2*M_PI;
		} else {
			v = atan(y/x) + M_PI;
		}

		Matrix3d Jmat; // jacobi matrix
		Vector3d fvalue; //f(t, u, v) = S(u, v)-C(t)
		bool ishit = false;
		Vec3f normal, hitp;

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

			point = point - Jmat.inverse() * fvalue;

			bool converge = true;
			for(int i=0; i<3; i++)
				if (abs(fvalue[i]) > 1e-7) {
					converge = false;
					break;
				}
			if (converge) {
				ishit = true;
				// caculate normal
				Vec3f dfdu(Jmat(0,1), Jmat(1,1), Jmat(2,1));
				Vec3f dfdv(Jmat(0,2), Jmat(1,2), Jmat(2,2));
				normal = (dfdu % dfdv).norm();
				if(ray.direction.dot(normal) >= 0)
					normal = -normal;
				hitp = ray.origin + t*ray.direction;

				//cout<<"iter time = "<<i<<endl;
				//cout<<"parameter = "<<point<<endl;
				//cout<<"fvalue = \n"<<fvalue<<endl;
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
			hitp += m_pos;
			return ObjectIntersection(true, t, normal, hitp, m_material);
		} else
			return ObjectIntersection(false);
	}
};