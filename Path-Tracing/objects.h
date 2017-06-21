#pragma once

#include "vector.h"
#include "material.h"

struct ObjectIntersection 
{
	bool hit;	// If there was an intersection
	double t;	// Distance to intersection along ray
	Vec3f n;	// Normal of intersected face
	Vec3f hitp; // hit point
	Material material;	// Material of intersected face

	ObjectIntersection(
		bool hit_, double t_ = 0, Vec3f n_ = Vec3f(), Vec3f hitp_ = Vec3f(), Material m_ = Material()
	) : hit(hit_), t(t_), n(n_), hitp(hitp_), material(m_) {}
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

	virtual double get_radius() { return m_radius; }
	virtual Material get_material() { return m_material; }

	// Check if ray intersects with sphere. Returns ObjectIntersection data structure
	virtual ObjectIntersection get_intersection(const Ray &ray) {
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

