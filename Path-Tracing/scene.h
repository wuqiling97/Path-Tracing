#pragma once

#include "ray.h"
#include "vector.h"
#include "objects.h"
#include "util.h"
#include "erand48.h"
#include <algorithm>
#include <random>


class Scene {
private:
	std::vector<Object*> m_objects;

public:
	Scene() {};
	void add(Object *object) {
		m_objects.push_back(object);
	}
	ObjectIntersection intersect(const Ray &ray) {
		ObjectIntersection isintersect(false);

		for (Object* i: m_objects){
		    ObjectIntersection temp = i->get_intersection(ray);
		    if (temp.hit && (isintersect.t == 0 || temp.t < isintersect.t)) {
		        isintersect = temp;
		    }
		}
		return isintersect;
	}
	Vec3f trace_ray(const Ray &ray, int depth, const ushort*Xi) {
		ObjectIntersection isct = intersect(ray);

		// If no hit, return world colour
		if (!isct.hit) return Vec3f();
		/*if (!isct.hit){
		double u, v;
		v = (acos(Vec3f(0,0,1).dot(ray.direction))/M_PI);
		u = (acos(ray.direction.y)/ M_PI);
		return bg.get_pixel(fabs(u), fabs(v))*1.2;
		}*/

		if (isct.material.type == EMIT) return isct.material.emission;

		Vec3f color = isct.material.color;

		// Calculate max reflection
		double p = std::max(std::max(color.x, color.y), color.z);

		// Russian roulette termination.
		// If random number between 0 and 1 is > p, terminate and return hit object's emmission
		static std::default_random_engine generator;
		static std::uniform_real_distribution<double> uni(0, 1);
		double rnd = uni(generator);
		if (++depth>5) {
			if (rnd<p*0.9) { // Multiply by 0.9 to avoid infinite loop with colours of 1.0
				color = color*(0.9 / p);
			} else {
				return isct.material.get_emission();
			}
		}

		Ray reflected = isct.material.get_reflected_ray(ray, isct.hitp, isct.n, Xi);

		return color.mult(trace_ray(reflected, depth, Xi));
	}
};

