#pragma once

#include "vector.h"
#include "ray.h"
#include "util.h"

#include <random>
#include <ctime>



enum MaterialType { DIFF, SPEC, EMIT };


class Material 
{

public:
	MaterialType type;
	Vec3f color;
	Vec3f emission;
	Material(MaterialType type = DIFF, Vec3f color = Vec3f(1, 1, 1), Vec3f emission = Vec3f(0, 0, 0)):
		type(type), color(color), emission(emission) {
		//m_texture = tex;
	}

	MaterialType get_type() const { return type; }
	Vec3f get_emission() const { return emission; }
	Vec3f get_color() const { return color; }

	Ray get_reflected_ray(const Ray &ray, Vec3f &hitpoint, const Vec3f &norm) const {
		static std::default_random_engine generator(time(0)+1);
		static std::uniform_real_distribution<double> halfuni(-0.5, 0.5);
		static std::uniform_real_distribution<double> uni(0, 1);

		// Ideal specular reflection
		if (type == SPEC) {
			double roughness = 0;
			Vec3f reflect_ray = ray.direction - norm * 2 * norm.dot(ray.direction);
			reflect_ray = (reflect_ray + 
				roughness * Vec3f(halfuni(generator), halfuni(generator), halfuni(generator)));
			reflect_ray.normalize();

			return Ray(hitpoint, reflect_ray);
			//return Ray(p, r.direction - n * 2 * n.dot(r.direction));
		}
		// Ideal diffuse reflection
		if (type == DIFF) {
			Vec3f nl = norm.dot(ray.direction)<0 ? norm : norm*-1;
			double r1 = 2 * M_PI*uni(generator), r2 = uni(generator), r2s = sqrt(r2);
			Vec3f w = nl, u = ((fabs(w.x)>.1 ? Vec3f(0, 1, 0) : Vec3f(1)) % w).norm(), v = w%u;
			Vec3f d = (u*cos(r1)*r2s + v*sin(r1)*r2s + w*sqrt(1 - r2)).norm();

			return Ray(hitpoint, d);
		}
	}

};


