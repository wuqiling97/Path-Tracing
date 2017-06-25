#pragma once

#include "vector.h"
#include "ray.h"
#include "util.h"
#include <random>
#include <ctime>

class Camera
{
private:
	double m_width_inv; // 1/width
	double m_height_inv;
	double m_ratio;
	double m_spacing;
	double m_spacing_half;
	Vec3f m_position;
	Vec3f m_direction;
	//Vec3f m_rawdirection;
	Vec3f m_x_direction;
	Vec3f m_y_direction;

public:
	const int width, height;
	Camera(Vec3f position, Vec3f direction, int width, int height) :
		width(width), height(height), m_position(position), m_direction(direction.norm()) {
		m_width_inv = 1. / width;
		m_height_inv = 1. / height;
		m_ratio = (double)width / height;

		// 计算与视线垂直的2个方向向量, 长度为 x: w/h, y: 1
		m_x_direction = m_direction.cross(Vec3f(0, 1, 0)).norm() * 
			(double(width) / double(height));
		m_y_direction = m_x_direction.cross(m_direction).norm();

		m_spacing = 2.0 / (double)height;
		m_spacing_half = m_spacing * 0.5;
	}
	inline float convert_rand(float num) {
		if (num >= 0)
			return 1 - sqrt(num);
		else
			return sqrt(-num) - 1;
	}
	// Returns ray from camera origin through pixel at x,y
	Ray get_ray(double x, double y, bool jitter)
	{
		static std::default_random_engine generator(time(0));
		const double range = 1./ssaalen;
		static std::uniform_real_distribution<double> uni(-range, range);

		double x_jitter;
		double y_jitter;

		// If jitter == true, jitter point for anti-aliasing
		if (jitter) {
			// [-1, 1]/width
			x_jitter = convert_rand(uni(generator));
			y_jitter = convert_rand(uni(generator));
		} else {
			x_jitter = 0;
			y_jitter = 0;
		}

		Vec3f raydir = m_direction*1.6 + (
			((x + x_jitter) * 2*m_width_inv - 1) * m_x_direction +
			(1 - (y + y_jitter) * 2*m_height_inv) * m_y_direction);

		return Ray(m_position, raydir.norm());
	}
};

