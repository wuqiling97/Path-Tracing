#pragma once

#include "vector.h"
#include "ray.h"
#include "util.h"
#include <random>

class Camera {

private:
	double m_width_inv; // 1/width
	double m_height_inv;
	double m_ratio;
	double m_spacing;
	double m_spacing_half;
	Vec3f m_position;
	Vec3f m_direction;
	Vec3f m_target;
	Vec3f m_x_direction;
	Vec3f m_y_direction;

public:
	const int width, height;
	Camera(Vec3f position, Vec3f target, int width, int height);
	Ray get_ray(int x, int y, bool jitter, ushort* Xi);
};

Camera::Camera(Vec3f position, Vec3f target, int width, int height) :
	width(width), height(height), m_position(position), m_target(target)
{
	m_width_inv = 1. / width;
	m_height_inv= 1. / height;
	m_ratio = (double)width / height;

	m_direction = (target - m_position).norm();
	// ���������ߴ�ֱ��������λ����
	m_x_direction = m_direction.cross(Vec3f(0, 1, 0)).norm();
	m_y_direction = m_x_direction.cross(m_direction).norm();

	m_spacing = 2.0 / (double)height;
	m_spacing_half = m_spacing * 0.5;
}

// Returns ray from camera origin through pixel at x,y
Ray Camera::get_ray(int x, int y, bool jitter, ushort* Xi) 
{
	static std::default_random_engine generator;
	static std::uniform_real_distribution<double> uni(-1, 1);

	double x_jitter;
	double y_jitter;

	// If jitter == true, jitter point for anti-aliasing
	if (jitter) {
		// [-1, 1]/width
		x_jitter = uni(generator) * m_spacing_half;
		y_jitter = uni(generator) * m_spacing_half;
	} else {
		x_jitter = 0;
		y_jitter = 0;
	}

	Vec3f raydir = m_target - m_position + (
		(2 * x*m_width_inv - 1) * m_ratio * m_x_direction +
		(1 - 2 * y*m_height_inv) * m_y_direction);

	return Ray(m_position, raydir.norm());
}