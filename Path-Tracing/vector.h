#pragma once

//#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "util.h"

template<typename T>
class Vec3
{
public:
	T x, y, z;
	//Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
	Vec3(T xx=0, T yy=0, T zz=0) : x(xx), y(yy), z(zz) {}
	Vec3(const Eigen::Vector3d& v) : x(v[0]), y(v[1]), z(v[2]) {}

	Vec3& normalize() {
		T nor2 = length2();
		if (nor2 > 0) {
			T invNor = 1 / sqrt(nor2);
			x *= invNor, y *= invNor, z *= invNor;
		}
		return *this;
	}
	Vec3 norm() {
		T veclen = length();
		myassert(veclen>0);
		return *this * (1/veclen);
	}

	Vec3<T> operator * (const T &f) const { return Vec3<T>(x * f, y * f, z * f); }
	friend Vec3<T> operator * (const T& f, const Vec3<T>& v) { return Vec3<T>(f*v.x, f*v.y, f*v.z); }
	Vec3<T> mult (const Vec3<T> &v) const { return Vec3<T>(x * v.x, y * v.y, z * v.z); }
	T dot(const Vec3<T> &v) const { return x * v.x + y * v.y + z * v.z; }

	Vec3<T> cross(const Vec3<T>& v) {
		return Vec3<T>(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
	}
	Vec3<T> operator % (const Vec3<T>& v) {
		return Vec3<T>(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
	}

	Vec3<T> operator - (const Vec3<T> &v) const { return Vec3<T>(x - v.x, y - v.y, z - v.z); }
	Vec3<T> operator + (const Vec3<T> &v) const { return Vec3<T>(x + v.x, y + v.y, z + v.z); }
	Vec3<T>& operator += (const Vec3<T> &v) { x += v.x, y += v.y, z += v.z; return *this; }
	Vec3<T>& operator -= (const Vec3<T> &v) { x -= v.x, y -= v.y, z -= v.z; return *this; }
	Vec3<T>& operator *= (const Vec3<T> &v) { x *= v.x, y *= v.y, z *= v.z; return *this; }
	Vec3<T> operator - () const { return Vec3<T>(-x, -y, -z); }

	T length2() const { return x * x + y * y + z * z; }
	T length() const { return sqrt(length2()); }
	friend std::ostream & operator << (std::ostream &os, const Vec3<T> &v)
	{
		os << "[" << v.x << " " << v.y << " " << v.z << "]";
		return os;
	}
	Eigen::Vector3d toeigen() const {
		return Eigen::Vector3d(x, y, z);
	}
};

typedef Vec3<double> Vec3f;