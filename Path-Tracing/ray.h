#pragma once

#include "vector.h"

struct Ray
{
	Vec3f origin, direction;
	Ray(Vec3f origin_, Vec3f direction_) :
		origin(origin_), direction(direction_) {}
};