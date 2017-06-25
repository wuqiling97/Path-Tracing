#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#include <iostream>

const int ssaalen = 2;
const double eps = 1e-4;

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef long long int64;
typedef unsigned long long uint64;

class Exception
{};

void _assert_fail(const char* exp)
{
	using std::cout;
	std::cerr << "Assertion fail:  " << exp << endl;
	throw Exception();
}

#define myassert(exp) if(!(exp)) { _assert_fail(#exp);}

// Clamp double to min/max of 0/1
inline double clamp(double x) { return x<0 ? 0 : x>1 ? 1 : x; }