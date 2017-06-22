#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
	cout << "Assertion fail:  " << exp << endl;
	throw Exception();
}

#define myassert(exp) if(!(exp)) { _assert_fail(#exp);}