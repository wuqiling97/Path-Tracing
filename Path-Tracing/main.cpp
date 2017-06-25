#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
using std::cout; using std::cin; using std::endl;

// 使用的坐标系: x向右, y向上, z向前

//#define private public
//#include "objects.h"
//#include "aabbox.h"

#include "vector.h"
#include "camera.h"
#include "scene.h"
#include "renderer.h"

void test()
{
	Vec3f a = Vec3f(1, 2, 3);
	cout<<2*a;
}


int main(int argc, char* argv[])
{
	using Eigen::Vector3d;

	std::ifstream fin("config.txt");

	clock_t timestart = clock();

	int samples = 10;
	if(argc==2) samples = atoi(argv[1]);

	//test();

	int w, h;
	fin>>w>>h;
	printf("width: %d  height: %d\n", w, h);
	Camera camera = Camera(Vec3f(0, 4.3, 6), Vec3f(0, -0.26, -1), w, h);     // Create camera
	Scene scene = Scene();                                              // Create scene

	// Add objects to scene
	scene.add(new Sphere(Vec3f(0, -1e5, 0), 1e5, Material(DIFF, Vec3f(0.8, 0.8, 0.8)))); //bottom
	scene.add(new Sphere(Vec3f(-(1e5+4), 0, 0), 1e5, Material(DIFF, Vec3f(0.85, 0.4, 0.4)))); //left
	scene.add(new Sphere(Vec3f(1e5+4, 0, 0) , 1e5, Material(DIFF, Vec3f(0.4, 0.4, 0.85)))); //right
	scene.add(new Sphere(Vec3f(0, 0, -(1e5+4)), 1e5, Material(DIFF, Vec3f(0.8, 0.8, 0.8)))); //back
	double top = 6;
	scene.add(new Sphere(Vec3f(0, 1e5 + top, 0), 1e5, Material(DIFF, Vec3f(0.8, 0.8, 0.8)))); //top
	int lighth = 100;
	scene.add(new Sphere(Vec3f(0, lighth + top - 0.015, 0)  , lighth , Material(EMIT, Vec3f(1, 1, 1), Vec3f(7,7,7)))); //light
	scene.add(new Sphere(Vec3f(-1.7, 1.3, -1)   , 1.3   , Material(SPEC, Vec3f(0.8, 0.8, 0.8)), "tex/marble.jpg"));
	scene.add(new Sphere(Vec3f(2, 1, -0.5) , 1   , Material(DIFF, Vec3f(1, 1, 1)), "tex/orange.jpg"));
	Vector3d pts[4] = {
		Vector3d(0.338, 0.000, 0),
		Vector3d(1.600, 1.523, 0),
		Vector3d(0.000, 1.862, 0),
		Vector3d(0.446, 3.385, 0)
	};
	scene.add(new Bezier<3>(Vec3f(0.4, 0, 0.4), pts, Material(SPEC, Vec3f(0.4, 0.85, 0.4))));

	//Ray ray = camera.get_ray(202, 184, 0);
	//cout<<ray.origin<<' '<<ray.direction<<endl;
	//
	//Bezier<3> obj = Bezier<3>(Vec3f(0, 0, 0), pts, Material(SPEC, Vec3f(0.4, 0.85, 0.4)));
	//ObjectIntersection inter =  obj.get_intersection(ray);
	//cout<<"\nintersection info\n";
	//cout<<inter.hit<<' '<<inter.t<<endl
	//	<<inter.hitp<<' '<<inter.n<<endl;


	Renderer renderer = Renderer(&scene, &camera);  // Create renderer with our scene and camera
	renderer.render(samples);                       // Render image to pixel buffer
	renderer.save_image("image.ppm");              // Save image

	clock_t timeend = clock();
	double diff = double(timeend - timestart) / CLOCKS_PER_SEC;
	int hrs = (int)diff / 3600;
	int mins = ((int)diff / 60) - (hrs * 60);
	float secs = diff - (hrs * 3600) - (mins * 60);
	printf("\rRendering (%i samples): Complete!\nTime Taken: %i hrs, %i mins, %.2f secs\n\n", samples, hrs, mins, secs);
	system("pause");
}