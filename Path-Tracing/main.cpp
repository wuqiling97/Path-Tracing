#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
using std::cout; using std::cin; using std::endl;

// 使用的坐标系: x向右, y向上, z向前

#define private public
#include "objects.h"
#include "aabbox.h"

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

	clock_t timestart = clock();

	int samples = 10;
	if(argc==2) samples = atoi(argv[1]);

	//test();

	Camera camera = Camera(Vec3f(0, 2, 5), Vec3f(0, 0, -1), 640, 360);     // Create camera
	Scene scene = Scene();                                              // Create scene

	// Add objects to scene
	scene.add(new Sphere(Vec3f(0, -1000, 0), 1000, Material()));
	scene.add(new Sphere(Vec3f(-1004, 0, 0), 1000, Material(DIFF, Vec3f(0.85, 0.4, 0.4))));
	scene.add(new Sphere(Vec3f(1004, 0, 0) , 1000, Material(DIFF, Vec3f(0.4, 0.4, 0.85))));
	scene.add(new Sphere(Vec3f(0, 0, -1006), 1000, Material()));
	scene.add(new Sphere(Vec3f(0, 110, 0)  , 100 , Material(EMIT, Vec3f(1, 1, 1), Vec3f(2.2, 2.2, 2.2))));
	scene.add(new Sphere(Vec3f(-1.7, 0.7, 0)   , 0.7   , Material(SPEC, Vec3f(1, 1, 1))));
	scene.add(new Sphere(Vec3f(1.7, 0.7, -0.5) , 0.7   , Material(DIFF, Vec3f(1, 1, 1))));
	Vector3d pts[4] = {
		Vector3d(0.303, 0.000, 0),
		Vector3d(2.000, 1.645, 0),
		Vector3d(0.145, 1.934, 0),
		Vector3d(0.184, 3.382, 0)
	};
	scene.add(new Bezier<3>(Vec3f(0, 0, 0), pts, Material(SPEC, Vec3f(0.4, 0.85, 0.4))));

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