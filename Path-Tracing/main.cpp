#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
using std::cout; using std::cin; using std::endl;

// 使用的坐标系: x向右, y向上, z向前

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
	clock_t timestart = clock();

	int samples = 10;
	if(argc==2) samples = atoi(argv[1]);

	//test();

	Camera camera = Camera(Vec3f(0, 1, 3), Vec3f(0, 0, -1), 640, 360);     // Create camera
	Scene scene = Scene();                                              // Create scene

																		// Add objects to scene
	scene.add(new Sphere(Vec3f(0, -1000, 0), 1000, Material()));
	scene.add(new Sphere(Vec3f(-1004, 0, 0), 1000, Material(DIFF, Vec3f(0.85, 0.4, 0.4))));
	scene.add(new Sphere(Vec3f(1004, 0, 0) , 1000, Material(DIFF, Vec3f(0.4, 0.4, 0.85))));
	scene.add(new Sphere(Vec3f(0, 0, -1006), 1000, Material()));
	scene.add(new Sphere(Vec3f(0, 110, 0)  , 100 , Material(EMIT, Vec3f(1, 1, 1), Vec3f(2.2, 2.2, 2.2))));
	scene.add(new Sphere(Vec3f(-1.3, 0.7, 0)   , 0.7   , Material(SPEC, Vec3f(1, 1, 1))));
	scene.add(new Sphere(Vec3f(1.3, 0.7, -0.5) , 0.7   , Material(DIFF, Vec3f(1, 1, 1))));
	//scene.add( dynamic_cast<Object*>(new Mesh(Vec3f(), "../obj/dragon2.obj", Material(DIFF, Vec3f(0.9, 0.9, 0.9)))) );


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