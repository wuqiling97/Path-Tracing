#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <Eigen/Dense>
using std::cout; using std::cin; using std::endl;

// 使用的坐标系: x向右, y向上, z向前

//#define private public
//#include "objects.h"
//#include "aabbox.h"

#include "vector.h"
#include "camera.h"
#include "scene.h"
#include "renderer.h"

void write_obj(Bezier<3> obj)
{
	using namespace Eigen;
	// 存储控制点
	std::vector<Vector3d> points(0);          // 存储点
	std::vector<Vector4i> meshes(0);            // 存储面
	int nu = 100, nv = 100;   // 密度应该“除”得整
	double du = 1./nu, dv = 2*M_PI/nv;             // 自定义密度

	int pid = 0;                            // 点序号
	double epsilon = 1e-6;
	for (int i = 0; i<nu; i++) {
		for (int j = 0; j<nv; j++) {
			double u = double(i) / nu, v = double(j) / nv * 2*M_PI;
			Vector3d tmp = obj.get_point(u);
			Vector3d point = Vector3d(tmp.x()*sin(v), tmp.y(), tmp.x() * cos(v));
			points.push_back(point);          // 写递归或者DP搞定函数P的计算
			pid++;                              // OBJ格式网格序号从1开始
			if (i != 0 && j != 0) {
				meshes.push_back(Vector4i(pid - nv - 1, pid - nv, pid, pid - 1));
			}
			if (j==nv-1 && i!=0) {
				meshes.push_back(Vector4i(pid-nv, pid-2*nv+1, pid-nv+1, pid));
			}
		}
	}
	//assert(points.size()==meshes.size());
	FILE* fp = fopen("bezier.obj", "w");
	for(Vector3d pt: points)
		fprintf(fp, "v %f %f %f\n", pt.x(), pt.y(), pt.z());
	for(Vector4i face: meshes)
		fprintf(fp, "f %d %d %d %d\n", face[0], face[1], face[2], face[3]);
}


int main(int argc, char* argv[])
{
	if (argc != 2) {
		printf("usage: <this.exe> <sample time: int>\n");
		return 0;
	}
	using Eigen::Vector3d;

	std::ifstream fin("config.txt");

	clock_t timestart = clock();

	int samples = 100;
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
	Bezier<3> vase = Bezier<3>(Vec3f(0.4, 0, 0.4), pts, Material(SPEC, Vec3f(0.4, 0.85, 0.4)));
	scene.add(&vase);

	write_obj(vase);


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