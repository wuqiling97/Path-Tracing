#pragma once

#include "vector.h"
#include "scene.h"
#include "camera.h"
#include "util.h"
#include <fstream>

// Clamp to between 0-255
inline int toInt(double x) { return int(clamp(x) * 255 + .5); }

class Renderer {

private:
	Scene *m_scene;
	Camera *m_camera;
	Vec3f *m_pixel_buffer;

public:
	Renderer(Scene *scene, Camera *camera) {
		m_scene = scene;
		m_camera = camera;
		m_pixel_buffer = new Vec3f[m_camera->width * m_camera->height];
	}
	void render(int samples) {
		int width = m_camera->width;
		int height = m_camera->height;
		double samples_inv = 1. / samples;
		int subsamples = samples/(ssaalen*ssaalen);

		// Main Loop
#pragma omp parallel for schedule(dynamic, 1)       // OpenMP
		for (int y = 0; y<height; y++) {             // Stores seed for erand48

			fprintf(stderr, "\rRendering (%i samples): %.2f%% ",      // Prints
				subsamples*(ssaalen*ssaalen), (double)y / height * 100);                   // progress

			for (int x = 0; x<width; x++) {
				Vec3f color = Vec3f();

				for (int a = 0; a<subsamples; a++) {
					// SSAA
					for (int sx = 0; sx < ssaalen; sx++)
						for (int sy = 0; sy < ssaalen; sy++) {
							Ray ray = m_camera->get_ray(x + (sx + 0.5) / ssaalen, y + (sy + 0.5) / ssaalen, a>0);
							color = color + m_scene->trace_ray(ray, 0);
						}
				}

				m_pixel_buffer[(y)*width + x] = color * samples_inv;
			}
		}
	}
	void save_image(std::string file_path) {
		int width = m_camera->width;
		int height = m_camera->height;

		std::ofstream ofs(file_path, std::ios::out | std::ios::binary);
		ofs << "P6\n" << width << " " << height << "\n255\n";
		for (int i = 0; i < width*height; i++) {
			ofs<<uchar(toInt(m_pixel_buffer[i].x))
			   <<uchar(toInt(m_pixel_buffer[i].y))
			   <<uchar(toInt(m_pixel_buffer[i].z));
		}
	}

};


