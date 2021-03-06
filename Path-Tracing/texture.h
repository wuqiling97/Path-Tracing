#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <cmath>
#include <stdexcept>

#include "vector.h"



class Texture
{
private:
	std::vector<Vec3f> pixels;
	int width, height;
	bool loaded = false;

	inline double uchar2d(uchar ch) {
		return ch / 255.;
	}
public:
	Texture() {}
	Texture(std::string filepath) {
		using cv::Mat; using cv::Vec3b; using cv::imread;

		Mat img = cv::imread(filepath);
		if (img.data != nullptr) {
			cout<<"texture "<<filepath<<" loaded\n";
			loaded = true;
			width = img.cols;
			height = img.rows;
			for (int i = 0; i < height; i++) {
				for (int j = 0; j < width; j++) {
					// BGR store
					Vec3b& c = img.at<Vec3b>(i, j);
					pixels.push_back(Vec3f(uchar2d(c[2]), uchar2d(c[1]), uchar2d(c[0])));
				}
			}
		}
	}
	inline bool isload() { return loaded; }
	Vec3f getcolor(double u, double v) {
		//myassert(loaded);
		int x = round(clamp(u) * (width-1));
		int y = round(clamp(v) * (height-1));
		if (x >= 0 && x < width && y >= 0 && y < height) {
			return pixels[y*width + x];
		} else {
			printf("error with uv, yx: %.7lf, %.7lf - %i, %i (width, height: %i, %i) \n", u, v, x, y, width, height);
			return Vec3f(0, 1, 0);
		}

	}
};