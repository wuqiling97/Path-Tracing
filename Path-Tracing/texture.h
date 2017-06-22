#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <cmath>

#include "vector.h"
#include "util.h"


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
		myassert(loaded);
		int i = round(clamp(u) * (width-1));
		int j = round(clamp(v) * (height-1));
		return pixels[i*width + j];
	}
};