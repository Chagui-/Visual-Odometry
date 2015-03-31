#pragma once
#include "opencv2/core/core.hpp"

using namespace cv;

class ImagePkg{
public:
	Mat* images[2];

	ImagePkg(){}
	~ImagePkg(){}
};

