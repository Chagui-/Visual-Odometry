#pragma once
#include <opencv2\core\core.hpp>

using namespace cv;

static class RANSAC{
private:
	static int countInliers(Mat T, std::vector<Mat>& X, std::vector<Mat>& Y, float margin);

public:
	RANSAC(){}
	~RANSAC(){}

	static Mat compute(std::vector<Mat>& X, std::vector<Mat>& Y, float margin);
	static Mat getTransformationMatrix(std::vector<Mat>& X, std::vector<Mat>& Y);

};

