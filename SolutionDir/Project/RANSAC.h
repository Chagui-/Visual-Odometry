#pragma once
#include <opencv2\core\core.hpp>

using namespace cv;

class RANSAC{
private:
	static int countInliers(Mat T, std::vector<Mat>& X, std::vector<Mat>& Y, float margin);
	static int m_max_iterations;
	static float m_margin;

public:
	RANSAC(){}
	~RANSAC(){}

	static void initialize(int max_iterations = 400,float margin = 0.1f);

	static Mat compute(std::vector<Mat>& X, std::vector<Mat>& Y);
	static Mat getTransformationMatrix(std::vector<Mat>& X, std::vector<Mat>& Y);

};