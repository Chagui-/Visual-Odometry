#pragma once
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
using namespace cv;

class FMAlgorithm{
protected:

public:

	FMAlgorithm()	{
	}

	~FMAlgorithm(){}

	virtual std::vector<DMatch> match(Mat* queryDescriptors, Mat* trainDescriptors){ return  std::vector<DMatch>(); }
};

