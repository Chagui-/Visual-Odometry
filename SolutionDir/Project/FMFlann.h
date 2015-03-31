#pragma once
#include "FMAlgorithm.h"
class FMFlann : public FMAlgorithm{
private:
	FlannBasedMatcher m_matcher;

public:
	FMFlann();
	~FMFlann();

	std::vector<DMatch> match(Mat* queryDescriptors, Mat* trainDescriptors);
};

