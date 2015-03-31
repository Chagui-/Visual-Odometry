#pragma once
#include "FRAlgorithm.h"
#include <opencv2\nonfree\features2d.hpp>

class FRSurf : public FRAlgorithm{
private:
	int m_minHessian;
	SurfFeatureDetector m_detector;
	SurfDescriptorExtractor m_extractor;

public:
	FRSurf(void);
	FRSurf(int minHessian);
	~FRSurf(void);

	void setMinHessian(int minHessian);
	int getMinHessian();

	virtual ImageFeatures* detect( Mat* img );
};

