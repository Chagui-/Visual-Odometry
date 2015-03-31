#pragma once
#include "FRAlgorithm.h"
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2/nonfree/gpu.hpp>

using namespace gpu;

class FRSurfGpu : public FRAlgorithm{
private:
	int m_minHessian;
	SurfFeatureDetector m_detector;
	SurfDescriptorExtractor m_extractor;
	SURF_GPU surf;

public:
	FRSurfGpu(void);
	FRSurfGpu(int minHessian);
	~FRSurfGpu(void);

	void setMinHessian(int minHessian);
	int getMinHessian();

	virtual ImageFeatures* detect(GpuMat* img);
};

