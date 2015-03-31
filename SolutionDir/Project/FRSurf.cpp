#include "FRSurf.h"


FRSurf::FRSurf(void): m_minHessian(500){
}

FRSurf::FRSurf(int minHessian) : m_minHessian(minHessian), m_detector(minHessian){
	
}

FRSurf::~FRSurf(void)
{
}

ImageFeatures* FRSurf::detect( Mat* img ){
	std::vector<KeyPoint> keypoints;
	Mat descriptors;

	m_detector.detect( *img, keypoints );

	m_extractor.compute( *img, keypoints, descriptors );

	return new ImageFeatures(keypoints,descriptors);
}

void FRSurf::setMinHessian(int minHessian){
	m_minHessian = minHessian;
	m_detector = SurfFeatureDetector(minHessian);
}

int FRSurf::getMinHessian(){
	return m_minHessian;
}
