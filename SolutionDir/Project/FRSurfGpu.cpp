#include "FRSurfGPU.h"


FRSurfGpu::FRSurfGpu(void) : m_minHessian(500){
}

FRSurfGpu::FRSurfGpu(int minHessian) : m_minHessian(minHessian), m_detector(minHessian){

}

FRSurfGpu::~FRSurfGpu(void)
{
}

ImageFeatures* FRSurfGpu::detect(GpuMat* img){
	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

	// detecting keypoints & computing descriptors
	GpuMat keypoints1GPU;
	GpuMat descriptors1GPU;
	surf(*img, GpuMat(), keypoints1GPU, descriptors1GPU);

	vector<KeyPoint> keypoints1;
	vector<float> descriptors1;
	surf.downloadKeypoints(keypoints1GPU, keypoints1);
	surf.downloadDescriptors(descriptors1GPU, descriptors1);

	Mat descriptors(descriptors1);

	return new ImageFeatures(keypoints1, descriptors);
}

void FRSurfGpu::setMinHessian(int minHessian){
	m_minHessian = minHessian;
	m_detector = SurfFeatureDetector(minHessian);
}

int FRSurfGpu::getMinHessian(){
	return m_minHessian;
}
