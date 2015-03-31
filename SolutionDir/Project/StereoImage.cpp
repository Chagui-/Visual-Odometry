#include "StereoImage.h"
#include "Camera.h"
#include "FRAlgorithm.h"
#include "FMAlgorithm.h"

StereoImage::StereoImage(){
}

StereoImage::~StereoImage(){
	delete m_features_l;
	delete m_features_r;
	m_features_l = nullptr;
	m_features_r = nullptr;
}

Point3f StereoImage::triangluatePoint(Camera c, Point2f p1, Point2f p2){
	Point3f output(0);
	Point2f pp1 = c.pixels2mm(p1);
	Point2f pp2 = c.pixels2mm(p2);
	//NOTE: max disparity is 0.912mm for 1m depth
	output.z = c.getProperties().getFocalLength_mm() * c.getProperties().getDistanceBetweenLenses_mm() / (pp1.x - pp2.x);
	output.x = pp1.x * output.z / c.getProperties().getFocalLength_mm();
	output.y = pp1.y * output.z / c.getProperties().getFocalLength_mm();

	output *= 0.001f; // to meters
	
	return output;
}

void StereoImage::triangulate(Camera c){
	for (int i = 0; i < m_good_matches.size(); i++)	{
		Point3f point_3d = triangluatePoint(
			c, 
			m_features_l->getKeypoints()[m_good_matches[i].queryIdx].pt,
			m_features_r->getKeypoints()[m_good_matches[i].trainIdx].pt);
		m_coords[m_good_matches[i].queryIdx] = point_3d;
	}
}


void StereoImage::computeFeatures(FRAlgorithm* fr_algorithm, FMAlgorithm* fm_algorithm, Mat* img_l, Mat* img_r){
	//Extract Features
	m_features_l = fr_algorithm->detect(img_l);
	m_features_r = fr_algorithm->detect(img_r);


	//Matching descriptor vectors using FLANN matcher
	m_matches = fm_algorithm->match(m_features_l->getDescriptors(), m_features_r->getDescriptors());
}

//void StereoImage::computeFeaturesGPU(FRSurfGpu* fr_surf_gpu, FMAlgorithm* fm_algorithm, Mat* img_l, Mat* img_r){
//	//Extract Features
//	gpu::GpuMat img_l_gpu(*img_l);
//	gpu::GpuMat img_r_gpu(*img_r);
//	m_features_l = fr_surf_gpu->detect(&img_l_gpu);
//	m_features_r = fr_surf_gpu->detect(&img_r_gpu);
//
//
//	//Matching descriptor vectors using FLANN matcher
//	m_matches = fm_algorithm->match(m_features_l->getDescriptors(), m_features_r->getDescriptors());
//}