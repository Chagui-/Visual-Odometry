#include "VO3D3D.h"
#include "StereoImage.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>
#include "RANSAC.h"


VO3D3D::VO3D3D(FRAlgorithm* fr_algorithm, FMAlgorithm* fm_algorithm, Camera* camera) :
	VOAlgorithm(fr_algorithm, fm_algorithm, camera) {
}

VO3D3D::~VO3D3D(){}

CameraImage* VO3D3D::getNextFrame(ImagePkg* pkg){
	StereoImage* img = new StereoImage();

	//if (m_camera->getProperties().getDistortionCoefsLeft().data != nullptr && m_camera->getProperties().getDistortionCoefsRight().data != nullptr){
	//	Mat l_undistorted;
	//	Mat r_undistorted;
	//	undistort(*pkg->images[0], l_undistorted, m_camera->getCameraMatrix_pixels(), m_camera->getProperties().getDistortionCoefsLeft());
	//	undistort(*pkg->images[1], r_undistorted, m_camera->getCameraMatrix_pixels(), m_camera->getProperties().getDistortionCoefsRight());

	//	*pkg->images[0] = l_undistorted;
	//	*pkg->images[1] = r_undistorted;
	//}

	img->computeFeatures(m_fr_algorithm, m_fm_algorithm, pkg->images[0], pkg->images[1]);
	
	filterBadResultsTriangulation(img);

	img->triangulate(*m_camera);

	return img;
}

Mat VO3D3D::getCameraMovement(CameraImage* t0, CameraImage* t1, ImagePkg* data, ImagePkg* last_data){
	StereoImage* img_t0 = (StereoImage*) t0;
	StereoImage* img_t1 = (StereoImage*) t1;

	//Match keypoints from both images
	std::vector<DMatch> matches = 
		m_fm_algorithm->match(
		img_t0->getImagFeaturesLeft()->getDescriptors(),
		img_t1->getImagFeaturesLeft()->getDescriptors()
		);
	std::vector<DMatch> good_matches;

	filterBadResults(img_t0->getImagFeaturesLeft()->getDescriptors(),
		matches, good_matches);
	
	//Get {X} and {Y}, and validate matches between images
	std::map<long, Point3f> coords_0 = img_t0->getCoordinates();
	std::map<long, Point3f> coords_1 = img_t1->getCoordinates();

	std::vector<Mat> X, Y;
	for (int i = 0; i < good_matches.size(); i++)	{
		std::map<long, Point3f>::iterator it_0 = coords_0.find(good_matches[i].queryIdx);
		std::map<long, Point3f>::iterator it_1 = coords_1.find(good_matches[i].trainIdx);
		if (it_0 == coords_0.end() || it_1 == coords_1.end()){
			//printf("point not found\n");
			continue;
		}
		//if (it_0->second.x >3 || it_0->second.x < -3 || 
		//	it_1->second.x >3 || it_1->second.x < -3){
		//	continue;
		//}
		Mat x(it_0->second, CV_32FC1);
		Mat y(it_1->second, CV_32FC1);
		transpose(x, x);
		transpose(y, y);
		X.push_back(x);
		Y.push_back(y);
	}
	if (X.size() < 5){
		printf("-----No enough features-----\n");
		return Mat::eye(4, 4, CV_32FC1);
	}
		
	//Apply RANSAC

	Mat m = RANSAC::compute(X, Y);
	//Mat m = RANSAC::getTransformationMatrix(X, Y);
		
	return m;
}

void VO3D3D::filterBadResults(Mat* descriptors_0, std::vector<DMatch> matches, std::vector<DMatch>& good_matches){
	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_0->rows; i++)	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	
	for (long i = 0; i < descriptors_0->rows; i++)	{
		if (matches[i].distance <= max(15.f*min_dist, 0.02))	{
			good_matches.push_back(matches[i]);
		}
	}
}

void VO3D3D::filterBadResultsTriangulation(StereoImage* img){
	double max_dist = 0; double min_dist = 100;

	Mat* descriptors_0 = img->getImagFeaturesLeft()->getDescriptors();
	std::vector<DMatch>& matches = img->getMatches();
	std::vector<DMatch>& good_matches = img->getGoodMatches();
	ImageFeatures* f_l = img->getImagFeaturesLeft();
	ImageFeatures* f_r = img->getImagFeaturesRight();

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_0->rows; i++)	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	for (long i = 0; i < descriptors_0->rows; i++)	{
		if (matches[i].distance <= max(40.f*min_dist, 0.02))	{
			//the found keypoints must have similar y coordinates
			//and their x coordinates difference must be no more than 240px(1m)
			//and no less than 2.43px(100m)
			Point2f p_l = f_l->getKeypoints()[matches[i].queryIdx].pt;
			Point2f p_r = f_r->getKeypoints()[matches[i].trainIdx].pt;
			if (abs(p_r.y - p_l.y) > 4 || abs(p_r.x - p_l.x) > 486.4f || abs(p_r.x - p_l.x) < 8.11f){
				//printf("Filtered!: %.2f\n", 0.001f*3.8f*240/(abs(p_r.x - p_l.x)*3.75f*0.001f));
				continue;
			}
			good_matches.push_back(matches[i]);
		}
	}

}