#include "ImageFeatures.h"


ImageFeatures::ImageFeatures(std::vector<KeyPoint>& keypoints,Mat& descriptors){
	m_keypoints = keypoints;
	m_descriptors = descriptors;
}


ImageFeatures::~ImageFeatures(void){

}

