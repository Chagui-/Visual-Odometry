#pragma once
#include "opencv2/core/core.hpp"
#include <opencv2/features2d/features2d.hpp>

using namespace cv;

class ImageFeatures{
private:
	std::vector<KeyPoint> m_keypoints;
	Mat m_descriptors;

public:
	ImageFeatures(std::vector<KeyPoint>& keypoints,Mat& descriptors);
	~ImageFeatures(void);

	inline std::vector<KeyPoint> getKeypoints(){
		return m_keypoints;
	}

	inline Mat* getDescriptors(){
		return &m_descriptors;
	}
};

