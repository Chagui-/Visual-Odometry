#pragma once
#include "CameraImage.h"
#include "ImageFeatures.h"
//#include "FRSurfGpu.h"

class FMAlgorithm;
class FRAlgorithm;
class Camera;
class StereoImage :public CameraImage{
private:
	ImageFeatures* m_features_l;
	ImageFeatures* m_features_r;
	std::vector<DMatch> m_matches;
	std::vector<DMatch> m_good_matches;
	std::map<long, Point3f> m_coords;

	Point3f triangluatePoint(Camera c, Point2f p1, Point2f p2);
public:
	StereoImage();
	virtual ~StereoImage();

	void triangulate(Camera c);

	void computeFeatures(FRAlgorithm* fr_algorithm, FMAlgorithm* fm_algorithm, Mat* img_l, Mat* img_r);
	//void computeFeaturesGPU(FRSurfGpu* fr_surf_gpu, FMAlgorithm* fm_algorithm, Mat* img_l, Mat* img_r);

	inline ImageFeatures* getImagFeaturesLeft(){
		return m_features_l;
	}
	inline ImageFeatures* getImagFeaturesRight(){
		return m_features_r;
	}
	inline std::map<long,Point3f>& getCoordinates(){
		return m_coords; 
	}
	inline std::vector<DMatch>& getMatches(){
		return m_matches;
	}
	inline std::vector<DMatch>& getGoodMatches(){
		return m_good_matches;
	}
};

