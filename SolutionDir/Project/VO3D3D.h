#pragma once
#include "VOAlgorithm.h"

class StereoImage;
class VO3D3D :	public VOAlgorithm{
private:

	void filterBadResults(Mat* descriptors_0, std::vector<DMatch> matches,
		std::vector<DMatch>& good_matches);
	void filterBadResultsTriangulation(StereoImage* img);

public:
	VO3D3D(FRAlgorithm* fr_algorithm, FMAlgorithm* fm_algorithm, Camera camera);
	~VO3D3D();

	CameraImage* getNextFrame(ImagePkg* pkg);

	Mat getCameraMovement(CameraImage* t0, CameraImage* t1, ImagePkg* data, ImagePkg* last_data);
};

