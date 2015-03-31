#pragma once
#include "FRAlgorithm.h"
#include "FMAlgorithm.h"
#include "Camera.h"
#include "CameraImage.h"
#include "ImagePkg.h"

class VOAlgorithm{
protected:
	FRAlgorithm* m_fr_algorithm;
	FMAlgorithm* m_fm_algorithm;
	Camera m_camera;

public:

	VOAlgorithm(FRAlgorithm* fr_algorithm, FMAlgorithm* fm_algorithm, Camera camera):
		m_camera(camera), m_fr_algorithm(fr_algorithm), m_fm_algorithm(fm_algorithm){
	}

	~VOAlgorithm(){}

	virtual CameraImage* getNextFrame(ImagePkg* pkg){
		return nullptr;
	}

	virtual Mat getCameraMovement(CameraImage* t0, CameraImage* t1, ImagePkg* data, ImagePkg* last_data){
		return Mat(1,1,CV_32FC1);
	}
};

