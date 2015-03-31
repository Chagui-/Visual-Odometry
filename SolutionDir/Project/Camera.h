#pragma once
#include "CameraProperties.h"

class Camera{
private:
	CameraProperties m_properties;
	//Point3f n_start_pose;

public:
	Camera(String camera_model, float focal_length_mm, float pixel_size_um, int resolution_x, int resolution_y, float distance_between_lenses_mm = 0) :
		m_properties(camera_model, focal_length_mm, pixel_size_um, distance_between_lenses_mm, resolution_x, resolution_y){}

	~Camera(){}

	CameraProperties& getProperties(){
		return m_properties;
	}

	inline Point2f pixels2mm(Point2f p){
		return Point2f(
			(p.x - getProperties().getCameraResolution().x / 2) * m_properties.getPixelSize_um() * 0.001f,
			(p.y - getProperties().getCameraResolution().y / 2)  * m_properties.getPixelSize_um() * 0.001f
		);
	}

	
};

