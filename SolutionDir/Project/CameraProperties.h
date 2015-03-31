#pragma once

class CameraProperties{
private:
	float m_distance_between_lenses_mm;
	bool m_is_stereo;
	float m_focal_length_mm;
	float m_pixel_size_um;
	String m_camera_model;
	int m_resolution_x;
	int m_resolution_y;

	inline void setDistanceBetweenLenses_mm(float distance_between_lenses_mm){
		m_distance_between_lenses_mm = distance_between_lenses_mm;
		if (distance_between_lenses_mm == 0)
			m_is_stereo = false;
		else
			m_is_stereo = true;
	}
public:

	CameraProperties(String camera_model, float focal_length_mm, float pixel_size_um, float distance_between_lenses_mm, int resolution_x, int resolution_y) :
		m_focal_length_mm(focal_length_mm),
		m_resolution_x(resolution_x),
		m_resolution_y(resolution_y),
		m_pixel_size_um(pixel_size_um){
		setDistanceBetweenLenses_mm(distance_between_lenses_mm);
		m_camera_model = camera_model;
	}

	~CameraProperties(){}

	
	inline float getDistanceBetweenLenses_mm(){
		return m_distance_between_lenses_mm;
	}
	inline float getFocalLength_mm(){
		return m_focal_length_mm;
	}
	inline float getPixelSize_um(){
		return m_pixel_size_um;
	}
	inline String getcamera_model(){
		return m_camera_model;
	}
	inline bool isStereo(){
		return m_is_stereo;
	}
	inline void setCameraResolution(int x, int y){
		m_resolution_x = x;
		m_resolution_y = y;
	}
	inline Point2i getCameraResolution(){
		return Point2i(m_resolution_x, m_resolution_y);
	}
};

