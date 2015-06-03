#pragma once

class CameraProperties{
private:
	float m_distance_between_lenses_mm;
	bool m_is_stereo;
	float m_focal_length_x_mm;
	float m_focal_length_y_mm;
	float m_pixel_size_um;
	String m_camera_model;
	int m_resolution_x;
	int m_resolution_y;
	int m_center_x;
	int m_center_y;
	Mat m_distortion_coefs_left;
	Mat m_distortion_coefs_right;

	inline void setDistanceBetweenLenses_mm(float distance_between_lenses_mm){
		m_distance_between_lenses_mm = distance_between_lenses_mm;
		if (distance_between_lenses_mm == 0)
			m_is_stereo = false;
		else
			m_is_stereo = true;
	}
public:

	CameraProperties(String camera_model, float focal_length_mm, float pixel_size_um, float distance_between_lenses_mm, int resolution_x, int resolution_y, int center_x, int center_y) :
		m_focal_length_x_mm(focal_length_mm),
		m_focal_length_y_mm(focal_length_mm),
		m_resolution_x(resolution_x),
		m_resolution_y(resolution_y),
		m_center_x(center_x),
		m_center_y(center_y),
		m_pixel_size_um(pixel_size_um){
		setDistanceBetweenLenses_mm(distance_between_lenses_mm);
		m_camera_model = camera_model;
	}

	CameraProperties(String camera_model,float pixel_size_um, float distance_between_lenses_mm, int resolution_x, int resolution_y,Mat camera_matrix) :
		m_resolution_x(resolution_x),
		m_resolution_y(resolution_y),
		m_pixel_size_um(pixel_size_um){
		setDistanceBetweenLenses_mm(distance_between_lenses_mm);
		m_camera_model = camera_model;
		m_focal_length_x_mm = 1;
		m_focal_length_y_mm = 1;
		m_center_x = 1;
		m_center_y = 1;
	}

	~CameraProperties(){}

	inline Mat getDistortionCoefsLeft(){
		return m_distortion_coefs_left;
	}

	inline void setDistortionCoefsLeft(Mat values){
		m_distortion_coefs_left = values;
	}

	inline Mat getDistortionCoefsRight(){
		return m_distortion_coefs_right;
	}

	inline void setDistortionCoefsRight(Mat values){
		m_distortion_coefs_right = values;
	}
	
	inline float getDistanceBetweenLenses_mm(){
		return m_distance_between_lenses_mm;
	}
	inline Point2f getFocalLength_mm(){
		return Point2f(m_focal_length_x_mm, m_focal_length_y_mm);
	}
	inline void setFocalLength_mm(float x, float y){
		m_focal_length_x_mm = x;
		m_focal_length_y_mm = y;
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
	inline void setCameraCenter(int x, int y){
		m_center_x = x;
		m_center_y = y;
	}
	inline Point2i getCameraResolution(){
		return Point2i(m_resolution_x, m_resolution_y);
	}
	inline Point2i getCameraCenter(){
		return Point2i(m_center_x, m_center_y);
	}
};

