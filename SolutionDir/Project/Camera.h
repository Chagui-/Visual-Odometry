#pragma once
#include "CameraProperties.h"

class Camera{
private:
	CameraProperties m_properties;
	//Point3f n_start_pose;

public:
	Camera(String camera_model, float focal_length_mm, float pixel_size_um, int resolution_x, int resolution_y, int center_x, int center_y, float distance_between_lenses_mm = 0) :
		m_properties(camera_model, focal_length_mm, pixel_size_um, distance_between_lenses_mm, resolution_x, resolution_y, center_x, center_y){}

	Camera(String camera_model, float pixel_size_um, int resolution_x, int resolution_y, Mat camera_matrix, float distance_between_lenses_mm = 0) :
		m_properties(camera_model, pixel_size_um, distance_between_lenses_mm, resolution_x, resolution_y,camera_matrix){}

	~Camera(){}

	CameraProperties& getProperties(){
		return m_properties;
	}

	inline Point2f pixels2mm(Point2f p){
		return Point2f(
			(p.x - getProperties().getCameraCenter().x) * m_properties.getPixelSize_um() * 0.001f,
			(p.y - getProperties().getCameraCenter().y)  * m_properties.getPixelSize_um() * 0.001f
		);
	}

	inline Mat getCameraMatrix_pixels(){
		Mat output = Mat::eye(3, 3, CV_32FC1);
		output.at<float>(0, 0) = (float)getProperties().getFocalLength_mm().x / 0.00375;//fx
		output.at<float>(1, 1) = (float)getProperties().getFocalLength_mm().y / 0.00375;//fy
		output.at<float>(0, 2) = (float)getProperties().getCameraCenter().x;//cx
		output.at<float>(1, 2) = (float)getProperties().getCameraCenter().y;//cy
		return output;
	}
	inline Mat getCameraMatrix_mm(){
		Mat output = Mat::eye(3, 3, CV_32FC1);
		output.at<float>(0, 0) = (float)getProperties().getFocalLength_mm().x;//fx
		output.at<float>(1, 1) = (float)getProperties().getFocalLength_mm().y;//fy
		output.at<float>(0, 2) = (float)getProperties().getCameraCenter().x;//cx
		output.at<float>(1, 2) = (float)getProperties().getCameraCenter().y;//cy
		return output;
	}

	inline void setCameraMatrix_pixels(Mat camera_matrix){
		m_properties.setFocalLength_mm(camera_matrix.at<float>(0, 0)* 0.00375, camera_matrix.at<float>(1, 1) * 0.00375);
		m_properties.setCameraCenter((int)camera_matrix.at<float>(0,2), (int)camera_matrix.at<float>(1, 2));
	}

	inline void setCameraMatrix_mm(Mat camera_matrix){
		m_properties.setFocalLength_mm(camera_matrix.at<float>(0, 0), camera_matrix.at<float>(1, 1));
		m_properties.setCameraCenter((int)camera_matrix.at<float>(0, 2), (int)camera_matrix.at<float>(1, 2));
	}

	
};

