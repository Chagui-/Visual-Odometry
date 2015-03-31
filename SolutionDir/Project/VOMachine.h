#pragma once
#include "ImagePkg.h"
#include "VOAlgorithm.h"
#include <list>

struct BufferComponent{
	CameraImage* img;
	double time_triangulation;
	double time_motion_estimation;
	Point3f position;
	Mat T_matrix;
	
	BufferComponent() : img(nullptr), time_triangulation(0), time_motion_estimation(0),position(0){}

	~BufferComponent(){
		delete img;
	}
};

class CameraImage;
class VOMachine{
protected:

	VOAlgorithm* m_algorithm;
	std::list<BufferComponent*> img_buffer;

	bool computeMotion(ImagePkg* pkg, ImagePkg* last_pkg);
	void addImageToBuffer(BufferComponent* buf);
public:

	

	VOMachine(VOAlgorithm* algorithm);
	~VOMachine();

	CameraImage* feedNextFrame(ImagePkg* data);
	void printStats();
};

