#pragma once
#include "ImagePkg.h"
#include "VOAlgorithm.h"
#include <list>

struct BufferComponent{
	CameraImage* img;
	double time_triangulation;
	double time_motion_estimation;
	Point3f position;
	Point3f orientation;
	Mat T_matrix;
	
	BufferComponent() : img(nullptr), time_triangulation(0), time_motion_estimation(0), position(0), orientation(0){}

	~BufferComponent(){
		delete img;
	}
};

class CameraImage;
class VOMachine{
protected:
	bool m_log;
	string m_log_file;

	VOAlgorithm* m_algorithm;
	std::list<BufferComponent*> img_buffer;

	bool computeMotion(ImagePkg* pkg, ImagePkg* last_pkg);
	void addImageToBuffer(BufferComponent* buf);
	Mat getOrientationFromT(Mat T);
	void doLog();
public:
	void newLog(string filename);
	inline bool getIsLogging(){
		return m_log;
	}
	void clearLog();
	void reset();
	

	VOMachine(VOAlgorithm* algorithm);
	~VOMachine();

	CameraImage* feedNextFrame(ImagePkg* data);
	void printStats();
};

