#include "VOMachine.h"
#include "opencv2/core/core.hpp"
#include "CameraImage.h"
#include <ctime>
#include <fstream>

using namespace cv;

VOMachine::VOMachine(VOAlgorithm* algorithm) : m_algorithm(algorithm){
	m_log_file = "";
	m_log = false;
}

VOMachine::~VOMachine(){
	std::list<BufferComponent*>::iterator it = img_buffer.begin();
	for (; it != img_buffer.end(); ++it){
		//imgages needs to be deleted
		delete (*it);
	}
	img_buffer.clear();
}

void VOMachine::reset(){
	std::list<BufferComponent*>::iterator it = img_buffer.begin();
	for (; it != img_buffer.end(); ++it){
		//imgages needs to be deleted
		delete (*it);
	}
	img_buffer.clear();
}

void VOMachine::addImageToBuffer(BufferComponent* buf){
	if (img_buffer.size() > 2){
		//delete oldest image to make space
		delete img_buffer.front();
		img_buffer.pop_front();
	}
	//add image
	img_buffer.push_back(buf);
}


CameraImage* VOMachine::feedNextFrame(ImagePkg* pkg){
	std::clock_t start;
	static Mat last_image_l;
	static Mat last_image_r;
	ImagePkg last_image_frame;
	last_image_frame.images[0] = &last_image_l;
	last_image_frame.images[1] = &last_image_r;

	BufferComponent* buf_img = new BufferComponent();
	buf_img->time_motion_estimation = 0;
	buf_img->time_triangulation = 0;


	start = std::clock();
	//give paquet of data to algorithm, and get CameraImage
	CameraImage* img = m_algorithm->getNextFrame(pkg);
	buf_img->img = img;
	addImageToBuffer(buf_img);

	buf_img->time_triangulation = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	start = std::clock();
	//compute motion if nesesarry
	if (computeMotion(pkg, &last_image_frame)){
		buf_img->time_motion_estimation = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		doLog();
	}

	last_image_l = Mat(*pkg->images[0]);
	last_image_r = Mat(*pkg->images[1]);
	return img;
}

bool VOMachine::computeMotion(ImagePkg* pkg, ImagePkg* last_pkg){
	//static int not_processed = 0;
	//if (not_processed >= 1 ){
		if (img_buffer.size() >= 2){
			std::list<BufferComponent*>::reverse_iterator it = img_buffer.rbegin();
			CameraImage* img_t1 = (*it)->img;
			CameraImage* img_t0 = (*(++it))->img;
			Mat p(4, 1, CV_32FC1, float(0));
			p.at<float>(3) = 1;
			Mat T = m_algorithm->getCameraMovement(img_t0, img_t1,pkg,last_pkg);
			//get current position
			//p.at<float>(0) = (*it)->position.x;
			//p.at<float>(1) = (*it)->position.y;
			//p.at<float>(2) = (*it)->position.z;
			Point3f new_p(0);
			Point3f new_o(0);
			p = T*p;
			Mat o = getOrientationFromT(T);
			new_p.x = p.at<float>(0) + (*it)->position.x;
			new_p.y = p.at<float>(1) + (*it)->position.y;
			new_p.z = p.at<float>(2) + (*it)->position.z;
			new_o.x = o.at<float>(0) + (*it)->orientation.x;
			new_o.y = o.at<float>(1) + (*it)->orientation.y;
			new_o.z = o.at<float>(2) + (*it)->orientation.z;
			printf("-->Position %.2f  %.2f  %.2f", new_p.x, new_p.y, new_p.z);
			printf(" , %.2f  %.2f  %.2f\n\n", new_o.x, new_o.y, new_o.z);
			img_buffer.back()->position = new_p;
			img_buffer.back()->orientation = new_o;
		}
		else{
			return false;
		}
		//not_processed = 1;
		return true;
	//}
	//not_processed++;
	//return false;
}

Mat VOMachine::getOrientationFromT(Mat T){
	Mat out(4, 1, CV_32FC1, float(0));
	out.at<float>(3) = 1;

	float r11 = T.at<float>(0, 0);
	float r21 = T.at<float>(1, 0);
	float r31 = T.at<float>(2, 0);
	float r32 = T.at<float>(2, 1);
	float r33 = T.at<float>(2, 2);

	out.at<float>(0) = atan2(r32, r33);//roll
	out.at<float>(1) = atan2(-r31, sqrt(r32*r32 + r33*r33));//pitch
	out.at<float>(2) = atan2(r21, r11);//yaw
	return out;
}

void VOMachine::newLog(string filename){
	m_log = true;
	m_log_file = filename;
}

void VOMachine::clearLog(){
	//clear output file
	std::ofstream myfile;
	myfile.open("../../images/" + m_log_file);
	myfile << "";
	myfile.close();
}

void VOMachine::printStats(){
	if (img_buffer.size() != 0)
		printf("Last Frame Time (Triangulation): %.1fs\n", img_buffer.back()->time_triangulation);

	if (img_buffer.back()->time_motion_estimation != 0){
		printf("Last Frame Time (Motion): %.1fs\n", img_buffer.back()->time_motion_estimation);
	}
}

void VOMachine::doLog(){
	BufferComponent* buf_img = img_buffer.back();
	if (m_log && m_log_file.size() > 0){
		//write log
		std::ofstream myfile;
		myfile.open("../../images/" + m_log_file, std::ios::app);
		myfile << buf_img->position.x << "\t" << -buf_img->position.y << "\t" << -buf_img->position.z << "\t"
			<< buf_img->orientation.x << "\t" << -buf_img->orientation.y << "\t" << -buf_img->orientation.z << "\t"
			<< buf_img->time_triangulation << "\t" << buf_img->time_motion_estimation << std::endl;
		myfile.close();
	}
}