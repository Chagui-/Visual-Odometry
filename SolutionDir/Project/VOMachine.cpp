#include "VOMachine.h"
#include "opencv2/core/core.hpp"
#include "CameraImage.h"
#include <ctime>
#include <fstream>

using namespace cv;

VOMachine::VOMachine(VOAlgorithm* algorithm) : m_algorithm(algorithm){

}

VOMachine::~VOMachine(){
	std::list<BufferComponent*>::iterator it = img_buffer.begin();
	for (; it != img_buffer.end(); ++it){
		//imgages needs to be deleted
		delete (*it);
	}
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
		//write positions
		static std::string file = "1";
		//if (file.size() == 0){
		//	file += (char)getchar();
		//}
		std::ofstream myfile;
		myfile.open("../../images/positions"+file+".txt" , std::ios::app);		
		myfile << buf_img->position.x << "\t" << -buf_img->position.z << "\t" << buf_img->time_triangulation << "\t" << buf_img->time_motion_estimation << std::endl;
		myfile.close();
	}

	last_image_l = Mat(*pkg->images[0]);
	last_image_r = Mat(*pkg->images[1]);
	return img;
}

bool VOMachine::computeMotion(ImagePkg* pkg, ImagePkg* last_pkg){
	static int not_processed = 0;
	if (not_processed >= 1 ){
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
			p = T*p;
			new_p.x = p.at<float>(0) + (*it)->position.x;
			new_p.y = p.at<float>(1) + (*it)->position.y;
			new_p.z = p.at<float>(2) + (*it)->position.z;
			printf("-->Position %.2f  %.2f  %.2f\n\n", p.at<float>(0), p.at<float>(1), p.at<float>(2));
			img_buffer.back()->position = new_p;
		}
		not_processed = 1;
		return true;
	}
	not_processed++;
	return false;
}

void VOMachine::printStats(){
	if (img_buffer.size() != 0)
	printf("Last Frame Time (Triangulation): %.1fs\n", img_buffer.back()->time_triangulation);
	if (img_buffer.back()->time_motion_estimation != 0)
		printf("Last Frame Time (Motion): %.1fs\n", img_buffer.back()->time_motion_estimation);
}
