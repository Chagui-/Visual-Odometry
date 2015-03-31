#include <stdio.h>
#include <Windows.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <fstream>

#include "FRSurf.h"
#include "FMFlann.h"
#include "Camera.h"
#include "VOMachine.h"
#include "VO3D3D.h"
#include "StereoImage.h"

using namespace cv;

void computePairOfImages(Mat img_t0, Mat img_t1);
void drawTextWithShadow(Mat& mat, String txt, Point origin);

int main(int argc, char** argv){
	//Variable declaration
	Mat img_1, img_2;

	//suscribe to ros topic for images
	// 	

	FRSurf fr_surf(500);
	FMFlann fm_flann;
	Camera camera("BumbleBee XB3 13S2C-38", 3.8f, 3.75f, 1280, 960, 240.f);
	VO3D3D vo_3d3d(&fr_surf,&fm_flann,camera);


	VOMachine vom(&vo_3d3d);

	ImagePkg pkg;
	pkg.images[0] = &img_1;
	pkg.images[1] = &img_2;

	std::string images_path = "..\\..\\images\\outdir_28-29";
	std::string search_path = images_path + "\\left*.jpg";
	WIN32_FIND_DATA file;
	HANDLE search_handle =  
		FindFirstFile(std::wstring(search_path.begin(), search_path.end()).c_str(), &file);
	if (search_handle)
	{
		//clear output file
		std::ofstream myfile;
		myfile.open("../../images/positions1.txt");
		myfile << "";
		myfile.close();
		myfile.open("../../images/positions2.txt");
		myfile << "";
		myfile.close();
		myfile.open("../../images/positions3.txt");
		myfile << "";
		myfile.close();
		do
		{
			if (file.cFileName[0] == '.'){
				continue;
			}
			static int n = 0;
			n++;
			if (n % 2 == 0)
				continue;

			std::wstring filename_w(file.cFileName);
			std::string filename(filename_w.begin(), filename_w.end());
			int timestamp_length = filename.find(".") - filename.find("_") - 1;
			std::string timestamp = filename.substr(filename.find("_") + 1, timestamp_length);

			//TODO: get frames left and right from ros topic
			char file[100];
			std::string temp = images_path + "/left_%s.jpg";
			sprintf(file, temp.c_str(), timestamp.c_str());
			img_1 = imread(file, CV_LOAD_IMAGE_GRAYSCALE);
			temp = images_path + "/right_%s.jpg";
			sprintf(file, temp.c_str(), timestamp.c_str());
			img_2 = imread(file, CV_LOAD_IMAGE_GRAYSCALE);
			printf("Procesing files: %i,   %s \n", n, timestamp.c_str());

			if (!img_1.data || !img_2.data)
			{
				std::cout << " --(!) Error reading images " << std::endl; return -1;
			}
			//int scale_1 = 80;
			//int method_1 = scale_1 / 100.f < 1 ? INTER_AREA : INTER_CUBIC;
			//resize(img_1, img_1, Size(), scale_1 / 100.f, scale_1 / 100.f, method_1);
			//resize(img_2, img_2, Size(), scale_1 / 100.f, scale_1 / 100.f, method_1);

			camera.getProperties().setCameraResolution(img_1.size().width, img_1.size().height);

			StereoImage* img = (StereoImage*)vom.feedNextFrame(&pkg);

			//Mat img_matches = *pkg.images[0];
			//cvtColor(img_matches, img_matches, CV_GRAY2RGB);

			////drawMatches(img_1, img->getImagFeaturesLeft()->getKeypoints(), img_2, img->getImagFeaturesRight()->getKeypoints(),
			////	img->getGoodMatches(), img_matches, Scalar::all(-1), Scalar::all(-1),
			////	vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

			//char txt_1[50];
			//char txt_2[50];
			//char txt_3[50];
			//std::map<long, Point3f> coords = img->getCoordinates();
			////get max depth
			//float max_depth = 0;
			//for (std::map<long, Point3f>::iterator it2 = coords.begin(); it2 != coords.end(); ++it2){
			//	max_depth = max(it2->second.z, max_depth);
			//}
			////printf("-->Max depth of frame: %.0f\n", max_depth);
			//for (int i = 0; i < img->getGoodMatches().size(); i++){
			//	Point3f point_3d = img->getCoordinates()[img->getGoodMatches()[i].queryIdx];
			//	Point2f point_2d = img->getImagFeaturesLeft()->getKeypoints()[img->getGoodMatches()[i].queryIdx].pt;
			//	Point2f point_2d_r = img->getImagFeaturesRight()->getKeypoints()[img->getGoodMatches()[i].trainIdx].pt;
			//	float disparity = point_2d.x - point_2d_r.x;

			//	//sprintf(txt_1, "%.2fm, %.2fm, %.1fm", point_3d.x, point_3d.y, point_3d.z);
			//	//sprintf(txt_2, "(%.0f, %.0f)", img_f0.getKeypoints()[good_matches[i].queryIdx].pt.x, img_f0.getKeypoints()[good_matches[i].queryIdx].pt.y);
			//	sprintf(txt_3, "%.0fm", point_3d.z);
			//	//sprintf(txt_3, "%.1f", disparity);
			//	//(img_matches, txt_1, point_2d + Point2f(11, 8));
			//	if (point_3d.z < 0)
			//		circle(img_matches, point_2d, 3, Scalar(0, 0, 255), 3);
			//	else{

			//		circle(img_matches, point_2d, 3, Scalar(0, 255, 0), 3);
			//	}
			//}
			////for (int i = 0; i < img->getGoodMatches().size(); i++){
			////	Point3f point_3d = img->getCoordinates()[img->getGoodMatches()[i].queryIdx];
			////	Point2f point_2d = img->getImagFeaturesLeft()->getKeypoints()[img->getGoodMatches()[i].queryIdx].pt;
			////	Point2f point_2d_r = img->getImagFeaturesRight()->getKeypoints()[img->getGoodMatches()[i].trainIdx].pt;
			////	float disparity = point_2d.x - point_2d_r.x;

			////	//sprintf(txt_1, "%.2fm, %.2fm, %.1fm", point_3d.x, point_3d.y, point_3d.z);
			////	//sprintf(txt_2, "(%.0f, %.0f)", img_f0.getKeypoints()[good_matches[i].queryIdx].pt.x, img_f0.getKeypoints()[good_matches[i].queryIdx].pt.y);
			////	sprintf(txt_3, "%.0fm", point_3d.z);
			////	//sprintf(txt_3, "%.1f", disparity);
			////	//(img_matches, txt_1, point_2d + Point2f(11, 8));
			////	if ((rand() % 10) >= 8)
			////		drawTextWithShadow(img_matches, txt_3, point_2d + Point2f(10, 5));
			////}

			//int scale = 70;
			//int method = scale / 100.f < 1 ? INTER_AREA : INTER_CUBIC;
			//resize(img_matches, img_matches, Size(), scale / 100.f, scale / 100.f, method);
			//imshow("Left Image", img_matches);
			//waitKey(0);

			//print stats
			vom.printStats();

		} while (FindNextFile(search_handle, &file));
		CloseHandle(search_handle);

	}

	//computePairOfImages(img_1, img_2);
	
	//waitKey(0);
	printf("FINISH\n");
	getchar();

	return 0;
}

void drawTextWithShadow(Mat& mat, String txt, Point origin){
	float scale = 0.7f;
	putText(mat, txt, origin + Point(-2, 0), FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 0, 0), 2,CV_AA);
	putText(mat, txt, origin + Point(2, 0), FONT_HERSHEY_SIMPLEX , scale, Scalar(0, 0, 0), 2,CV_AA);
	putText(mat, txt, origin + Point(0, 2), FONT_HERSHEY_SIMPLEX , scale, Scalar(0, 0, 0), 2,CV_AA);
	putText(mat, txt, origin + Point(0, -2), FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 0, 0), 2,CV_AA);

	putText(mat, txt, origin + Point(0, 0), FONT_HERSHEY_SIMPLEX, scale, Scalar(255, 255, 255), 2,CV_AA);
}
