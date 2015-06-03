#include <stdio.h>
#include <Windows.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2\imgproc\imgproc.hpp>
#include <fstream>

#include "FRSurf.h"
#include "FMFlann.h"
#include "Camera.h"
#include "VOMachine.h"
#include "VO3D3D.h"
#include "StereoImage.h"
#include "RANSAC.h"

using namespace cv;

void computePairOfImages(Mat img_t0, Mat img_t1);
void drawTextWithShadow(Mat& mat, String txt, Point origin);
int doWork(string search_path, string images_path, ImagePkg* pkg, VOMachine* vom, int iteration, int test_number);

int main(int argc, char** argv){
	//Variable declaration
	Mat img_1, img_2;

	//suscribe to ros topic for images
	// 	
	
	FRSurf fr_surf(500);
	FMFlann fm_flann;
	Camera camera("BumbleBee XB3 13S2C-38", 3.8f, 3.75f, 1280, 960, 1280 / 2, 960/2, 240.f);
	VO3D3D vo_3d3d(&fr_surf, &fm_flann, &camera);
	RANSAC::initialize();

	Mat camera_matrix = Mat::eye(3, 3, CV_32FC1);
	//camera_matrix.at<float>(0, 0) =  3.7393f;//fx
	//camera_matrix.at<float>(1, 1) =  3.7393f;//fy
	camera_matrix.at<float>(0, 0) =  997.17f;//fx
	camera_matrix.at<float>(1, 1) =  997.17f;//fy
	camera_matrix.at<float>(0, 2) = 667.57f,//cx
	camera_matrix.at<float>(1, 2) = 496.31f;//cy

	camera.setCameraMatrix_pixels(camera_matrix);

	Mat dist_coefs_left = Mat::zeros(8, 1, CV_32FC1);
	Mat dist_coefs_right = Mat::zeros(8, 1, CV_32FC1);
	dist_coefs_left.at<float>(0) = -0.177067f;
	dist_coefs_left.at<float>(1) = -0.165155f;
	dist_coefs_left.at<float>(2) = 0.000571f;
	dist_coefs_left.at<float>(3) = 0.000539f;
	dist_coefs_left.at<float>(4) = -0.142561f;
	dist_coefs_left.at<float>(5) = 0.192250f;
	dist_coefs_left.at<float>(6) = -0.316417f;
	dist_coefs_left.at<float>(7) = -0.196546f;

	dist_coefs_right.at<float>(0) = 0.746517f;
	dist_coefs_right.at<float>(1) = -0.321874f;
	dist_coefs_right.at<float>(2) = 0.000959f;
	dist_coefs_right.at<float>(3) = 0.000514f;
	dist_coefs_right.at<float>(4) = -0.788330f;
	dist_coefs_right.at<float>(5) = 1.099655f;
	dist_coefs_right.at<float>(6) = -0.043691f;
	dist_coefs_right.at<float>(7) = -1.203787f;

	camera.getProperties().setDistortionCoefsLeft(dist_coefs_left);
	camera.getProperties().setDistortionCoefsRight(dist_coefs_right);



	VOMachine vom(&vo_3d3d);	

	ImagePkg pkg;
	pkg.images[0] = &img_1;
	pkg.images[1] = &img_2;
	std::string images_path = "..\\..\\images\\outdir_27-28";
	std::string search_path = images_path + "\\left*.jpg";

	int repetitions = 3;
	int test_number = 0;
	
	if (argc >= 2)
		test_number = atoi(argv[1]);

	switch (test_number)
	{
	case 0:
	case 4:
		RANSAC::initialize(200, 0.07f);
		break;
	case 1:
	case 5:
		RANSAC::initialize(400, 0.05f);
		break;
	case 2:
	case 6:
		RANSAC::initialize(800, 0.03f);
		break;
	case 3:
	case 7:
		RANSAC::initialize(1600, 0.01f);
		break;
	}

	if (argc >= 3)
		repetitions = atoi(argv[2]);

	for (int i = 0; i < repetitions; i++)
	{		
		char log_filename[100];
		sprintf(log_filename, "log_%u_%u.txt", test_number, i);

		vom.newLog(log_filename);
		vom.clearLog();
		if (test_number >= 4){
			images_path = "..\\..\\images\\outdir_28-29";
			search_path = images_path + "\\left*.jpg";
		}
		
		doWork(search_path, images_path, &pkg, &vom, i + 1, test_number);
		vom.reset();
	}
	

	//computePairOfImages(img_1, img_2);
	
	//waitKey(0);
	printf("FINISH\n");
	getchar();

	return 0;
}

int doWork(string search_path, string images_path, ImagePkg* pkg, VOMachine* vom, int iteration, int test_number){
	Mat* img_1 = pkg->images[0];
	Mat* img_2 = pkg->images[1];
	static int n = 0;
	n = 0;
	WIN32_FIND_DATA file;
	HANDLE search_handle =
		FindFirstFile(std::wstring(search_path.begin(), search_path.end()).c_str(), &file);
	if (search_handle)
	{

		do
		{
			if (file.cFileName[0] == '.'){
				continue;
			}
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
			*img_1 = imread(file, CV_LOAD_IMAGE_GRAYSCALE);
			temp = images_path + "/right_%s.jpg";
			sprintf(file, temp.c_str(), timestamp.c_str());
			*img_2 = imread(file, CV_LOAD_IMAGE_GRAYSCALE);
			printf("Procesing file: %i, iteration: %i \n", n, iteration);
			//printf("Procesing file: %i,   %s, iteration: %i \n", n, timestamp.c_str(), iteration);

			if (!(*img_1).data || !(*img_2).data)
			{
				std::cout << " --(!) Error reading images " << std::endl;
				continue;
			}
			//equalizeHist(*img_1, *img_1);
			//equalizeHist(*img_2, *img_2);

			//int scale_1 = 80;
			//int method_1 = scale_1 / 100.f < 1 ? INTER_AREA : INTER_CUBIC;
			//resize(img_1, img_1, Size(), scale_1 / 100.f, scale_1 / 100.f, method_1);
			//resize(img_2, img_2, Size(), scale_1 / 100.f, scale_1 / 100.f, method_1);

			//camera.getProperties().setCameraResolution(img_1.size().width, img_1.size().height);

			StereoImage* img = (StereoImage*)vom->feedNextFrame(pkg);

			//print stats
			//vom->printStats();

		} while (FindNextFile(search_handle, &file));
		CloseHandle(search_handle);

	}
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
