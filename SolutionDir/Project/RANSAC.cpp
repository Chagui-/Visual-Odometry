#include "RANSAC.h"
#include <ctime>

int myrandom(int i) { return std::rand() % i; }

int RANSAC::m_max_iterations = 400;
float RANSAC::m_margin = 0.1f;

void RANSAC::initialize(int max_iterations, float margin){
	m_max_iterations = max_iterations;
	m_margin = margin;
}

Mat RANSAC::compute(std::vector<Mat>& X, std::vector<Mat>& Y){
	static int num_points = 5;
	Mat T = Mat::eye(4, 4, CV_32FC1);
	Mat best_T = Mat::eye(4, 4, CV_32FC1);

	int max_inliers = 0;
	std::srand(unsigned(std::time(0)));

	std::vector<int> indices;
	for (int i = 0; i < X.size(); ++i) indices.push_back(i);
	for (int i = 0; i < m_max_iterations; i++){
		//get random points
		std::random_shuffle(indices.begin(), indices.end(), myrandom);
		std::vector<Mat> new_X;
		std::vector<Mat> new_Y;
		for (int j = 0; j < num_points; ++j){
			new_X.push_back(X[indices[j]]);
			new_Y.push_back(Y[indices[j]]);
		}
		//compute T
		T = getTransformationMatrix(new_X, new_Y);
		int inliers = countInliers(T, X, Y, m_margin);

		if (inliers > max_inliers){
			best_T = T;
			max_inliers = inliers;
		}
	}
	static float percentage_outliers = 0;
	percentage_outliers = (percentage_outliers + (X.size() - max_inliers)* 100.f / X.size()) / 2;
	printf("Ouliers(Accumulated): %.2f%%		Inliers: %i \n", percentage_outliers, max_inliers);
	return best_T;
}

Mat RANSAC::getTransformationMatrix(std::vector<Mat>& X, std::vector<Mat>& Y){
	//calculate centroids
	Mat u_x(1, 3, CV_32FC1, float(0));
	Mat u_y(1, 3, CV_32FC1, float(0));
	for (int i = 0; i < X.size(); i++)	{
		u_x += X[i];
		u_y += Y[i];
	}
	u_x /= (float)X.size();
	u_y /= (float)X.size();

	//Calculate E_xy covariance matrix
	Mat E_xy(3, 3, CV_32FC1, double(0));
	for (int i = 0; i < X.size(); i++)	{
		Mat a = (Y[i] - u_y);
		Mat b;
		transpose((X[i] - u_x), b);
		E_xy += b*a;
	}
	E_xy /= X.size();

	//Calculate SVD(E)
	Mat w, U, Vt;
	SVD::compute(E_xy, w, U, Vt, SVD::MODIFY_A);
	//Find S
	Mat S;
	double det_U = determinant(U);
	double det_Vt = determinant(Vt);
	if (det_U*det_Vt > 0){
		S = Mat::eye(3, 3, CV_32FC1);
	}
	else{
		S = Mat::eye(3, 3, CV_32FC1);
		S.at<float>(2, 2) = -1;
	}

	//Calculate Rotation and translation
	Mat V, Ut, Rt;
	transpose(Vt, V);
	transpose(U, Ut);
	Mat R = V*S*Ut;
	//Mat R = U*S*Vt;
	//transpose(R, Rt);
	transpose(u_x, u_x);
	Mat temp = R*u_x;
	transpose(temp, temp);
	Mat t = u_y - temp;
	transpose(t, t);

	//printf("E %.2f\t%.2f\t%.2f\n", E_xy.at<float>(0, 0), E_xy.at<float>(0, 1), E_xy.at<float>(0, 2));
	//printf("E %.2f\t%.2f\t%.2f\n", E_xy.at<float>(1, 0), E_xy.at<float>(1, 1), E_xy.at<float>(1, 2));
	//printf("E %.2f\t%.2f\t%.2f\n", E_xy.at<float>(2, 0), E_xy.at<float>(2, 1), E_xy.at<float>(2, 2));
	//printf("R %.2f\t%.2f\t%.2f\n", R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2));
	//printf("R %.2f\t%.2f\t%.2f\n", R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2));
	//printf("R %.2f\t%.2f\t%.2f\n", R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
	//printf("\n");
	//printf("U %.2f\t%.2f\t%.2f\n", U.at<float>(0, 0), U.at<float>(0, 1), U.at<float>(0, 2));
	//printf("U %.2f\t%.2f\t%.2f\n", U.at<float>(1, 0), U.at<float>(1, 1), U.at<float>(1, 2));
	//printf("U %.2f\t%.2f\t%.2f\n", U.at<float>(2, 0), U.at<float>(2, 1), U.at<float>(2, 2));
	//printf("\n");
	//printf("Vt %.2f\t%.2f\t%.2f\n", Vt.at<float>(0, 0), Vt.at<float>(0, 1), Vt.at<float>(0, 2));
	//printf("Vt %.2f\t%.2f\t%.2f\n", Vt.at<float>(1, 0), Vt.at<float>(1, 1), Vt.at<float>(1, 2));
	//printf("Vt %.2f\t%.2f\t%.2f\n", Vt.at<float>(2, 0), Vt.at<float>(2, 1), Vt.at<float>(2, 2));
	//printf("\n");

	//printf("t %.2f\n", t.at<float>(0));
	//printf("t %.2f\n", t.at<float>(1));
	//printf("t %.2f\n", t.at<float>(2));
	//printf("\n");

	//Get Transformation matrix
	Mat m = Mat::eye(4, 4, CV_32FC1);
	R.copyTo(m(Rect(0, 0, 3, 3)));
	Mat aux_t = m(Rect(3, 0, 1, 3));
	t.copyTo(aux_t);

	//printf("T %.3f\t%.3f\t%.3f\t%.3f\n", m.at<float>(0, 0), m.at<float>(0, 1), m.at<float>(0, 2), m.at<float>(0, 3));
	//printf("T %.3f\t%.3f\t%.3f\t%.3f\n", m.at<float>(1, 0), m.at<float>(1, 1), m.at<float>(1, 2), m.at<float>(1, 3));
	//printf("T %.3f\t%.3f\t%.3f\t%.3f\n", m.at<float>(2, 0), m.at<float>(2, 1), m.at<float>(2, 2), m.at<float>(2, 3));
	//printf("T %.3f\t%.3f\t%.3f\t%.3f\n", m.at<float>(3, 0), m.at<float>(3, 1), m.at<float>(3, 2), m.at<float>(3, 3));
	//printf("\n");


	//final movement Y_i = T*X_i
	return m;
}

int RANSAC::countInliers(Mat T, std::vector<Mat>& X, std::vector<Mat>& Y, float margin){
	int num_inliers = 0;
	for (int i = 0; i < X.size(); i++){
		Mat y = Mat::ones(4, 1, CV_32FC1);
		y.at<float>(0) = Y[i].at<float>(0);
		y.at<float>(1) = Y[i].at<float>(1);
		y.at<float>(2) = Y[i].at<float>(2);
		Mat x = Mat::ones(4, 1, CV_32FC1);
		x.at<float>(0) = X[i].at<float>(0);
		x.at<float>(1) = X[i].at<float>(1);
		x.at<float>(2) = X[i].at<float>(2);
		Mat new_y = T*x;
		double distance = norm(y - new_y);
		if (distance < margin){
			num_inliers++;
		}
	}
	return num_inliers;
}