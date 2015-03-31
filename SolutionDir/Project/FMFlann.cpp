#include "FMFlann.h"


FMFlann::FMFlann(){
}

FMFlann::~FMFlann(){}

std::vector<DMatch> FMFlann::match(Mat* queryDescriptors, Mat* trainDescriptors){
	std::vector<DMatch> matches;
	m_matcher.match(*queryDescriptors, *trainDescriptors, matches);
	return matches;
}
