#pragma once
#include "ImageFeatures.h"

class FRAlgorithm{
public:

	FRAlgorithm(void){}

	~FRAlgorithm(void){}

	virtual ImageFeatures* detect(Mat* img){
		return nullptr;
	}
};

