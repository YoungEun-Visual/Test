#include "ofxOpenCv.h"
#include <stdio.h>
#include <math.h>

#pragma once
class AlgorithmEvaluation
{
public:
	AlgorithmEvaluation();
	~AlgorithmEvaluation();

	float* AlgorithmEvaluation::SetPointTime(float duration);
	float AlgorithmEvaluation::CalcPrecision(CvPoint* algorithmpoint, int length);
	float AlgorithmEvaluation::CalcAccuracy(CvPoint correctpoint, CvPoint* algorithmpoint, int length);

	float POINT_TIME = 2.0;
	static const int POINT_NUMBER = 18;

};

