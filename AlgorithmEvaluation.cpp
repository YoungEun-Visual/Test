#include "AlgorithmEvaluation.h"



AlgorithmEvaluation::AlgorithmEvaluation()
{
}


AlgorithmEvaluation::~AlgorithmEvaluation()
{
}

//���󿡼� �� point�� ������ �ð� ���
float* AlgorithmEvaluation::SetPointTime(float duration)
{
	float point_time[2*POINT_NUMBER];
	float end_time = duration;
	float start_time = end_time - POINT_TIME;
	for (int i = POINT_NUMBER - 1; i >= 0; i--) {
		point_time[2 *i + 1] = end_time - 0.5; //0.5�� ����
		point_time[2 *i] = start_time + 0.5;
		end_time -= POINT_TIME;
		start_time -= POINT_TIME;
	}
	return point_time;
}

//���е� ��� by ǥ������
float AlgorithmEvaluation::CalcPrecision(CvPoint* algorithmpoint, int length) {

	float x_total = 0, y_total = 0, total = 0;
	CvPoint average;
	for (int i = 0; i < length; i++) {
		x_total += algorithmpoint[i].x;
		y_total += algorithmpoint[i].y;
	}
	average.x = x_total / length;
	average.y = y_total / length;

	for (int i = 0; i < length; i++) {
		total += pow(algorithmpoint[i].x - average.x, 2) + pow(algorithmpoint[i].y - average.y, 2);
	}
	return sqrt(total / length);
}

//��Ȯ�� ��� by ���
float AlgorithmEvaluation::CalcAccuracy(CvPoint correctpoint, CvPoint* algorithmpoint, int length) {
	
	float x_total = 0, y_total = 0, diff = 0;
	CvPoint average;
	for (int i = 0; i < length; i++) {
		x_total += algorithmpoint[i].x;
		y_total += algorithmpoint[i].y;
	}
	average.x = x_total / length;
	average.y = y_total / length;

	diff = sqrt(pow(correctpoint.x - average.x, 2) + pow(correctpoint.y - average.y, 2));

	return diff;
}