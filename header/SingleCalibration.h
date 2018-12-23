#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <Windows.h>

#include "features2d.hpp"
#include "core.hpp"
#include "calib3d\calib3d.hpp"
#include "highgui.hpp"
#include "imgcodecs.hpp"
#include "imgproc.hpp"
#include "videoio.hpp"

#include "source.h"
#include "resource.h"


using namespace std;
using namespace cv;

class SingleCalibration
{
private:
	float squareSize;			// ������ ������� ��������/������� �����?
	bool useFisheye;			// ������������� ����� - ������
	bool loadOptions;			// True, ���� ���������� ��������� � �-�� ��������� ��������

	vector<vector<Point3f>> objectPoints;		// �������� ������� ��������� ����� �� ���� ������������ � �������� ����
	vector<vector<Point2f>> imagePoints;		// ���������� ����� �� ���� ����������� �� ������������ ���������

	//������ ����������
	string patternToUse;			// ������ ����������(��� ������ � ����)
	Pattern calibrationPattern;		// ������ ����������
	Size patternSize;				// ���������� �����/�������
	Size imageSize;					// ������ �����

	Mat cameraMatrix;			// ������� ���������� ���������� ������
	Mat distCoeff;				// ������ ���������

	vector<Mat> rvecs;			// ������� ��������(����� ������������� � �������) 
	vector<Mat> tvecs;			// ������� ��������
	Mat map[2];					// ����� ��������� �� undistortRecifyMap()

	void createKnownPosition(vector<Point3f>& corners);
	bool getCorners(vector<Mat>& imageArray, vector<vector<Point2f>>& allFoundCorners);	
	bool singleCalibrate(vector<Mat>& calibrationImages);

public:
	SingleCalibration(void);
	~SingleCalibration();

	bool startVideoSingleCalibration(const string& _filename, const Size& _patternSize,
		const Pattern& _patternType, const float& _squareEdgeLength);
	bool startOnlineSingleCalibration(const Size& _patternSize, const Pattern& _patternType, const float& _squareEdgeLength);

	bool singleRectify(Mat _R = Mat(), Mat _P = Mat());
	bool singleRemap(const Mat& src, Mat& dst);

	bool read(string filename);
	bool write(string filename);
};