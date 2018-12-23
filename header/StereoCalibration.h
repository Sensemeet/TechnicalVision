#pragma once

#include <iostream>
#include <vector>
#include <string>

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


class StereoCalibration
{
private:
	float squareSize;				//������ ������� ��������/������� �����?
	bool useFisheye;				//������������� ������ � �������� "����� ����"
	bool loadOptions;				//true, ���� ��������� ���������

	vector<vector<Point3f>> objectPoints;
	vector<vector<Point2f>> imagePoints1, imagePoints2;

	//������ ����������
	string patternToUse;			// ������ ����������(��� ������ � ����)
	Pattern calibrationPattern;		// ������ ����������
	Size patternSize;				// ���������� �����/�������
	

	Mat cameraMatrix1, cameraMatrix2;
	Mat distCoeff1, distCoeff2;
	Mat map1[2], map2[2];

	Mat Rotate;			//������� �������� ����� ��������� ��������� ������ � ������ ����� 3�3
	Mat E;				//������������ �������. P������� � ������������� ������������
	Mat F;				//��������������� ������� �������� ���� ����������: ��� ��� ������ ����������� ����� � ��� ��� ����������, ������� ��������� ��� ��������� �����������
	Vec3d T;			//������ ����������� ����� ����� ��������
	Mat R[2];			//��� 3x3 ��������� ����������� ������������ ������� �������� ��� ����� � ������ ���������� �����������
	Mat P[2];			//������� �������� 3x4 � ����� (������������) �������� ���������
	Mat Q;				//������� �����������������(������� �� ���� ��������� � ������������)
	Rect roi1, roi2;	//������� �������� ����� ������������

	void createKnownPosition(vector<Point3f>& corners);
	bool getCorners(vector<Mat>& imagesCam1, vector<Mat>& imagesCam2,
		vector<vector<Point2f>>& allFoundCorners1, vector<vector<Point2f>>& allFoundCorners2);

	bool calibrate(vector<Mat>& calibrationImagesCamera1, vector<Mat>& calibrationImagesCamera2);
	void check(int nimages);

public:
	Size imageSize;					// ������ �����

	StereoCalibration();
	~StereoCalibration();

	bool startVideoStereoCalibrate(const string& filename, const Size& _patternSize, const Pattern& _patternType, const float& _squareEdgeLength);
	bool startOnlineStereoCalibrate(const Size& _patternSize, const Pattern& _patternType, const float& _squareEdgeLength);

	bool stereoRectifyAndGetMap();
	bool stereoRemap(const Mat& left, const Mat& right, Mat& leftRem, Mat& rightRem);
	bool stereoRemap(const Mat& originalFrame, Mat& remFrame);

	bool write(string filename = "StereoCalibration.yml");
	bool read(string filename = "StereoCalibration.yml");
	bool readSettingsSingleCalibration(const int& numberOfCamera, const string& filename);

	double	GetBaseline()			{ return (-1) * T(0) / 1000; }
	double	GetFocalLength()		{ return ((cameraMatrix1.at<double>(0, 0) + cameraMatrix2.at<double>(0, 0)) / 2); }
	Mat		GetFundamental()		{ return F; }
	Vec3d	GetT()					{ return T; }
};