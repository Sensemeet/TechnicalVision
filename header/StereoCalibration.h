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
	float squareSize;				//размер стороны квадрата/диаметр круга?
	bool useFisheye;				//использование камеры с эффектом "рыбий глаз"
	bool loadOptions;				//true, если параметры загружены

	vector<vector<Point3f>> objectPoints;
	vector<vector<Point2f>> imagePoints1, imagePoints2;

	//способ калибровки
	string patternToUse;			// Шаблон калибровки(для записи в файл)
	Pattern calibrationPattern;		// Шаблон калибровки
	Size patternSize;				// Количество углов/центров
	

	Mat cameraMatrix1, cameraMatrix2;
	Mat distCoeff1, distCoeff2;
	Mat map1[2], map2[2];

	Mat Rotate;			//матрица вращения между системами координат первой и второй камер 3х3
	Mat E;				//существенная матрица. Pаботает с материальными координатами
	Mat F;				//Фундаментальная матрица содержит семь параметров: два для каждой эпиполярной точки и три для гомографии, которая связывает две плоскости изображения
	Vec3d T;			//вектор перемещения между двумя камерами
	Mat R[2];			//это 3x3 построчно выравненные исправленные матрицы вращения для левой и правой плоскостей изображения
	Mat P[2];			//матрица проекции 3x4 в новых (выпрямленных) системах координат
	Mat Q;				//матрица перепроецирования(перевод из пикс координат в материальные)
	Rect roi1, roi2;	//область интереса после ректификации

	void createKnownPosition(vector<Point3f>& corners);
	bool getCorners(vector<Mat>& imagesCam1, vector<Mat>& imagesCam2,
		vector<vector<Point2f>>& allFoundCorners1, vector<vector<Point2f>>& allFoundCorners2);

	bool calibrate(vector<Mat>& calibrationImagesCamera1, vector<Mat>& calibrationImagesCamera2);
	void check(int nimages);

public:
	Size imageSize;					// Размер кадра

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