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
	float squareSize;			// Размер стороны квадрата/диаметр круга?
	bool useFisheye;			// Использование фишай - камеры
	bool loadOptions;			// True, если внутренние параметры и к-ты искажений получены

	vector<vector<Point3f>> objectPoints;		// Создание шаблона координат точек на всех изображениях в реальном мире
	vector<vector<Point2f>> imagePoints;		// Координаты точек на всех изображенях на проекционной плоскости

	//способ калибровки
	string patternToUse;			// Шаблон калибровки(для записи в файл)
	Pattern calibrationPattern;		// Шаблон калибровки
	Size patternSize;				// Количество углов/центров
	Size imageSize;					// Размер кадра

	Mat cameraMatrix;			// Матрица внутренних параметров камеры
	Mat distCoeff;				// Вектор искажений

	vector<Mat> rvecs;			// Векторы поворота(можно преобразовать в матрицу) 
	vector<Mat> tvecs;			// Векторы смещения
	Mat map[2];					// Карты искажений из undistortRecifyMap()

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