#include "StereoCalibration.h"



StereoCalibration::StereoCalibration()
{
	squareSize = 1.0;
	useFisheye = false;
	loadOptions = false;
	imageSize = Size(0, 0);
}

StereoCalibration::~StereoCalibration()
{
	Rotate.release();
	P[0].release();
	P[1].release();
	R[0].release();
	R[1].release();
	Q.release();
	F.release();
	E.release();

	map1[0].release();
	map1[1].release();
	map2[0].release();
	map2[1].release();

	cameraMatrix1.release();
	cameraMatrix2.release();
	distCoeff1.release();
	distCoeff2.release();

	objectPoints.clear();
	imagePoints1.clear();
	imagePoints2.clear();
}


//
// Онлайн поиск угловых точек и последующая калибровка
//
bool StereoCalibration::startOnlineStereoCalibrate(const Size& _patternSize, const Pattern& _patternType, const float& _squareEdgeLength)
{
	calibrationPattern = _patternType;
	squareSize = _squareEdgeLength;
	patternSize = _patternSize;

	if (calibrationPattern == CHESSBOARD) patternToUse = "CHESSBOARD";
	else if (calibrationPattern == CIRCLES_GRID) patternToUse = "CIRCLES_GRID";
	else if (calibrationPattern == ASYMMETRIC_CIRCLES_GRID) patternToUse = "ASYMMETRIC_CIRCLES_GRID";
	else
	{
		cout << "Calibration error. Invalid template." << endl;
		return false;
	}

	int count = 0;								// Счетчик сохраненных изображений
	int result = RES_CALIBRATION_NOT_STARTED;	// Результат калибровки

	// Вектора сохраненных изображений с левой и правой части кадра
	vector<Mat> savedImages1;
	vector<Mat> savedImages2;

	Mat originalFrame;					// Оригинальный большой кадр

	Source cam1, cam2;
	cam1.SelectDevice();
	cam2.SelectDevice();

	int64_t width1, height1;
	cam1.GetFrameDimensions(width1, height1);
	int64_t width2, height2;
	cam2.GetFrameDimensions(width2, height2);

	char *data1 = new char[width1 * height1];
	char *data2 = new char[width2 * height2];

	Mat frame1 = Mat(height1, width1, CV_8UC1, data1);
	Mat frame2 = Mat(height2, width2, CV_8UC1, data2);

	namedWindow("Stereo calibrate", WINDOW_FREERATIO);
	namedWindow("drawTest", WINDOW_FREERATIO);

	while (true)
	{
		if (!cam1.ApplicationLoop(data1))	continue;
		if (!cam2.ApplicationLoop(data2))	continue;

		// Начинает калибровку по нажатию клавиши
		if (KEY == KEY_C)
		{
			if (savedImages1.size() == savedImages2.size() && savedImages1.size() >= MIN_NUMBER_OF_IMAGES)
			{
				cout << "Number of images in the first matrix vector: " << savedImages1.size() << endl
					<< "Number of images in the second matrix vector: " << savedImages2.size() << endl;
				if (calibrate(savedImages1, savedImages2))
				{
					result = RES_OK;
					break;
				}
				else
				{
					result = RES_CALIBRATION_FAILED;
					break;
				}
			}
			else
			{
				cout << "More images needed for calibration" << endl
					<< "Number of images: " << savedImages1.size() << endl;

				result = RES_FEW_IMAGES;
			}
		}

		// Поиск угловых точек по нажатию клавиши
		if (KEY == KEY_SPACE)
		{			
			//координаты точек на плоскости
			vector<Vec2f> foundPoints1;
			vector<Vec2f> foundPoints2;

			bool found1 = false;
			bool found2 = false;

			//поиск угловых точек на кадре
			switch (calibrationPattern)
			{
			case CHESSBOARD:
				//для изображений с шахматной доской
				found1 = findChessboardCorners(frame1, patternSize, foundPoints1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
				found2 = findChessboardCorners(frame2, patternSize, foundPoints2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
				break;
			case CIRCLES_GRID:
				//для симметричных изображений с кругами
				found1 = findCirclesGrid(frame1, patternSize, foundPoints1);
				found2 = findCirclesGrid(frame2, patternSize, foundPoints2);
				break;
			case ASYMMETRIC_CIRCLES_GRID:
				//для асимметричных изображений с кругами
				found1 = findCirclesGrid(frame1, patternSize, foundPoints1, CALIB_CB_ASYMMETRIC_GRID);
				found2 = findCirclesGrid(frame2, patternSize, foundPoints2, CALIB_CB_ASYMMETRIC_GRID);
				break;
			}

			//сохраняет оба кадра, если на них были найдены угловые точки
			if (found1 && found2)
			{
				Mat tmp1, tmp2;
				frame1.copyTo(tmp1);
				frame2.copyTo(tmp2);

				savedImages1.push_back(tmp1);
				savedImages2.push_back(tmp2);

				count++;

				cout << "--> StereoImage Save " << count << endl;

				imshow("drawTest", originalFrame);

				tmp1.release();
				tmp2.release();
			}
			else
			{
				cout << "Corners not found" << endl;
			}
		}

		// Калибрация при достижении максимального количества сохраненных изображений
		if (savedImages1.size() == savedImages2.size() && savedImages1.size() >= MAX_NUMBER_OF_IMAGES)
		{
			cout << "Number of images in the first matrix vector: " << savedImages1.size() << endl
				<< "Number of images in the second matrix vector: " << savedImages2.size() << endl;
			if (calibrate(savedImages1, savedImages2))
			{
				result = RES_OK;
				break;
			}
			else
			{
				result = RES_CALIBRATION_FAILED;
				break;
			}
		}

		// Выход из калибровки по нажатию клавиши ESC
		if (KEY == KEY_ESC)
		{
			cout << "Calibration aborted" << endl;

			result = RES_ABORTED;
			break;
		}

		hconcat(frame1, frame2, originalFrame);
		imshow("Stereo calibrate", originalFrame);
	}

	originalFrame.release();

	frame1.release();
	frame2.release();

	delete[] data1;
	delete[] data2;

	savedImages1.clear();
	savedImages2.clear();

	cv::destroyWindow("Stereo calibrate");
	cv::destroyWindow("drawTest");

	if (result != RES_OK)
	{
		return false;
	}

	return true;
}
//
// Запускает переданное видео, на котором идет поиск точек с кадров
//
bool StereoCalibration::startVideoStereoCalibrate(const string& _filename, const Size& _patternSize, const Pattern& _patternType, const float& _squareEdgeLength)
{
	calibrationPattern = _patternType;
	squareSize = _squareEdgeLength;
	patternSize = _patternSize;

	if (calibrationPattern == CHESSBOARD) patternToUse = "CHESSBOARD";
	else if (calibrationPattern == CIRCLES_GRID) patternToUse = "CIRCLES_GRID";
	else if (calibrationPattern == ASYMMETRIC_CIRCLES_GRID) patternToUse = "ASYMMETRIC_CIRCLES_GRID";
	else
	{
		cout << "Calibration error. Invalid template." << endl;
		return false;
	}

	int count = 0;								// Счетчик сохраненных изображений
	int result = RES_CALIBRATION_NOT_STARTED;	// Результат калибровки

	// Вектора сохраненных изображений с левой и правой части кадра
	vector<Mat> savedImages1;
	vector<Mat> savedImages2;

	Mat originalFrame;					// Оригинальный большой кадр
	VideoCapture capture(_filename);

	if (!capture.isOpened())
	{
		cout << "Capture is not open" << endl;
		return false;
	}

	namedWindow("Stereo calibrate", WINDOW_FREERATIO);
	namedWindow("drawTest", WINDOW_FREERATIO);

	while (true)
	{
		if (!capture.read(originalFrame))
		{
			if (savedImages1.size() == savedImages2.size() && savedImages1.size() >= MIN_NUMBER_OF_IMAGES)
			{
				cout << "Number of images in the first matrix vector: " << savedImages1.size() << endl
					<< "Number of images in the second matrix vector: " << savedImages2.size() << endl;
				if (calibrate(savedImages1, savedImages2))
				{
					result = RES_OK;
					break;
				}
				else
				{
					result = RES_CALIBRATION_FAILED;
					break;
				}
			}
			else
			{
				cout << "More images needed for calibration" << endl
					<< "Number of images: " << savedImages1.size() << endl;

				result = RES_FEW_IMAGES;
				break;
			}
		}

		if (RESIZE_FRAME)
		{
			resize(originalFrame, originalFrame, Size(1280, 480));
		}

		// Поиск угловых точек по нажатию клавиши
		if (KEY == KEY_SPACE)
		{
			// Матрицы, в которых будут храниться части оригинального кадра
			Mat frame1 = Mat(originalFrame, Rect(0, 0, originalFrame.cols / 2, originalFrame.rows));
			Mat frame2 = Mat(originalFrame, Rect(originalFrame.cols / 2, 0, originalFrame.cols / 2, originalFrame.rows));

			//координаты точек на плоскости
			vector<Vec2f> foundPoints1;
			vector<Vec2f> foundPoints2;

			bool found1 = false;
			bool found2 = false;

			//поиск угловых точек на кадре
			switch (calibrationPattern)
			{
			case CHESSBOARD:
				//для изображений с шахматной доской
				found1 = findChessboardCorners(frame1, patternSize, foundPoints1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
				found2 = findChessboardCorners(frame2, patternSize, foundPoints2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
				break;
			case CIRCLES_GRID:
				//для симметричных изображений с кругами
				found1 = findCirclesGrid(frame1, patternSize, foundPoints1);
				found2 = findCirclesGrid(frame2, patternSize, foundPoints2);
				break;
			case ASYMMETRIC_CIRCLES_GRID:
				//для асимметричных изображений с кругами
				found1 = findCirclesGrid(frame1, patternSize, foundPoints1, CALIB_CB_ASYMMETRIC_GRID);
				found2 = findCirclesGrid(frame2, patternSize, foundPoints2, CALIB_CB_ASYMMETRIC_GRID);
				break;
			}

			//сохраняет оба кадра, если на них были найдены угловые точки
			if (found1 && found2)
			{
				Mat tmp1, tmp2;
				frame1.copyTo(tmp1);
				frame2.copyTo(tmp2);
				
				savedImages1.push_back(tmp1);
				savedImages2.push_back(tmp2);

				count++;

				cout << "--> StereoImage Save " << count << endl;

				imshow("drawTest", originalFrame);

				tmp1.release();
				tmp2.release();
			}
			else
			{
				cout << "Corners not found" << endl;
			}

			frame1.release();
			frame2.release();
		}

		// Калибрация при достижении максимального количества сохраненных изображений
		if (savedImages1.size() == savedImages2.size() && savedImages1.size() >= MAX_NUMBER_OF_IMAGES)
		{
			cout << "Number of images in the first matrix vector: " << savedImages1.size() << endl
				<< "Number of images in the second matrix vector: " << savedImages2.size() << endl;
			if (calibrate(savedImages1, savedImages2))
			{
				result = RES_OK;
				break;
			}
			else
			{
				result = RES_CALIBRATION_FAILED;
				break;
			}
		}

		// Выход из калибровки по нажатию клавиши ESC
		if (KEY == KEY_ESC)
		{
			cout << "Calibration aborted" << endl;

			result = RES_ABORTED;
			break;
		}

		imshow("Stereo calibrate", originalFrame);
	}

	originalFrame.release();

	capture.release();
	savedImages1.clear();
	savedImages2.clear();

	cv::destroyWindow("Stereo calibrate");
	cv::destroyWindow("drawTest");

	if (result != RES_OK)
	{
		return false;
	}

	return true;
}

//
// Калибровка
//
bool StereoCalibration::calibrate(vector<Mat>& calibrationImagesCamera1, vector<Mat>& calibrationImagesCamera2)
{
	imageSize = Size(calibrationImagesCamera1[0].size());		// Размер изображения
	int flags = 0;

	// Проверка наличия и соответсвия размеров изображений в полученных векторах
	if (calibrationImagesCamera1.size() == 0 || calibrationImagesCamera2.size() == 0 || 
		calibrationImagesCamera1.size() != calibrationImagesCamera2.size())
	{
		cout << "Error: images were not uploaded. Return." << endl;
		return false;
	}

	// Поиск координат углов на сохраненных изображениях
	if (!getCorners(calibrationImagesCamera1, calibrationImagesCamera2, imagePoints1, imagePoints2))
	{
		cout << "Corners not found" << endl;
		return false;
	}

	// Создание вектора, содержащего материальные 
	// координаты точек со всех изображений
	objectPoints = vector<vector<Point3f>>(1);
	createKnownPosition(objectPoints[0]);
	objectPoints.resize(imagePoints1.size(), objectPoints[0]);

	// Флаг, отвечающий за уточнение результатов ранее выполненной калибровки.
	// Только если калибровочные матрицы уже были получены.
	if (loadOptions || (cameraMatrix1.dims != 0 && cameraMatrix2.dims != 0))
	{
		cout << "Data was previously downloaded. Matrix will be updated" << endl;

		flags += CALIB_USE_INTRINSIC_GUESS		// Уточнение матриц, и центральная точка извлекается из загруженных матриц
			+ CALIB_FIX_PRINCIPAL_POINT;		// Этот флаг может быть использован с и без CV_CALIB_USE_INTRINSIC_GUESS.Если без, то точка устанавливается, как правило, 
												// в центре изображения; если с, то точка устанавливается, как правило, на предполагаемое начальное значение intrinsic_matrix.
	}

	// Коэффициент К3(фиксируется, если не используется фишай - камера)
	if (!useFisheye)
	{
		flags += CALIB_FIX_K3;
	}

	//Этот флаг очень важен в случае калибровки высококачественных камер, которые в результате соблюдения точности при изготовлении имеют очень малые
	//тангенциальные искажения.Попытки подгона параметров, близких к 0, могут привести к зашумлению ложным значениям и проблемам вычислительной устойчивости.
	//Установка данного флага приводит к тому, что параметры тангенциального искажения и устанавливаются в 0.
	flags += CALIB_ZERO_TANGENT_DIST;

	// Этот флаг задействует функцию оптимизации 
	// за счет использования fx и fy, переданных в intrinsic_matrix.
	//flags += CALIB_FIX_FOCAL_LENGTH

	// Инициализация значений матриц, если они не были загружены
	if (cameraMatrix1.dims == 0 || cameraMatrix2.dims == 0)
	{
		cameraMatrix1 = initCameraMatrix2D(objectPoints, imagePoints1, imageSize, 0);
		cameraMatrix2 = initCameraMatrix2D(objectPoints, imagePoints2, imageSize, 0);
	}

	cout << endl << "Running stereo calibration... ";

	// Функция возвращает среднюю ошибку повторного проецирования.
	// Это число дает хорошую оценку точности найденных параметров. 
	// Оно должно быть как можно ближе к нулю
	double rms = stereoCalibrate(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeff1, cameraMatrix2, distCoeff2, imageSize, Rotate, T, E, F, flags);

	cout << "Complete" << endl;
	cout << "Done with RMS error=" << rms << endl;

	loadOptions = true;

	return true;
}
//
// Проверка стерео калибровки по эпиполярному ограничению m2^t * F * m1 = 0
//
void StereoCalibration::check(int nimages)
{
	cout << "Check stereocalibration" << endl;

	if (!loadOptions)
	{
		cout << "Stereo calibration has not been performed" << endl;
		return;
	}

	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];

	vector<vector<Point2f>> imagePoints[2];
	imagePoints[0] = vector<vector<Point2f>>(imagePoints1);
	imagePoints[1] = vector<vector<Point2f>>(imagePoints2);

	vector<Mat> cameraMatrix[2];
	cameraMatrix[0] = vector<Mat>(cameraMatrix1);
	cameraMatrix[1] = vector<Mat>(cameraMatrix2);

	vector<Mat> distCoeffs[2];
	distCoeffs[0] = vector<Mat>(distCoeff1);
	distCoeffs[1] = vector<Mat>(distCoeff2);

	for (int i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (int k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (int j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] +
				imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x * lines[0][j][0] +
					imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average epipolar err = " << err / npoints << endl;
}

//
// Получение координат точек в мировой системе координат
//
void StereoCalibration::createKnownPosition(vector<Point3f>& corners)
{
	corners.clear();

	switch (calibrationPattern)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for (int i = 0; i < patternSize.height; ++i)
			for (int j = 0; j < patternSize.width; ++j)
				corners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
		break;

	case ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < patternSize.height; i++)
			for (int j = 0; j < patternSize.width; j++)
				corners.push_back(Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
		break;
	default:
		cout << "Error pattern" << endl;
	}
}
//
// Получение координат углов на проекционной плоскости
//
bool StereoCalibration::getCorners(vector<Mat>& imagesCam1, vector<Mat>& imagesCam2,
	vector<vector<Point2f>>& allFoundCorners1, vector<vector<Point2f>>& allFoundCorners2)
{
	allFoundCorners1.clear();
	allFoundCorners2.clear();

	for (vector<Mat>::iterator iter1 = imagesCam1.begin(), iter2 = imagesCam2.begin(); iter1 != imagesCam1.end() && iter2 != imagesCam2.end(); iter1++, iter2++)
	{
		if (iter1->size != iter2->size)
		{
			std::cout << "ERROR: image1.size != image2.size" << std::endl;
			
			return false;
		}

		vector<Point2f> pointBuf1;
		vector<Point2f> pointBuf2;

		Mat gray1, gray2;

		//переводим изображение в оттенки серого(необходимо для субпиксельного уточнения)
		if (iter1->channels() == 3 && iter2->channels() == 3)
		{
			cvtColor(*iter1, gray1, CV_BGR2GRAY);
			cvtColor(*iter2, gray2, CV_BGR2GRAY);
		}
		else if (iter1->channels() == 1 && iter2->channels() == 1)
		{
			iter1->copyTo(gray1);
			iter2->copyTo(gray2);
		}
		else
		{
			cout << "ERROR calibrate" << endl;
			return false;
		}
		

		bool found1 = false;
		bool found2 = false;

		switch (calibrationPattern)
		{
		case CHESSBOARD:
			//для изображений с шахматной доской
			found1 = findChessboardCorners(gray1, patternSize, pointBuf1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
			found2 = findChessboardCorners(gray2, patternSize, pointBuf2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

			if (found1 && found2)
			{
				//субпиксельное уточнение углов
				cornerSubPix(gray1, pointBuf1, patternSize, Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
					30,		// max number of iterations 
					0.1));     // min accuracy);

				cornerSubPix(gray2, pointBuf1, patternSize, Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
					30,		// max number of iterations 
					0.1));     // min accuracy);
			}
			break;
		case CIRCLES_GRID:
			//для симметричных изображений с кругами
			found1 = findCirclesGrid(*iter1, patternSize, pointBuf1);
			found2 = findCirclesGrid(*iter2, patternSize, pointBuf2);
			break;
		case ASYMMETRIC_CIRCLES_GRID:
			//для асимметричных изображений с кругами
			found1 = findCirclesGrid(*iter1, patternSize, pointBuf1, CALIB_CB_ASYMMETRIC_GRID);
			found2 = findCirclesGrid(*iter2, patternSize, pointBuf2, CALIB_CB_ASYMMETRIC_GRID);
			break;
		}

		//добавление точек с каждого изображения в общий массив
		if (found1 && found2)
		{
			allFoundCorners1.push_back(pointBuf1);
			allFoundCorners2.push_back(pointBuf2);
		}
	}

	return true;
}

//
// Computes rectification transforms for each head of a calibrated stereo camera
//
bool StereoCalibration::stereoRectifyAndGetMap()
{
	cout << "--> Initialization stereo rectify and undistorted rectify map... ";
	if (loadOptions)
	{
		stereoRectify(cameraMatrix1, distCoeff1, cameraMatrix2, distCoeff2, imageSize, Rotate, T, R[0], R[1], P[0], P[1], Q, CALIB_ZERO_DISPARITY, -1, imageSize, &roi1, &roi2);

		initUndistortRectifyMap(cameraMatrix1, distCoeff1, R[0], P[0], imageSize, CV_32FC1, map1[0], map1[1]);
		initUndistortRectifyMap(cameraMatrix2, distCoeff2, R[1], P[1], imageSize, CV_32FC1, map2[0], map2[1]);

		cout << "Complete" << endl;
		return true;
	}
	else
	{
		cout << "Failed";
		cout << endl << "Stereo calibration has not been performed. Distortion map not received" << endl;
		return false;
	}
}
//
// Исправление изображений с помощью полученных карт искажений
//
bool StereoCalibration::stereoRemap(const Mat& left, const Mat& right, Mat& leftRem, Mat& rightRem)
{
	if (loadOptions && (map1[0].dims != 0 && map1[1].dims != 0) && (map2[0].dims != 0 && map2[1].dims != 0))
	{
		remap(left, leftRem, map1[0], map1[1], INTER_LINEAR);
		remap(right, rightRem, map2[0], map2[1], INTER_LINEAR);

		return true;
	}
	else
	{
		cout << "Rectification has not been performed. Original image returned" << endl;

		left.copyTo(leftRem);
		right.copyTo(rightRem);

		return false;
	}
}
bool StereoCalibration::stereoRemap(const Mat& originalFrame, Mat& remFrame)
{
	if (loadOptions && (map1[0].dims != 0 && map1[1].dims != 0) && (map2[0].dims != 0 && map2[1].dims != 0))
	{
		Mat left = Mat(originalFrame, Rect(0, 0, originalFrame.cols / 2, originalFrame.rows));
		Mat right = Mat(originalFrame, Rect(originalFrame.cols / 2, 0, originalFrame.cols / 2, originalFrame.rows));		

		Mat remL, remR;
		remap(left, remL, map1[0], map1[1], INTER_LINEAR);
		remap(right, remR, map2[0], map2[1], INTER_LINEAR);


		hconcat(remL, remR, remFrame);

		return true;
	}
	else
	{
		cout << "Rectification has not been performed. Original image returned" << endl;

		originalFrame.copyTo(remFrame);

		return false;
	}
}


//
// Запись результатов стерео калибровки в файл
//
bool StereoCalibration::write(string filename)
{
	cout << "--> Saving stereo calibrate parameters to " << filename << "... ";

	FileStorage fs(filename, FileStorage::WRITE);

	if (!fs.isOpened())
	{
		cout << "Saving failed" << endl;
		return false;
	}

	fs << "patternToUse" << patternToUse
		<< "patternSize" << patternSize;

	fs << "Square_Size" << squareSize;
	fs << "imageSize" << imageSize;

	fs << "M1" << cameraMatrix1
		<< "D1" << distCoeff1;

	fs << "M2" << cameraMatrix2
		<< "D2" << distCoeff2;

	fs << "Rotate" << Rotate
		<< "T" << T
		<< "E" << E
		<< "F" << F;

	fs << "R1" << R[0]
		<< "R2" << R[1];
	fs << "P1" << P[0]
		<< "P2" << P[1];
	fs << "Q" << Q;

	fs << "roi1" << roi1
		<< "roi2" << roi2;

	/*fs << "map1X" << map1[0]
		<< "map1Y" << map1[1];
	fs << "map2X" << map2[0]
		<< "map2Y" << map2[1];*/

	cout << "Complete" << endl;

	fs.release();

	return true;
}
//
// Чтение результатов стерео калибровки из файла
//
bool StereoCalibration::read(string filename)
{
	cout << "--> Download stereo calibrate parameters from " << filename << "... ";

	FileStorage node(filename, FileStorage::READ);

	if (!node.isOpened())
	{
		cout << "Download failed" << endl;
		return false;
	}

	node["patternToUse"] >> patternToUse;
	node["patternSize"] >> patternSize;

	node["Square_Size"] >> squareSize;
	node["imageSize"] >> imageSize;

	node["M1"] >> cameraMatrix1;
	node["D1"] >> distCoeff1;

	node["M2"] >> cameraMatrix2;
	node["D2"] >> distCoeff2;

	node["Rotate"] >> Rotate;
	node["T"] >> T;
	node["E"] >> E;
	node["F"] >> F;

	node["R1"] >> R[0];
	node["R2"] >> R[1];
	node["P1"] >> P[0];
	node["P2"] >> P[1];
	node["Q"] >> Q;

	node["roi1"] >> roi1;
	node["roi2"] >> roi2;

	/*node["map1X"] >> map1[0];
	node["map1Y"] >> map1[1];
	node["map2X"] >> map2[0];
	node["map2Y"] >> map2[1];*/

	cout << "Complete" << endl;
	
	loadOptions = true;

	node.release();

	return true;
}
//
// Чтение результатов одиночной калибровки из файла
//
bool StereoCalibration::readSettingsSingleCalibration(const int& numberOfCamera, const string& filename)
{
	cout << "--> Download Camera" << numberOfCamera << " calibrate parameters... ";
	FileStorage node(filename, FileStorage::READ);

	if (!node.isOpened() || numberOfCamera > 2 || numberOfCamera <= 0)
	{
		cout << "Download failed" << endl;
		return false;
	}

	node["imageSize"] >> imageSize;

	if (numberOfCamera == 1)
	{
		node["M"] >> cameraMatrix1;
		node["D"] >> distCoeff1;

		//node["map1"] >> map1[0];
		//node["map2"] >> map1[1];
	}
	if (numberOfCamera == 2)
	{
		node["M"] >> cameraMatrix2;
		node["D"] >> distCoeff2;
	}
	
	cout << "Complete" << endl;

	node.release();

	return true;
}