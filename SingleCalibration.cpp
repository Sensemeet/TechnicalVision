#include "SingleCalibration.h"


SingleCalibration::SingleCalibration(void)
{
	loadOptions = false;
	squareSize = 1.0;
	useFisheye = false;	
}

SingleCalibration::~SingleCalibration()
{
	cameraMatrix.release();
	distCoeff.release();

	rvecs.clear();
	tvecs.clear();

	map[0].release();
	map[1].release();

	objectPoints.clear();
	imagePoints.clear();
}


//
// ������ ����� ������� ����� � ����������� ����������
//
bool SingleCalibration::startOnlineSingleCalibration(const Size& _patternSize, const Pattern& _patternType, const float& _squareEdgeLength)
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

	int count = 0;								// ������� ����������� �����������
	int result = RES_CALIBRATION_NOT_STARTED;	// ��������� ����������

	vector<Mat> savedImages;

	Source cam;
	cam.SelectDevice();

	int64_t width, height;
	cam.GetFrameDimensions(width, height);

	char *data = new char[width * height];

	Mat frame = Mat(height, width, CV_8UC1, data);
	Mat tmpResize;

	namedWindow("Single Calibration", WINDOW_FREERATIO);
	namedWindow("drawtest", WINDOW_FREERATIO);

	while (true)
	{
		// ���������� ����� �� ���������
		vector<Vec2f> foundPoints;

		if (!cam.ApplicationLoop(data))
		{
			continue;
		}

		// �������� ���������� �� ������� �������
		if (KEY == KEY_C)
		{
			if (savedImages.size() >= MIN_NUMBER_OF_IMAGES)
			{
				cout << "Number of images: " << savedImages.size() << endl;
				if (singleCalibrate(savedImages))
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
					<< "Number of images: " << savedImages.size() << endl;

				result = RES_FEW_IMAGES;
			}
		}

		if (RESIZE_FRAME)
		{
			resize(frame, tmpResize, Size(640, 480));
		}
		else
		{
			frame.copyTo(tmpResize);
		}

		// ����� ������� ����� �� ������� �������
		if (KEY == KEY_SPACE)
		{
			//����� ������� �����
			bool found = false;

			switch (calibrationPattern)
			{
			case CHESSBOARD:
				//��� ����������� � ��������� ������
				found = findChessboardCorners(tmpResize, patternSize, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
				break;
			case CIRCLES_GRID:
				//��� ������������ ����������� � �������
				found = findCirclesGrid(tmpResize, patternSize, foundPoints);
				break;
			case ASYMMETRIC_CIRCLES_GRID:
				//��� ������������� ����������� � �������
				found = findCirclesGrid(tmpResize, patternSize, foundPoints, CALIB_CB_ASYMMETRIC_GRID);
				break;
			}

			//��������� ����, ���� �� �em ���� ������� ������� �����
			if (found)
			{
				Mat tmp;
				tmpResize.copyTo(tmp);
				savedImages.push_back(tmp);

				count++;
				cout << "--> Image Save: " << count << endl;

				imshow("drawtest", tmpResize);
				tmp.release();
			}
			else
			{
				cout << "Corners not found" << endl;
			}
		}

		// ���������� ��� ���������� ������������� ���������� ����������� �����������
		if (savedImages.size() >= MAX_NUMBER_OF_IMAGES)
		{
			cout << "Number of images: " << savedImages.size() << endl;
			if (singleCalibrate(savedImages))
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

		// ����� �� ���������� �� ������� ������� ESC
		if (KEY == KEY_ESC || GetAsyncKeyState(27))
		{
			cout << "Calibration aborted" << endl;

			result = RES_ABORTED;
			break;
		}

		imshow("Single Calibration", frame);
	}

	//delete[] data;
	frame.release();
	delete[] data;

	savedImages.clear();

	cv::destroyWindow("Single Calibration");
	cv::destroyWindow("drawtest");

	if (result != RES_OK)
	{
		return false;
	}

	return true;
}
//
// ��������� ���������� �����, �� ������� ���� ����� ����� � ������
//
bool SingleCalibration::startVideoSingleCalibration(const string& filename, 
	const Size& _patternSize, const Pattern& patternType, const float& squareEdgeLength)
{
	calibrationPattern = patternType;
	squareSize = squareEdgeLength;
	patternSize = _patternSize;

	if (calibrationPattern == CHESSBOARD) patternToUse = "CHESSBOARD";
	else if (calibrationPattern == CIRCLES_GRID) patternToUse = "CIRCLES_GRID";
	else if (calibrationPattern == ASYMMETRIC_CIRCLES_GRID) patternToUse = "ASYMMETRIC_CIRCLES_GRID";
	else
	{
		cout << "Calibration error. Invalid template." << endl;
		return false;
	}

	int count = 0;								// ������� ����������� �����������
	int result = RES_CALIBRATION_NOT_STARTED;	// ��������� ����������

	Mat frame;
	vector<Mat> savedImages;
	VideoCapture capture(filename);

	if (!capture.isOpened())
	{
		cout << "Capture is not open" << endl;
		return false;
	}

	namedWindow("Single Calibration", WINDOW_FREERATIO);
	namedWindow("drawtest", WINDOW_FREERATIO);

	while(true)
	{
		//���������� ����� �� ���������
		vector<Vec2f> foundPoints;

		if (!capture.read(frame))
		{
			if (savedImages.size() >= MIN_NUMBER_OF_IMAGES)
			{
				cout << "Number of images: " << savedImages.size() << endl;
				if (singleCalibrate(savedImages))
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
					<< "Number of images: " << savedImages.size() << endl;

				result = RES_FEW_IMAGES;
				break;
			}
		}

		if (RESIZE_FRAME)
		{
			resize(frame, frame, Size(640, 480));
		}

		if (KEY == KEY_SPACE)
		{
			//����� ������� �����
			bool found = false;

			switch (calibrationPattern)
			{
			case CHESSBOARD:
				//��� ����������� � ��������� ������
				found = findChessboardCorners(frame, patternSize, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
				break;
			case CIRCLES_GRID:
				//��� ������������ ����������� � �������
				found = findCirclesGrid(frame, patternSize, foundPoints);
				break;
			case ASYMMETRIC_CIRCLES_GRID:
				//��� ������������� ����������� � �������
				found = findCirclesGrid(frame, patternSize, foundPoints, CALIB_CB_ASYMMETRIC_GRID);
				break;
			}

			//��������� ����, ���� �� �em ���� ������� ������� �����
			if (found)
			{
				Mat tmp;
				frame.copyTo(tmp);
				savedImages.push_back(tmp);

				count++;
				cout << "--> Image Save: " << count << endl;

				imshow("drawtest", frame);
				tmp.release();
			}
			else
			{
				cout << "Corners not found" << endl;
			}
		}

		// ���������� ��� ���������� ������������� ���������� ����������� �����������
		if (savedImages.size() >= MAX_NUMBER_OF_IMAGES)
		{
			cout << "Number of images: " << savedImages.size() << endl;
			if (singleCalibrate(savedImages))
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

		// ����� �� ���������� �� ������� ������� ESC
		if (KEY == KEY_ESC || GetAsyncKeyState(27))
		{
			cout << "Calibration aborted" << endl;

			result = RES_ABORTED;
			break;
		}

		imshow("Single Calibration", frame);
	}

	//delete[] data;
	frame.release();
	capture.release();
	savedImages.clear();

	cv::destroyWindow("Single Calibration");
	cv::destroyWindow("drawtest");
	
	if (result != RES_OK)
	{
		return false;
	}

	return true;
}

//
// ����������
//
bool SingleCalibration::singleCalibrate(vector<Mat>& calibrationImages)
{
	// ������ �����������
	imageSize = Size(calibrationImages[0].size());

	// ����� ��������� ����� �� ����������� ������������
	if (!getCorners(calibrationImages, imagePoints))
	{
		cout << "Corners not found" << endl;
		return false;
	}

	// �������� �������, ����������� ������������ 
	// ���������� ����� �� ���� �����������
	objectPoints = vector<vector<Point3f>>(1);
	createKnownPosition(objectPoints[0]);
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//
	// ��������� ����������� ������
	//

	int flags = 0;

	// ����, ���������� �� ��������� ����������� ����� ����������� ����������
	// ���� ������� ���������� ��� ���� ��������.
	if (loadOptions && cameraMatrix.dims != 0)
	{
		cout << "Data was previously downloaded. Matrix will be updated" << endl;
		flags += CALIB_USE_INTRINSIC_GUESS
			+ CALIB_FIX_PRINCIPAL_POINT;
	}

	// ����������� �3(�����������, ���� �� ������������ �����-������)
	if (!useFisheye)
	{
		flags += CALIB_FIX_K3;
	}

	// ���� ���� ����� ����� � ������ ���������� ������������������ �����, ������� � ���������� ���������� �������� ��� ������������ ����� ����� �����
	// �������������� ���������.������� ������� ����������, ������� � 0, ����� �������� � ���������� ������ ��������� � ��������� �������������� ������������.
	// ��������� ������� ����� �������� � ����, ��� ��������� ��������������� ��������� � ��������������� � 0.
	flags += CALIB_ZERO_TANGENT_DIST;

	// �������� ��������� ������� ���������� ����������,
	// ���� ��� �� ���� �������� �����
	if (cameraMatrix.dims == 0)
	{
		cameraMatrix = initCameraMatrix2D(objectPoints, imagePoints, imageSize, 0);
	}

	cout << endl << "Running calibration... ";

	// ������� ���������� ������� ������ ���������� �������������.
	// ��� ����� ���� ������� ������ �������� ��������� ����������. 
	// ��� ������ ���� ��� ����� ����� � ����
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeff, rvecs, tvecs, flags);

	cout << "Complete" << endl;
	cout << "Done with RMS error=" << rms << endl;

	loadOptions = true;

	return true;
}

//
// ��������� ��������� ����� � ������� ������� ���������
//
void SingleCalibration::createKnownPosition(vector<Point3f>& corners)
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
// ��������� ��������� ����� �� ������������ ���������
//
bool SingleCalibration::getCorners(vector<Mat>& imageArray, vector<vector<Point2f>>& allFoundCorners)
{
	allFoundCorners.clear();

	//destroyAllWindows();
	//namedWindow("frame", WINDOW_FREERATIO);

	for (vector<Mat>::iterator iter = imageArray.begin(); iter != imageArray.end(); iter++)
	{
		vector<Point2f> pointBuf;
		bool found = false;

		Mat gray;

		//imshow("frame", *iter);
		//waitKey();

		//��������� ����������� � ������� ������(���������� ��� �������������� ���������)
		if (iter->channels() == 3)
		{
			cvtColor(*iter, gray, CV_BGR2GRAY);
		}
		else if (iter->channels() == 1)
		{
			iter->copyTo(gray);
		}
		else
		{
			cout << "ERROR calibrate" << endl;
			return false;
		}

		switch (calibrationPattern)
		{
		case CHESSBOARD:
			//��� ����������� � ��������� ������
			found = findChessboardCorners(gray, patternSize, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

			if (found)
			{
				//������������� ��������� �����
				cornerSubPix(gray, pointBuf, patternSize, Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
					30,		// max number of iterations 
					0.1));     // min accuracy);
			}

			break;
		case CIRCLES_GRID:
			//��� ������������ ����������� � �������
			found = findCirclesGrid(*iter, patternSize, pointBuf);
			break;
		case ASYMMETRIC_CIRCLES_GRID:
			//��� ������������� ����������� � �������
			found = findCirclesGrid(*iter, patternSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
			break;
		}

		if (found)
		{
			allFoundCorners.push_back(pointBuf);
		}
	}
	return true;
}

//
// ��������� ���� ���������
//
bool SingleCalibration::singleRectify(Mat _R, Mat _P)
{
	cout << "--> Initialization undistorted rectify map... " ;

	if (loadOptions)
	{
		initUndistortRectifyMap(cameraMatrix, distCoeff, _R, _P, imageSize, CV_32FC1, map[0], map[1]);

		cout << "Complete" << endl;
		return true;
	}
	else
	{
		cout << "Failed";
		cout << std::endl << "Calibration has not been performed. Distortion map not received" << endl;
		return false;
	}
}

//
// ����������� ����������� � ������� ���������� ���� ���������
//
bool SingleCalibration::singleRemap(const Mat& src, Mat& dst)
{
	if (loadOptions && (map[0].dims != 0 && map[1].dims != 0))
	{
		// ��������� ������� �����������
		remap(src, dst, map[0], map[1],	INTER_LINEAR);

		return true;
	}
	else
	{
		cout << "Rectification has not been performed. Original image returned" << endl;
		src.copyTo(dst);

		return false;
	}
}

//
// ������ ����������� ���������� � ����
//
bool SingleCalibration::write(string filename)
{
	cout << "--> Saving calibrate parameters... ";
	FileStorage fs(filename, FileStorage::WRITE);

	if (!fs.isOpened())
	{
		cout << "Saving failed" << endl;
		return false;
	}

	fs << "Calibrate_Pattern" << patternToUse
		<< "patternSize" << patternSize
		<< "squareSize" << squareSize;

	fs << "imageSize" << imageSize;

	fs << "M" << cameraMatrix
		<< "D" << distCoeff;

	//fs << "rvecs" << rvecs;
	//fs << "tvecs" << tvecs;

	//fs << "map1" << map[0]
	//	<< "map2" << map[1];

	fs.release();

	cout << "Complete" << endl;

	return true;
}
//
// ������ ����������� ���������� �� �����
//
bool SingleCalibration::read(string filename)
{
	cout << "--> Download calibrate parameters... ";

	FileStorage node(filename, cv::FileStorage::READ);
	if (!node.isOpened())
	{
		cout << "Download failed" << endl;
		return false;
	}

	node["Calibrate_Pattern"] >> patternToUse;
	node["patternSize"] >> patternSize;
	node["squareSize"] >> squareSize;

	node["imageSize"] >> imageSize;

	node["M"] >> cameraMatrix;
	node["D"] >> distCoeff;

	//node["rvecs"] >> rvecs;
	//node["tvecs"] >> tvecs;

	//node["map1"] >> map[0];
	//node["map2"] >> map[1];

	node.release();

	loadOptions = true;

	cout << "Complete" << endl;

	return true;
}