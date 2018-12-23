#include "SingleCalibration.h"
#include "StereoCalibration.h"
#include "source.h"

#include <cmath>

#define CAM1_ID		( "169.254.1.2" )
#define CAM2_ID		( "169.254.1.70" )

#define FILENAME_CONFIGURATION1		( "./config/configuration1.yml" )
#define FILENAME_CONFIGURATION2		( "./config/configuration2.yml" )
#define CONFIG_CAM1					( "./config/cam1_config.yml" )
#define CONFIG_CAM2					( "./config/cam2_config.yml" )
#define CONFIG_STEREO				( "./config/best_stereo_config.yml" )

#define VIDEO_SINGLE_FILE1		( "./video/cam1.avi" )
#define VIDEO_SINGLE_FILE2		( "./video/cam2.avi" )
#define VIDEO_STEREO_FILE		( "./video/stereo2.avi" )

#define FILE_SINGLE_CALIB_CAM1	( "./config/cam1_chess.yml" )
#define FILE_SINGLE_CALIB_CAM2	( "./config/cam2_chess.yml" )
#define FILE_STEREO_CALIB		( "./config/stereo_chess.yml" )

#define PATTERN_SIZE			Size(9, 6)
#define PATTERN_SIZE_CIRCLES	Size(4, 11)
#define SQUARE_SIZE				float(85.0)

using namespace std;
using namespace cv;

Ptr<StereoBM> bm = StereoBM::create();

// Калибрация
bool singleCalibrationFunc(const string& videoFileName, const Size& patternSize,
	const Pattern& pattern, const float& squareSize, const string& fileSettings);
bool stereoCalibrationFunc(const string& videoFileName, const Size& patternSize, const Pattern& patternType,
	const float& squareSize, const string& fileSettings1 = "", const string& fileSettings2 = "");

// Запись видео
bool writeVideo();
// Показ ректифицированных кадров
bool showStereoRemap(string CalibFilename);
bool showStereoPairRemap(string VideoFilename, string CalibFilename);
bool depthMap(string VideoFilename, string CalibFilename);

static void CallBackFunc(int event, int x, int y, int flags, void* points)
{
	vector<Point2f> *tmp = (vector<Point2f>*)points;
	if (event == EVENT_LBUTTONDOWN)
	{
		tmp->push_back(Point2f(x, y));
		cout << "Point " << tmp->size() << ": [ " << x << ", " << y << " ]" << endl;
	}
}
Mat drawPointsFrame;
static void goodFeaturesToTrack_Demo(int maxCorners, void*)
{
	if (maxCorners == 0) { maxCorners = 1; }

	// Parameters for Shi-Tomasi algorithm
	RNG rng(12345);
	vector<Point2f> corners[2];
	double qualityLevel = 0.01;
	double minDistance = 200;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;
	int radius = 50;

	Mat gray = drawPointsFrame.clone();
	if (gray.channels() != 1)
	{
		cvtColor(drawPointsFrame, gray, CV_BGR2GRAY);
	}
	Mat grayL = Mat(gray, Rect(0, 0, gray.cols / 2, gray.rows));
	Mat grayR = Mat(gray, Rect(gray.cols / 2, 0, gray.cols / 2, gray.rows));


	// Apply corner detection
	goodFeaturesToTrack(grayL, corners[0], maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k);
	goodFeaturesToTrack(grayR, corners[1], maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k);

	// Draw corners detected
	cout << "-- > Number of corners1 detected: " << corners[0].size() << "\tCorners2: " << corners[1].size() << endl;;
	for (int i = 0; i < corners[0].size(); i++)
	{
		circle(grayL, corners[0][i], radius, Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
			rng.uniform(0, 255)), 4, 8, 0);
		string temp = "[ " + to_string(int(corners[0][i].x)) + ", " + to_string(int(corners[0][i].y)) + " ]";
		putText(grayL, temp, Point2f(corners[0][i].x + 30, corners[0][i].y + 30),1, 4, Scalar(255, 255, 255), 7);
	}
	for (int i = 0; i < corners[1].size(); i++)
	{
		circle(grayR, corners[1][i], radius, Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
			rng.uniform(0, 255)), 4, 8, 0);
		string temp = "[ " + to_string(int(corners[1][i].x)) + ", " + to_string(int(corners[1][i].y)) + " ]";
		putText(grayR, temp, Point2f(corners[1][i].x + 30, corners[1][i].y + 30), 1, 4, Scalar(255, 255, 255), 7);
	}
	imshow("Point", gray);
}

static void track_minDisparity(int minDisparity, void*)
{
	if (minDisparity!= bm->getMinDisparity())
	{
		bm->setMinDisparity(minDisparity);
		cout << "minDisparity: " << bm->getMinDisparity() << endl;
	}
}
static void track_numDisparities(int numDisparities, void*)
{
	if (numDisparities != bm->getNumDisparities())
	{
		if (numDisparities % 16 == 0)
		{
			bm->setNumDisparities(numDisparities);
			cout << "numDisparities: " << bm->getNumDisparities() << endl;
		}
	}
}
static void track_blockSize(int blockSize, void*)
{
	if (blockSize != bm->getBlockSize())
	{
		if (blockSize % 2 == 1 && blockSize > 4)
		{
			bm->setBlockSize(blockSize);
			cout << "blockSize: " << bm->getBlockSize() << endl;
		}
		else
		{
			cout << "blockSize: parameter error" << endl;
		}
	}
}
static void track_disp12MaxDiff(int disp12MaxDiff, void*)
{
	if (disp12MaxDiff != bm->getDisp12MaxDiff())
	{
		bm->setDisp12MaxDiff(disp12MaxDiff);
		cout << "disp12MaxDiff: " << bm->getDisp12MaxDiff() << endl;
	}
}
static void track_preFilterCap(int preFilterCap, void*)
{
	if (preFilterCap != bm->getPreFilterCap())
	{
		if (preFilterCap > 0 && preFilterCap < 64)
		{
			bm->setPreFilterCap(preFilterCap);
			cout << "preFilterCap: " << bm->getPreFilterCap() << endl;
		}
		else
		{
			cout << "preFilterCap: parameter error" << endl;
		}
	}
}
static void track_uniquenessRatio(int uniquenessRatio, void*)
{
	if (uniquenessRatio != bm->getUniquenessRatio())
	{
		bm->setUniquenessRatio(uniquenessRatio);
		cout << "uniquenessRatio: " << bm->getUniquenessRatio() << endl;
	}
}
static void track_speckleWindowSize(int speckleWindowSize, void*)
{
	if (speckleWindowSize != bm->getSpeckleWindowSize())
	{
		bm->setSpeckleWindowSize(speckleWindowSize);
		cout << "speckleWindowSize: " << bm->getSpeckleWindowSize() << endl;
	}
}
static void track_speckleRange(int speckleRange, void*)
{
	if (speckleRange != bm->getSpeckleRange())
	{
		bm->setSpeckleRange(speckleRange);
		cout << "speckleRange: " << bm->getSpeckleRange() << endl;
	}
}
static void track_textureThreshold(int textureThreshold, void*)
{
	if (textureThreshold != bm->getTextureThreshold())
	{
		bm->setTextureThreshold(textureThreshold);
		cout << "textureThreshold: " << bm->getTextureThreshold() << endl;
	}
}
static void track_preFilterSize(int preFilterSize, void*)
{
	if (preFilterSize != bm->getPreFilterSize())
	{
		if (preFilterSize > 5 && preFilterSize < 255)
		{
			bm->setPreFilterSize(preFilterSize);
			cout << "preFilterSize: " << bm->getPreFilterSize() << endl;
		}
		else
		{
			cout << "preFilterSize: parameter error." << endl;
		}
	}
}

int main()
{
	// Запись видео
	//if (writeVideo())								exit(0);

	//if (showStereoRemap(FILE_STEREO_CALIB))		exit(0);
	if (showStereoPairRemap("./video/stereopair.avi", FILE_STEREO_CALIB)) return 0;
	//if (depthMap("./video/stereopair.avi", FILE_STEREO_CALIB))				return 0;
	
	// Cam1
	//if (singleCalibrationFunc(VIDEO_SINGLE_FILE1, PATTERN_SIZE, CHESSBOARD, SQUARE_SIZE, FILE_SINGLE_CALIB_CAM1))								exit(0);
	// Cam2
	//if (singleCalibrationFunc(VIDEO_SINGLE_FILE2, PATTERN_SIZE, CHESSBOARD, SQUARE_SIZE, FILE_SINGLE_CALIB_CAM2))								exit(0);
	// Stereo
	//if (stereoCalibrationFunc(VIDEO_STEREO_FILE, PATTERN_SIZE, CHESSBOARD, SQUARE_SIZE, FILE_SINGLE_CALIB_CAM1, FILE_SINGLE_CALIB_CAM2))		exit(0);

	Source cam1, cam2;
	cam1.SelectDevice(CAM1_ID);
	cam2.SelectDevice(CAM2_ID);

	//
	// Вычисление размеров изображения с каждой камеры
	//
	int64_t width1, height1;
	cam1.GetFrameDimensions(width1, height1);
	int64_t width2, height2;
	cam2.GetFrameDimensions(width2, height2);

	char *data1 = new char[width1 * height1];
	char *data2 = new char[width2 * height2];

	Mat frame1 = Mat(height1, width1, CV_8UC1, data1);
	Mat frame2 = Mat(height2, width2, CV_8UC1, data2);
	
	namedWindow("stereo", WINDOW_FREERATIO);
	cv::moveWindow("stereo", 50, 50);

	vector<Point2f> points;

	while (waitKey(1) != 27)
	{
		if (!cam1.ApplicationLoop(data1))	continue;
		if (!cam2.ApplicationLoop(data2))	continue;

		try
		{
			Mat stereo;
			hconcat(frame1, frame2, stereo);
			imshow("stereo", stereo);
		}
		catch(cv::Exception)
		{
			break;
		}
		if (GetAsyncKeyState(27))
		{
			break;
		}
	}
	cv::destroyAllWindows();

	cam1.TearDown(true);
	cam2.TearDown(true);

	frame1.release();
	frame2.release();

	delete[] data1;
	delete[] data2;

	_getch();
	return 0;
}

bool writeVideo()
{
	cout << endl << endl << "/***********************VideoWrite***********************/" << endl << endl;
	Source cam1, cam2;
	cam1.SelectDevice(CAM1_ID);
	cam2.SelectDevice(CAM2_ID);

	//
	// Вычисление размеров изображения с каждой камеры
	//
	int64_t width1, height1;
	cam1.GetFrameDimensions(width1, height1);
	int64_t width2, height2;
	cam2.GetFrameDimensions(width2, height2);

	char *data1 = new char[width1 * height1];
	char *data2 = new char[width2 * height2];

	Mat frame1 = Mat(height1, width1, CV_8UC1, data1);
	Mat frame2 = Mat(height2, width2, CV_8UC1, data2);

	int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
	//запись видео
	VideoWriter writer1 = VideoWriter("./video/writer_cam1.avi", fourcc, 15, Size(width1, height1), 0);
	VideoWriter writer2 = VideoWriter("./video/writer_cam2.avi", fourcc, 15, Size(width2, height2), 0);
	VideoWriter writer3 = VideoWriter("./video/writer_stereo.avi", fourcc, 15, Size(width1 * 2, height1), 0);
	if (!writer1.isOpened()) return false;
	if (!writer2.isOpened()) return false;
	if (!writer3.isOpened()) return false;

	namedWindow("writeVideo", WINDOW_FREERATIO);
	cv::moveWindow("writeVideo", 50, 50);

	while (waitKey(1) != 27)
	{
		if (!cam1.ApplicationLoop(data1))	continue;
		if (!cam2.ApplicationLoop(data2))	continue;

		try
		{
			Mat stereo;

			hconcat(frame1, frame2, stereo);
			imshow("writeVideo", stereo);

			if (waitKey(1) == ' ') 
			{
				imwrite("./video/image.bmp", stereo); 
				cout << "--> Stereopair save" << endl;
			}

			//writer1.write(frame1);
			//writer2.write(frame2);
			//writer3.write(stereo);
		}
		catch (cv::Exception)
		{
			break;
		}
		if (GetAsyncKeyState(27)) {	break; }
	}
	writer1.release();
	writer2.release();
	writer3.release();
	cv::destroyAllWindows();

	cam1.TearDown(true);
	cam2.TearDown(true);

	frame1.release();
	frame2.release();

	delete[] data1;
	delete[] data2;

	_getch();
	return true;
}

bool showStereoRemap(string CalibFilename)
{
	cout << endl << endl << "/******************** showStereoRemap *****************/" << endl << endl;

	// 12мм 1.4"

	//
	// Baseline(расстояние между камерами)
	// d = Rh * tga * (x1 - x2) / H

	//
	// Расстояние до точки
	// Rh = (d * H) / (tga * (x1 - x2))

	// Расстояние до точки
	// Для откалиброванных камер
	// Z = f * d / (x1 - x2)

	StereoCalibration settings;
	if (!settings.read(CalibFilename))	return false;
	settings.stereoRectifyAndGetMap();

	// Baseline(m)
	const double d = settings.GetBaseline();
	// Focal length(pix)
	const double f = settings.GetFocalLength();
	// tga, a = 60* (альфа - угол обзора камеры) field of view
	const double a = 60;
	const double tga = tan(a);
	// Горизонтальное разрешение(frame1.cols + frame2.cols)
	static const int H = 2560;

	cout << "Baseline: " << d << " (m)" << "\tFocalLength: " << f << " (pix)" << endl
		<< "\ta = " << a << "grad" << "\ttang(a) = " << tga << "\tH = " << H << endl;

	Source cam1, cam2;
	cam1.SelectDevice(CAM1_ID);
	cam2.SelectDevice(CAM2_ID);

	//
	// Вычисление размеров изображения с каждой камеры
	//
	int64_t width1, height1;
	cam1.GetFrameDimensions(width1, height1);
	int64_t width2, height2;
	cam2.GetFrameDimensions(width2, height2);

	char *data1 = new char[width1 * height1];
	char *data2 = new char[width2 * height2];

	Mat frame1 = Mat(height1, width1, CV_8UC1, data1);
	Mat frame2 = Mat(height2, width2, CV_8UC1, data2);

	namedWindow("stereo", WINDOW_FREERATIO);
	cv::moveWindow("stereo", 50, 50);
	namedWindow("showStereoRemap", WINDOW_FREERATIO);
	cv::moveWindow("showStereoRemap", 50, 600);

	vector<Point2f> points;

	//set the callback function for any mouse event
	setMouseCallback("showStereoRemap", CallBackFunc, &points);
	//set the callback function for any mouse event
	setMouseCallback("stereo", CallBackFunc, &points);

	while (waitKey(1) != 27)
	{
		if (!cam1.ApplicationLoop(data1))	continue;
		if (!cam2.ApplicationLoop(data2))	continue;

		try
		{
			// Вычисление расстояния, если получены точки с каждого изображения
			// Последующая очистка вектора точек
			if (points.size() > 0 && points[0].x > settings.imageSize.width)
			{
				points.clear();
			}
			if (points.size() == 2)
			{
				cout << "Point1: [ " << points[0].x << ", " << points[0].y << " ]\t" <<
					"Point2: [ " << points[1].x << ", " << points[1].y << " ]" << endl;
				if (points[1].x > settings.imageSize.width)
				{
					points[1].x -= settings.imageSize.width;

					cout << endl << "Method 1: Z = (f * d) / (x1 - x2) = "
						<< (f * d) / (points[0].x - points[1].x) << endl << endl;

					cout << endl << "Method 2: Z = (d * H) / (tga * (x1 - x2) ) = "
						<< (d * H) / (tga * (points[0].x - points[1].x)) << endl << endl;
				}

				
				
				//if (points[1].y > points[0].y - 10 && points[1].y < points[0].y + 10)
				points.clear();
			}

			Mat left, right;
			Mat stereo;

			if (settings.imageSize != Size(width1,height1))
			{
				resize(frame1, left, settings.imageSize);
				resize(frame2, right, settings.imageSize);

				hconcat(left, right, stereo);
			}
			else
			{
				hconcat(frame1, frame2, stereo);
			}
			imshow("stereo", stereo);

			Mat remFrame;
			if (settings.stereoRemap(stereo, remFrame))
			{
				imshow("showStereoRemap", remFrame);
			}
		}
		catch (cv::Exception)
		{
			break;
		}
		if (GetAsyncKeyState(27))
		{
			break;
		}
	}
	cv::destroyAllWindows();

	cam1.TearDown(true);
	cam2.TearDown(true);

	frame1.release();
	frame2.release();

	delete[] data1;
	delete[] data2;

	_getch();
	return true;
}
bool showStereoPairRemap(string VideoFilename, string CalibFilename)
{
	cout << endl << endl << "/******************** showStereoPairRemap *****************/" << endl << endl;

	// 12мм 1.4"

	//
	// Baseline(расстояние между камерами)
	// d = Rh * tga * (x1 - x2) / H

	//
	// Расстояние до точки
	// Rh = (d * H) / (tga * (x1 - x2))

	// Расстояние до точки
	// Для откалиброванных камер
	// Z = f * d / (x1 - x2)

	StereoCalibration settings;
	if (!settings.read(CalibFilename))	return false;
	settings.stereoRectifyAndGetMap();

	// Baseline(m)
	const double d = settings.GetBaseline();
	// Focal length(pix)
	const double f = settings.GetFocalLength();
	// tga, a = 60* (альфа - угол обзора камеры) field of view
	const double a = 60;
	const double tga = tan(a);
	// Горизонтальное разрешение(frame1.cols + frame2.cols)
	static const int H = 2560;
	// Fundamental matrix
	Mat F = settings.GetFundamental();

	cout << endl << ">> Baseline: " << d << " (m)" << endl
		<< ">> FocalLength: " << f << " (pix)" << endl
		<< ">> a = " << a << " grad" << ", tang(a) = " << tga << endl 
		<< ">> H = " << H << endl << endl;

	Mat videoFrame, remFrame;
	videoFrame = imread("./video/image.bmp", -1);

	//namedWindow("stereopair", WINDOW_FREERATIO);
	//namedWindow("remap", WINDOW_FREERATIO);

	//vector<Point2f> points;
	//set the callback function for any mouse event
	//setMouseCallback("remap", CallBackFunc, &points);
	// Create Trackbar to set the number of corners

	int sl_maxCorners = 1;
	namedWindow("Point", WINDOW_FREERATIO);
	createTrackbar("Max corners:", "Point", &sl_maxCorners, 50, goodFeaturesToTrack_Demo);

	while (true)
	{
		if (!settings.stereoRemap(videoFrame, remFrame))
		{
			break;
		}

		drawPointsFrame = remFrame.clone();
		imshow("Point", drawPointsFrame);

		// Вычисление расстояния, если получены точки с каждого изображения
		// Последующая очистка вектора точек
		//if (points.size() > 0 && points[0].x > settings.imageSize.width)
		//{
		//	points.clear();
		//}
		//if (points.size() == 2)
		//{
		//	cout << "Point1: [ " << points[0].x << ", " << points[0].y << " ]\t" <<
		//		"Point2: [ " << points[1].x << ", " << points[1].y << " ]" << endl;
		//	if (points[1].x > settings.imageSize.width)
		//	{
		//		points[1].x -= settings.imageSize.width;
		//		cout << endl << "Method 1: Z = (f * d) / (x1 - x2) = "
		//			<< (f * d) / (points[0].x - points[1].x) << endl << endl;
		//		cout << endl << "Method 2: Z = (d * H) / (tga * (x1 - x2) ) = "
		//			<< (d * H) / (tga * (points[0].x - points[1].x)) << endl << endl;
		//	}
		//	//if (points[1].y > points[0].y - 10 && points[1].y < points[0].y + 10)
		//	points.clear();
		//}

		//imshow("stereopair", videoFrame);
		//imshow("remap", remFrame);
		if (waitKey() == 'f')
		{
			int x1, x2, y1, y2;
			cout << "Point1: x1 = ";
			cin >> x1;
			cout << "\ty1 = ";
			cin >> y1;
			cout << endl;

			cout << "Point2: x2 = ";
			cin >> x2;
			cout << "\ty2 = ";
			cin >> y2;
			cout << endl;

			cout << endl << "Method 1: Z = (f * d) / (x1 - x2) = "
				<< (f * d) / (x1 - x2) << endl << endl;
			cout << endl << "Method 2: Z = (d * H) / (tga * (x1 - x2) ) = "
				<< (d * H) / (tga * (x1 - x2)) << endl << endl;
		}
		if (waitKey() == ' ') { continue; }
		if (waitKey(1) == 27) { break; }
	}
	//video.release();
	videoFrame.release();
	remFrame.release();

	//points.clear();

	destroyAllWindows();

	return true;
}
bool depthMap(string VideoFilename, string CalibFilename)
{
	cout << endl << endl << "/******************** depthMap *****************/" << endl << endl;

	//slider on trackbar
	int sl_minDisparity = 100;
	int sl_numDisparities =   4;
	int sl_blockSize = 21;
	int sl_disp12MaxDiff = 1;
	int sl_preFilterCap = 8;
	int sl_uniquenessRatio = 0;
	int sl_speckleWindowSize = 0;
	int sl_speckleRange = 0;
	//BM
	int sl_textureThreshold = 10;
	int sl_preFilterSize = 9;

	StereoCalibration settings;
	if (!settings.read(CalibFilename))	return false;
	settings.stereoRectifyAndGetMap();

	Mat videoFrame, remFrame;
	VideoCapture video(VideoFilename);

	namedWindow("depthMap",		WINDOW_FREERATIO);
	namedWindow("remap",		WINDOW_FREERATIO);
	namedWindow("TrackbarWin",	WINDOW_FREERATIO);

	//трекбары для настройки параметров глубины
	createTrackbar("minDisparity", "TrackbarWin", &sl_minDisparity, 200, track_minDisparity);
	createTrackbar("numDisparities", "TrackbarWin", &sl_numDisparities, 16, track_numDisparities);
	createTrackbar("blockSize", "TrackbarWin", &sl_blockSize, 21, track_blockSize);
	createTrackbar("disp12MaxDiff", "TrackbarWin", &sl_disp12MaxDiff, 2, track_disp12MaxDiff);
	//track on preFilterCap
	createTrackbar("preFilterCap", "TrackbarWin", &sl_preFilterCap, 63, track_preFilterCap);
	createTrackbar("uniquenessRatio", "TrackbarWin", &sl_uniquenessRatio, 30, track_uniquenessRatio);
	createTrackbar("speckleWindowSize", "TrackbarWin", &sl_speckleWindowSize, 30, track_speckleWindowSize);
	createTrackbar("speckleRange", "TrackbarWin", &sl_speckleRange, 150, track_speckleRange);
	//BM
	createTrackbar("textureThreshold", "TrackbarWin", &sl_textureThreshold, 1000, track_textureThreshold);
	createTrackbar("preFilterSize", "TrackbarWin", &sl_preFilterSize, 200, track_preFilterSize);

	while (video.read(videoFrame))
	{
		if (!settings.stereoRemap(videoFrame, remFrame))
		{
			break;
		}

		Mat disparity;
		Mat left = Mat(remFrame, Rect(0, 0, remFrame.cols / 2, remFrame.rows));
		Mat right = Mat(remFrame, Rect(remFrame.cols / 2, 0, remFrame.cols / 2, remFrame.rows));

		// Get disparity

		if (left.size() != right.size())
		{
			cout << "Size mismatch" << endl;
			break;
		}
		Size imagesize = left.size();

		Mat gray1, gray2;

		if (remFrame.channels() != 1)
		{
			cvtColor(left, gray1, COLOR_BGR2GRAY);
			cvtColor(right, gray2, COLOR_BGR2GRAY);
		}
		disparity = Mat(imagesize.height, imagesize.width, CV_16S);

		try
		{
			bm->compute(gray1, gray2, disparity);
		}
		catch (cv::Exception)
		{
			cout << "Err" << endl;
		}
		// Get depthMap

		Mat disp8;
		disparity.convertTo(disp8, CV_8U);

		normalize(disparity, disp8, 0, 255, CV_MINMAX, CV_8U);

		// Get falseColorsMap

		double min, max;

		minMaxIdx(disp8, &min, &max);

		Mat adjMap;
		disp8.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

		Mat falseColorsMap;
		applyColorMap(adjMap, falseColorsMap, COLORMAP_JET);

		while (true)
		{		
			imshow("remap", remFrame);
			imshow("depthMap", falseColorsMap);


			if (waitKey(1) == ' ')
			{
				break;
			}
		}
		if (waitKey(1) == 27)
			break;
	}

	video.release();
	videoFrame.release();
	remFrame.release();

	destroyAllWindows();

	return true;
}

bool singleCalibrationFunc(const string& videoFileName, const Size& patternSize,
	const Pattern& pattern, const float& squareSize, const string& fileSettings)
{
	SingleCalibration settings;

	settings.read(fileSettings);

	// Онлайн
	//if (!settings.startOnlineSingleCalibration(patternSize, pattern, squareSize))					return false;
	
	// Офлайн
	if (!settings.startVideoSingleCalibration(videoFileName, patternSize, pattern, squareSize))		return false;
	
	settings.singleRectify();
	settings.write(fileSettings);

	Mat videoFrame, remFrame;
	VideoCapture video(VIDEO_SINGLE_FILE2);

	//settings.read(FILE_SINGLE_CALIB_CAM1);
	namedWindow("video", WINDOW_FREERATIO);
	namedWindow("remap", WINDOW_FREERATIO);

	while (video.read(videoFrame))
	{
		if (RESIZE_FRAME)
		{
			resize(videoFrame, videoFrame, Size(640, 480));
		}

		imshow("video", videoFrame);
		
		if (settings.singleRemap(videoFrame, remFrame))
		{
			imshow("remap", remFrame);
		}

		if (waitKey(1) == 27)
			break;
	}

	video.release();
	videoFrame.release();
	remFrame.release();

	destroyAllWindows();

	return true;
}

bool stereoCalibrationFunc(const string& videoFileName, const Size& patternSize, const Pattern& patternType,
	const float& squareSize, const string& fileSettings1, const string& fileSettings2)
{
	StereoCalibration settings;

	//settings.readSettingsSingleCalibration(1, fileSettings1);
	//settings.readSettingsSingleCalibration(2, fileSettings2);
	//settings.read(FILE_STEREO_CALIB);

	if (!settings.startVideoStereoCalibrate(videoFileName, patternSize, patternType, squareSize))	return false;
	settings.stereoRectifyAndGetMap();
	settings.write(FILE_STEREO_CALIB);

	Mat videoFrame, remFrame;
	VideoCapture video(VIDEO_STEREO_FILE);

	namedWindow("video", WINDOW_FREERATIO);
	namedWindow("remap", WINDOW_FREERATIO);

	while (video.read(videoFrame))
	{
		if (RESIZE_FRAME)
		{
			resize(videoFrame, videoFrame, Size(1280, 480));
		}

		imshow("video", videoFrame);
		
		Mat remleft, remright;
		Mat left = Mat(videoFrame, Rect(0, 0, videoFrame.cols / 2, videoFrame.rows));
		Mat right = Mat(videoFrame, Rect(videoFrame.cols / 2, 0, videoFrame.cols / 2, videoFrame.rows));

		if (settings.stereoRemap(videoFrame, remFrame))
		{
			imshow("remap", remFrame);
		}

		if (waitKey(1) == 27)
			break;
	}

	video.release();
	videoFrame.release();
	remFrame.release();

	destroyAllWindows();

	return true;
}