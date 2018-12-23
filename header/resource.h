#pragma once

#define MIN_NUMBER_OF_IMAGES 10
#define MAX_NUMBER_OF_IMAGES 20

#define KEY			cv::waitKey(10)
#define KEY_ESC		27
#define KEY_C		'c'
#define KEY_SPACE	' '

#define RES_OK						0
#define RES_FEW_IMAGES				1
#define RES_CALIBRATION_FAILED		2
#define RES_ABORTED					3
#define RES_CALIBRATION_NOT_STARTED	4

#define RESIZE_FRAME	false

// Выбор калибровочного шаблона
enum Pattern { CHESSBOARD = 0, CIRCLES_GRID = 1, ASYMMETRIC_CIRCLES_GRID = 2 };