#pragma once

#include <windows.h>
#include <conio.h>
#include <iostream>

#include "pvsampleutils.h"
#include "PvInterface.h"
#include "PvDevice.h"
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include "PvBuffer.h"
#include "PvStream.h"
#include "PvStreamGEV.h"
#include "PvStreamU3V.h"
#include "PvPixelType.h"
#include "ImagingBuffer.h"
#include "ImagingContrastFilter.h"
#include "PvPipeline.h"
#include "PvConfigurationWriter.h"
#include "PvConfigurationReader.h"

#include "features2d.hpp"
#include "core.hpp"
#include "calib3d\calib3d.hpp"
#include "highgui.hpp"
#include "imgcodecs.hpp"
#include "imgproc.hpp"
#include "videoio.hpp"

#include "PvBufferConverter.h"

#define DEVICE_CONFIGURATION_TAG	( "DeviceConfiguration" )
#define STREAM_CONFIGURAITON_TAG	( "StreamConfiguration" )
#define STRING_INFORMATION_TAG		( "StringInformation" )

//#define TEST_STRING ( "This is a test string" )


class Source : protected PvDeviceEventSink
{
public:

	Source();
	~Source();

	bool SelectDevice(const PvString& aConnectionID = "");
	bool ApplicationLoop(void* _data);
	void TearDown(bool aStopAcquisition);

	bool GetFrameDimensions(int64_t& width, int64_t &height);
	bool StoreConfiguration(const PvString& filename);
	bool RestoreConfiguration(const PvString& filename);
	bool getAllSettings();

protected:
	bool ConnectDevice();
	void DisconnectDevice();
	bool OpenStream();
	void CloseStream();
	bool StartAcquisition();
	bool StopAcquisition();

	// Inherited from PvDeviceEventSink.
	void OnLinkDisconnected(PvDevice* aDevice);


	bool DumpGenParameterArray(PvGenParameterArray *aArray);
	bool GetHostCommunicationRelatedSettings();
	bool GetDeviceSettings();
	bool GetImageStreamControllerSettings();


	bool RestoreDevice(const PvString& filename);
	bool RestoreStream(const PvString& filename);

private:
	PvString mConnectionID;

	PvDevice* mDevice;
	PvStream* mStream;
	PvPipeline* mPipeline;

	bool mConnectionLost;
};
