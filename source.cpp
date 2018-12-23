#include "source.h"

Source::Source()
{
	mDevice = NULL;
	mStream = NULL;
	mPipeline = NULL;
	mConnectionLost = false;
}

Source::~Source()
{
}


// Selects a device to work with
bool Source::SelectDevice(const PvString& aConnectionID)
{
	std::cout << "--> SelectDevice... ";
	
	// Select the device
	if (aConnectionID != static_cast<PvString>(""))
	{
		mConnectionID = aConnectionID;
	}
	else if (!PvSelectDevice(&mConnectionID))
	{
		std::cout << "No device selected." << std::endl;
		return false;
	}

	std::cout << "Device: " << aConnectionID.GetAscii() << std::endl;
	//cout << mConnectionID.GetAscii() << endl;

	// IMPORTANT: 
	//
	// Here we assume that the device will come back with the same IP 
	// address either through DHCP, static IP configuration or simple network
	// unplug/replug LLA. If the device can come back with a different IP address
	// the MAC address of the device should be used as connection ID. This will
	// not be as efficient but will allow reconnecting the device even if it comes
	// back with a different IP address. 
	//
	// This does not apply to USB3 Vision devices who should always stick to the same device GUID.
	//

	return true;
}

// Selects, connects a device.
bool Source::ConnectDevice()
{
	std::cout << "--> ConnectDevice " << mConnectionID.GetAscii() << std::endl;

	// Connect to the selected Device
	PvResult lResult = PvResult::Code::INVALID_PARAMETER;
	mDevice = PvDevice::CreateAndConnect(mConnectionID, &lResult);
	if (!lResult.IsOK())
	{
		return false;
	}

	// Register this class as an event sink for PvDevice call-backs
	mDevice->RegisterEventSink(this);

	// Clear connection lost flag as we are now connected to the device
	mConnectionLost = false;

	return true;
}

// Opens stream, pipeline, allocates buffers
bool Source::OpenStream()
{
	std::cout << "--> OpenStream" << std::endl;

	// Creates and open the stream object based on the selected device.
	PvResult lResult = PvResult::Code::INVALID_PARAMETER;
	mStream = PvStream::CreateAndOpen(mConnectionID, &lResult);
	if (!lResult.IsOK())
	{
		std::cout << "Unable to open the stream" << std::endl;
		return false;
	}

	mPipeline = new PvPipeline(mStream);

	// Reading payload size from device
	int64_t lSize = mDevice->GetPayloadSize();

	// Create, init the PvPipeline object
	mPipeline->SetBufferSize(static_cast<uint32_t>(lSize));
	mPipeline->SetBufferCount(16);

	// The pipeline needs to be "armed", or started before  we instruct the device to send us images
	lResult = mPipeline->Start();
	if (!lResult.IsOK())
	{
		std::cout << "Unable to start pipeline" << std::endl;
		return false;
	}

	// Only for GigE Vision, if supported
	PvGenBoolean *lRequestMissingPackets = dynamic_cast<PvGenBoolean *>(mStream->GetParameters()->GetBoolean("RequestMissingPackets"));
	if ((lRequestMissingPackets != NULL) && lRequestMissingPackets->IsAvailable())
	{
		// Disabling request missing packets.
		lRequestMissingPackets->SetValue(false);
	}

	return true;
}

// Closes the stream, pipeline
void Source::CloseStream()
{
	std::cout << "--> CloseStream" << std::endl;

	if (mPipeline != NULL)
	{
		if (mPipeline->IsStarted())
		{
			if (!mPipeline->Stop().IsOK())
			{
				std::cout << "Unable to stop the pipeline." << std::endl;
			}
		}

		//delete mPipeline;
		mPipeline = NULL;
	}

	if (mStream != NULL)
	{
		if (mStream->IsOpen())
		{
			if (!mStream->Close().IsOK())
			{
				std::cout << "Unable to stop the stream." << std::endl;
			}
		}

		PvStream::Free(mStream);
		mStream = NULL;
	}
}

// Starts image acquisition
bool Source::StartAcquisition()
{
	std::cout << "--> StartAcquisition" << std::endl;

	// Flush packet queue to make sure there is no left over from previous disconnect event
	PvStreamGEV* lStreamGEV = dynamic_cast<PvStreamGEV*>(mStream);
	if (lStreamGEV != NULL)
	{
		lStreamGEV->FlushPacketQueue();
	}

	// Set streaming destination (only GigE Vision devces)
	PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV*>(mDevice);
	if (lDeviceGEV != NULL)
	{
		// If using a GigE Vision, it is same to assume the stream object is GigE Vision as well
		PvStreamGEV* lStreamGEV = static_cast<PvStreamGEV*>(mStream);

		// Have to set the Device IP destination to the Stream
		PvResult lResult = lDeviceGEV->SetStreamDestination(lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort());
		if (!lResult.IsOK())
		{
			std::cout << "Setting stream destination failed" << std::endl;
			return false;
		}
	}

	// Enables stream before sending the AcquisitionStart command.
	mDevice->StreamEnable();

	// The pipeline is already "armed", we just have to tell the device to start sending us images
	PvResult lResult = mDevice->GetParameters()->ExecuteCommand("AcquisitionStart");
	if (!lResult.IsOK())
	{
		std::cout << "Unable to start acquisition" << std::endl;
		return false;
	}

	return true;
}

bool Source::StopAcquisition()
{
	std::cout << "--> StopAcquisition" << std::endl;

	// Tell the device to stop sending images.
	if(mDevice == NULL)
	{
		return false;
	}

	mDevice->GetParameters()->ExecuteCommand("AcquisitionStop");
	
	// Disable stream after sending the AcquisitionStop command.
	mDevice->StreamDisable();

	PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV*>(mDevice);
	if (lDeviceGEV != NULL)
	{
		// Reset streaming destination (optional...)
		lDeviceGEV->ResetStreamDestination();
	}

	return true;
}

// Acquisition loop
bool Source::ApplicationLoop(void* _data)
{
	int timeoutCount = 0;			// Счетчик для таймаута
	bool lFirstTimeout = true;

	while (true)
	{
		if (timeoutCount > 4)
		{
			TearDown(true);

			timeoutCount = 0;
			
			return false;
		}

		// If connection flag is up, teardown device/stream
		if (mConnectionLost && (mDevice != NULL))
		{
			// Device lost: no need to stop acquisition
			TearDown(false);
		}

		// If the device is not connected, attempt reconnection
		if (mDevice == NULL)
		{
			std::cout << std::endl;
			if (ConnectDevice())
			{
				// Device is connected, open the stream
				if (OpenStream())
				{
					// Device is connected, stream is opened: start acquisition
					if (!StartAcquisition())
					{
						TearDown(false);
					}
				}
				else
				{
					TearDown(false);
				}
			}
		}

		// If still no device, no need to continue the loop
		if (mDevice == NULL)
		{
			continue;
		}

		if ((mStream != NULL) && mStream->IsOpen() && (mPipeline != NULL) && mPipeline->IsStarted())
		{
			// Retrieve next buffer     
			PvBuffer *lBuffer = NULL;
			PvResult  lOperationResult;
			PvResult lResult = mPipeline->RetrieveNextBuffer(&lBuffer, 1000, &lOperationResult);
			
			if (lResult.IsOK())
			{
				if (lOperationResult.IsOK())
				{
					// If the buffer contains an image
					if (lBuffer->GetPayloadType() == PvPayloadTypeImage)
					{
						// Get image specific buffer interface.
						PvImage *lImage = lBuffer->GetImage();

						uint32_t lWidth = 0, lHeight = 0;
						lWidth = lImage->GetWidth();
						lHeight = lImage->GetHeight();
						if (lWidth == 0 || lHeight == 0)
						{
							mPipeline->ReleaseBuffer(lBuffer);
							return false;
						}

						memcpy(_data, lImage->GetDataPointer(), lWidth * lHeight * sizeof(char));

						mPipeline->ReleaseBuffer(lBuffer);

						return true;
					}

					lFirstTimeout = true;
				}
				else
				{

					//cout << lOperationResult.GetCodeString() << endl;
					//mPipeline->ReleaseBuffer(lBuffer);

					//return false;
				}
				// We have an image - do some processing (...) and VERY IMPORTANT,
				// release the buffer back to the pipeline.
				mPipeline->ReleaseBuffer(lBuffer);

				//return false;
			}
			else
			{
				// Timeout

				if (lFirstTimeout)
				{
					std::cout << "" << std::endl;
					lFirstTimeout = false;
				}

				std::cout << "Image timeout " << std::endl;
				timeoutCount++;
			}
		}

		if (cv::waitKey(1) == 27 || GetAsyncKeyState(27))
			break;
	}

	return false;
}

// \brief Disconnects the device
void Source::DisconnectDevice()
{
	std::cout << "--> DisconnectDevice" << std::endl;

	if (mDevice != NULL)
	{
		if (mDevice->IsConnected())
		{
			// Unregister event sink (call-backs).
			mDevice->UnregisterEventSink(this);
		}

		PvDevice::Free(mDevice);
		mDevice = NULL;
	}
}

// Tear down: closes, disconnects, etc.
void Source::TearDown(bool aStopAcquisition)
{
	std::cout << std::endl << "--> TearDown" << std::endl;
	if (mDevice == NULL)
	{
		cout << "--> Close device" << endl;
		return;
	}

	if (aStopAcquisition)
	{
		StopAcquisition();
	}

	CloseStream();
	DisconnectDevice();
}

// PvDeviceEventSink callback
// Notification that the device just got disconnected.
void Source::OnLinkDisconnected(PvDevice *aDevice)
{
	std::cout << "=====> PvDeviceEventSink::OnLinkDisconnected callback" << std::endl;

	// Just set flag indicating we lost the device. The main loop will tear down the 
	// device/stream and attempt reconnection. 
	mConnectionLost = true;

	// IMPORTANT:
	// The PvDevice MUST NOT be explicitly disconnected from this callback. 
	// Here we just raise a flag that we lost the device and let the main loop
	// of the application (from the main application thread) perform the
	// disconnect.
	//
}


// Function returning the image size a device is configured to stream
bool Source::GetFrameDimensions(int64_t& width, int64_t& height)
{
	while (true)
	{
		if (ConnectDevice())
			break;
	}
	PvGenParameterArray *lDeviceParams = mDevice->GetParameters();

	PvResult lResult = lDeviceParams->GetIntegerValue("Width", width);
	if (!lResult.IsOK())
	{
		cerr << "Could not get width" << endl;
		PvDevice::Free(mDevice);
		return false;
	}

	lResult = lDeviceParams->GetIntegerValue("Height", height);
	if (!lResult.IsOK())
	{
		cerr << "Could not get height" << endl;
		PvDevice::Free(mDevice);
		return false;
	}

	cout << "Frame dimensions are " << width << "x" << height << endl;

	DisconnectDevice();

	return true;
}


// Dumps the full content of a PvGenParameterArray.
bool Source::DumpGenParameterArray(PvGenParameterArray *aArray)
{
	// Getting array size
	uint32_t lParameterArrayCount = aArray->GetCount();
	cout << endl;
	cout << "Array has " << lParameterArrayCount << " parameters" << endl;

	// Traverse through Array and print out parameters available.
	for (uint32_t x = 0; x < lParameterArrayCount; x++)
	{
		// Get a parameter
		PvGenParameter *lGenParameter = aArray->Get(x);

		// Don't show invisible parameters - display everything up to Guru.
		if (!lGenParameter->IsVisible(PvGenVisibilityGuru))
		{
			continue;
		}

		// Get and print parameter's name.
		PvString lGenParameterName, lCategory;
		lGenParameter->GetCategory(lCategory);
		lGenParameter->GetName(lGenParameterName);
		cout << lCategory.GetAscii() << ":" << lGenParameterName.GetAscii() << ", ";

		// Parameter available?
		if (!lGenParameter->IsAvailable())
		{
			cout << "{Not Available}" << endl;
			continue;
		}

		// Parameter readable?
		if (!lGenParameter->IsReadable())
		{
			cout << "{Not readable}" << endl;
			continue;
		}

		// Get the parameter type
		PvGenType lType;
		lGenParameter->GetType(lType);
		switch (lType)
		{
		case PvGenTypeInteger:
		{
			int64_t lValue;
			static_cast<PvGenInteger *>(lGenParameter)->GetValue(lValue);
			cout << "Integer: " << lValue;
		}
		break;

		case PvGenTypeEnum:
		{
			PvString lValue;
			static_cast<PvGenEnum *>(lGenParameter)->GetValue(lValue);
			cout << "Enum: " << lValue.GetAscii();
		}
		break;

		case PvGenTypeBoolean:
		{
			bool lValue;
			static_cast<PvGenBoolean *>(lGenParameter)->GetValue(lValue);
			if (lValue)
			{
				cout << "Boolean: TRUE";
			}
			else
			{
				cout << "Boolean: FALSE";
			}
		}
		break;

		case PvGenTypeString:
		{
			PvString lValue;
			static_cast<PvGenString *>(lGenParameter)->GetValue(lValue);
			cout << "String: " << lValue.GetAscii();
		}
		break;

		case PvGenTypeCommand:
			cout << "Command";
			break;

		case PvGenTypeFloat:
		{
			double lValue;
			static_cast<PvGenFloat *>(lGenParameter)->GetValue(lValue);
			cout << "Float: " << lValue;
		}
		break;

		default:
			// Unexpected
			break;
		}
		cout << endl;
	}

	return true;
}

// Get Host's communication-related settings.
bool Source::GetHostCommunicationRelatedSettings()
{
	// Communication link can be configured before we connect to the device.
	// No need to connect to the device.
	cout << "Using non-connected PvDevice" << endl;
	PvDeviceGEV lDeviceGEV;

	// Get the communication link parameters array
	cout << "Retrieving communication link parameters array" << endl;
	PvGenParameterArray* lComLink = lDeviceGEV.GetCommunicationParameters();

	// Dumping communication link parameters array content
	cout << "Dumping communication link parameters array content" << endl;
	DumpGenParameterArray(lComLink);

	lDeviceGEV.Disconnect();

	return true;
}

// Get the Device's settings
bool Source::GetDeviceSettings()
{
	// Connect to the GigE Vision or USB3 Vision device
	PvResult lResult;
	PvDevice *lDevice = PvDevice::CreateAndConnect(mConnectionID, &lResult);
	if (!lResult.IsOK())
	{
		cout << "Unable to connect to device" << endl;
		PvDevice::Free(lDevice);
		return NULL;
	}

	// Get the device's parameters array. It is built from the 
	// GenICam XML file provided by the device itself.
	cout << "Retrieving device's parameters array" << endl;
	PvGenParameterArray* lParameters = lDevice->GetParameters();

	// Dumping device's parameters array content.
	cout << "Dumping device's parameters array content" << endl;
	DumpGenParameterArray(lParameters);

	// Get width parameter - mandatory GigE Vision parameter, it should be there.
	PvGenParameter *lParameter = lParameters->Get("Width");

	// Converter generic parameter to width using dynamic cast. If the
	// type is right, the conversion will work otherwise lWidth will be NULL.
	PvGenInteger *lWidthParameter = dynamic_cast<PvGenInteger *>(lParameter);

	if (lWidthParameter == NULL)
	{
		cout << "Unable to get the width parameter." << endl;
	}

	// Read current width value.
	int64_t lOriginalWidth = 0;
	if (!(lWidthParameter->GetValue(lOriginalWidth).IsOK()))
	{
		cout << "Error retrieving width from device" << endl;
	}

	// Read max.
	int64_t lMaxWidth = 0;
	if (!(lWidthParameter->GetMax(lMaxWidth).IsOK()))
	{
		cout << "Error retrieving width max from device" << endl;
	}

	// Change width value.
	if (!lWidthParameter->SetValue(lMaxWidth).IsOK())
	{
		cout << "Error changing width on device - the device is on Read Only Mode, please change to Exclusive to change value" << endl;
	}

	// Reset width to original value.
	if (!lWidthParameter->SetValue(lOriginalWidth).IsOK())
	{
		cout << "Error changing width on device" << endl;
	}

	return true;
}

// Get Image stream controller settings.
bool Source::GetImageStreamControllerSettings()
{
	PvString lDeviceID;
	PvResult lResult = PvResult::Code::INVALID_PARAMETER;

	// Creates stream object
	cout << "Opening stream" << endl;
	PvStream* lStream = PvStream::CreateAndOpen(mConnectionID, &lResult);
	if ((lStream == NULL) || !lResult.IsOK())
	{
		cout << "Error creating and opening stream" << endl;
		PvStream::Free(lStream);
		return false;
	}

	// Get the stream parameters. These are used to configure/control
	// some stream related parameters and timings and provides
	// access to statistics from this stream.
	cout << "Retrieving stream's parameters array" << endl;
	PvGenParameterArray* lParameters = lStream->GetParameters();

	// Dumping device's parameters array content.
	cout << "Dumping stream's parameters array content" << endl;
	DumpGenParameterArray(lParameters);

	// Close and free PvStream
	PvStream::Free(lStream);

	return true;
}

bool Source::getAllSettings()
{
	cout << "GenCamParameter sample" << endl << endl;
	// Communication link parameters display.
	cout << "1. Communication link parameters display" << endl << endl;
	GetHostCommunicationRelatedSettings();

	cout << endl;

	// Device parameters display.
	cout << "2. Device parameters display" << endl << endl;
	GetDeviceSettings();

	cout << endl;

	// Image stream parameters display.
	cout << "3. Image stream parameters display" << endl << endl;
	GetImageStreamControllerSettings();

	return true;
}


//  Store device and stream configuration.
//  Also store a string information.
bool Source::StoreConfiguration(const PvString& filename)
{
	PvResult lResult;
	PvConfigurationWriter lWriter;

	// Create the Buffer and fill it.
	cout << "Store the configuration" << endl << endl;
	// Connect to the GigE Vision or USB3 Vision device
	cout << "Connecting to device" << endl;
	PvDevice *lDevice = PvDevice::CreateAndConnect(mConnectionID, &lResult);
	if (!lResult.IsOK())
	{
		cout << "Unable to connect to device" << endl;
		PvDevice::Free(lDevice);
		return false;
	}

	// Store  with a PvDevice.
	cout << "Store device configuration" << endl;
	lWriter.Store(lDevice, DEVICE_CONFIGURATION_TAG);

	// Create and open PvStream
	cout << "Store stream configuration" << endl;
	PvStream *lStream = PvStream::CreateAndOpen(mConnectionID, &lResult);
	if (!lResult.IsOK())
	{
		cout << "Unable to open stream object from device" << endl;
		lDevice->Disconnect();
		PvDevice::Free(lDevice);
		return false;
	}

	// Store with a PvStream
	lWriter.Store(lStream, STREAM_CONFIGURAITON_TAG);

	lWriter.Save(filename);

	return true;
}

bool Source::RestoreConfiguration(const PvString& filename)
{
	cout << "Restore device configuration" << endl << endl;
	if (!RestoreDevice(filename))
	{
		cout << "Cannot restore the configuration correctly" << endl;
		return false;
	}

	cout << endl;
	cout << "Restore stream configuration" << endl << endl;
	if (!RestoreStream(filename))
	{
		cout << "Cannot restore the configuration correctly" << endl;
		return false;
	}

	/*cout << endl;
	cout << "Restore string information" << endl << endl;
	if (!RestoreString(filename))
	{
		cout << "Cannot restore the configuration correctly";
		return false;
	}*/

	return true;
}

// Restore device configuration and verify that the device is connected. 
bool Source::RestoreDevice(const PvString& filename)
{
	PvConfigurationReader lReader;

	// Load all the information into a reader.
	cout << "Load information and configuration" << endl;
	lReader.Load(filename);

	PvDeviceGEV lDeviceGEV;
	PvDeviceU3V lDeviceU3V;
	PvDevice *lDevice = NULL;

	cout << "Restore configuration for a device with the configuration name" << endl;

	// Attempt restoring as a GEV device
	PvResult lResult = lReader.Restore(DEVICE_CONFIGURATION_TAG, &lDeviceGEV);
	if (lResult.IsOK())
	{
		// Success, keep PvDevice pointer to GEV device
		lDevice = &lDeviceGEV;
	}
	else
	{
		// Attempt restoring as a U3V device
		lResult = lReader.Restore(DEVICE_CONFIGURATION_TAG, &lDeviceU3V);
		if (lResult.IsOK())
		{
			// Success, keep PvDevice pointer to U3V device
			lDevice = &lDeviceU3V;
		}
		else
		{
			return false;
		}
	}

	cout << "Verify operation success" << endl;
	if (!lDevice->IsConnected())
	{
		return false;
	}

	lDevice->Disconnect();

	return true;
}

// Restore stream configuration and verify that the stream is open.
bool Source::RestoreStream(const PvString& filename)
{
	PvConfigurationReader lReader;

	// Load all the information into a reader.
	cout << "Load information and configuration" << endl;
	lReader.Load(filename);

	PvStreamGEV lStreamGEV;
	PvStreamU3V lStreamU3V;
	PvStream *lStream = NULL;

	cout << "Restore configuration for a stream with the configuration name" << endl;

	// Attempt restoring as a GEV Stream
	PvResult lResult = lReader.Restore(STREAM_CONFIGURAITON_TAG, &lStreamGEV);
	if (lResult.IsOK())
	{
		// Success, keep PvStream pointer to GEV Stream
		lStream = &lStreamGEV;
	}
	else
	{
		// Attempt restoring as a U3V Stream
		lResult = lReader.Restore(STREAM_CONFIGURAITON_TAG, &lStreamU3V);
		if (lResult.IsOK())
		{
			// Success, keep PvStream pointer to U3V Stream
			lStream = &lStreamU3V;
		}
		else
		{
			return false;
		}
	}

	cout << "Verify operation success" << endl;
	if (!lStream->IsOpen())
	{
		return false;
	}

	lStream->Close();

	return true;
}