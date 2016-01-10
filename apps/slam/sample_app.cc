#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "live_slam_wrapper.h"

#include "util/settings.h"
#include "util/global_funcs.h"

//#include "util/undistorter.h"
//#include "io_wrapper/OpenCVImageStreamThread.h"
#include "VideoInputImageStream.h"
#include "slam_system.h"
#include "DebugOutput3DWrapper.h"

//#include "videoInput.h"

using namespace std;
using namespace lsd_slam;
char key;


// print symbolic names of available video capture devices
void PrintDeviceNames(const std::vector<Device> deviceList)
{
	printf("Available video capture devices:\n");
	for (int i = 0; i < deviceList.size(); ++i)
	{
		const Device& device = deviceList[i];
		printf("%ws \"%ws\"\n", device.symbolicName.c_str(), device.friendlyName.c_str());
	}
}


DeviceSettings FindCompatibleMediaType(Device dev, int stream, int width, int height)
{
	if (stream < 0 || stream > dev.listStream.size())
		throw new std::exception("Stream not available");

	const std::vector<MediaType>& listMediaType = dev.listStream[stream].listMediaType;
	for (int k = 0; k < listMediaType.size(); ++k)
	{
		const MediaType& mt = listMediaType[k];

		// search for compatible media type
		if (mt.width == width && mt.height == height)
		{
			DeviceSettings ds;
			ds.indexStream = stream;
			ds.indexMediaType = k;
			ds.symbolicLink = dev.symbolicName;
			return ds;
		}
	}
}


Device FindDevice(const std::wstring symbolicName, const std::vector<Device>& deviceList)
{
	for (int i = 0; i < deviceList.size(); ++i)
	{
		const Device& dev = deviceList[i];
		if (symbolicName.empty() || dev.symbolicName == symbolicName)
			return dev;
	}

	throw new std::exception("Device not found");
}


std::wstring InitializeVideoInput()
{
	videoInput& vInput = videoInput::getInstance();
	vInput.setVerbose(false);

	std::vector<Device> deviceList;
	vInput.getListOfDevices(deviceList);

	if (deviceList.empty()) {
		throw std::exception("No video capture device available.");
	}

	Device dev = FindDevice(L"", deviceList);

	DeviceSettings deviceSettings = FindCompatibleMediaType(dev, 0, 640, 480);

	CaptureSettings captureSettings;
	captureSettings.pIStopCallback = 0;
	captureSettings.videoFormat = CaptureVideoFormat::RGB32;
	captureSettings.readMode = ReadMode::SYNC;

	vInput.setupDevice(deviceSettings, captureSettings);

	return dev.symbolicName;
}


void VideoTest()
{
	std::wstring symbolicLink = InitializeVideoInput();
	videoInput& vInput = videoInput::getInstance();

	cvNamedWindow("VideoTest", CV_WINDOW_AUTOSIZE);

	IplImage* frame = cvCreateImage(cvSize(640, 480), 8, 4);

	ReadSetting readSetting = {};
	readSetting.symbolicLink = symbolicLink;
	readSetting.pPixels = reinterpret_cast<unsigned char *>(frame->imageData);

	while (1)
	{
		vInput.readPixels(readSetting);

		cvShowImage("VideoTest", frame);

		char c = cvWaitKey(33);
		if (c == 27)
			break;
	}

	cvDestroyWindow("VideoTest");
	cvReleaseImage(&frame);

	vInput.closeAllDevices();
}


int main(int argc, char** argv) {

	std::wstring symbolicLink = InitializeVideoInput();
	videoInput& vInput = videoInput::getInstance();


	std::string calib_fn = "data\\out_camera_data.xml";

	VideoInputImageStream inputStream(symbolicLink);
	inputStream.setCalibration(calib_fn);
	inputStream.run();

	{
		DebugOutput3DWrapper outputWrapper(inputStream.width(), inputStream.height());
		LiveSLAMWrapper slamNode(&inputStream, &outputWrapper);

		slamNode.Loop();
	}

	vInput.closeAllDevices();

	return 0;
}
