#pragma once

#include "videoInput.h"
#include "lsd_slam/io_wrapper/input_image_stream.h"


namespace lsd_slam
{
	class Undistorter;
}


class VideoInputImageStream
	: public lsd_slam::InputImageStream
{
public:
	VideoInputImageStream(const std::wstring& symbolicDeviceName);
	~VideoInputImageStream();

	void run();
	void setCalibration(const std::string& file);
	void operator()();

private:
	ReadSetting readSetting;
	bool haveCalib;
	lsd_slam::Undistorter* undistorter;
};
