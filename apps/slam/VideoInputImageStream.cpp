#include "VideoInputImageStream.h"
#include <opencv/cv.h>
#include <boost/thread.hpp>
#include "lsd_slam/util/undistorter.h"


VideoInputImageStream::VideoInputImageStream(const std::wstring& symbolicDeviceName)
	: haveCalib(false)
	, undistorter(nullptr)
{
	readSetting.symbolicLink = symbolicDeviceName;
	width_ = 0;
	height_ = 0;
	imageBuffer = new lsd_slam::NotifyBuffer<lsd_slam::TimestampedMat>(8);
}


VideoInputImageStream::~VideoInputImageStream()
{
	delete imageBuffer;
}


void VideoInputImageStream::run()
{
	boost::thread thread(boost::ref(*this));
}


void VideoInputImageStream::setCalibration(const std::string& file)
{
	if (file == "")
	{
		printf("NO camera calibration file!\n");
	}
	else
	{
		undistorter = lsd_slam::Undistorter::getUndistorterForFile(file.c_str());

		if (!undistorter)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			assert("Failed to read camera calibration from file... wrong syntax?");
		}

		fx_ = undistorter->getK().at<double>(0, 0);
		fy_ = undistorter->getK().at<double>(1, 1);
		cx_ = undistorter->getK().at<double>(2, 0);
		cy_ = undistorter->getK().at<double>(2, 1);

		width_ = undistorter->getOutputWidth();
		height_ = undistorter->getOutputHeight();
	}

	haveCalib = true;
}


void VideoInputImageStream::operator()()
{
	if (!haveCalib)
	{
		assert("no calibration");
		return;
	}

	videoInput& vInput = videoInput::getInstance();

	while (1)
	{
		lsd_slam::TimestampedMat bufferItem;
		bufferItem.timestamp = lsd_slam::Timestamp::now();

		IplImage* frame = cvCreateImage(cvSize(640, 480), 8, 4);
		readSetting.pPixels = reinterpret_cast<unsigned char *>(frame->imageData);

		ResultCode::Result r = vInput.readPixels(readSetting);
		if (r != ResultCode::READINGPIXELS_DONE)
			break;

		if (undistorter != nullptr)
		{
			assert(undistorter->isValid());
			cv::Mat m = cv::cvarrToMat(frame, false);
			undistorter->undistort(m, bufferItem.data);
		}
		else
		{
			bufferItem.data = cv::cvarrToMat(frame, false);
		}

		//bufferItem.data = cv::Mat(frame, true);
		imageBuffer->pushBack(bufferItem);
	}

	exit(1);
}
