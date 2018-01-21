/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

//int main(){
//  std::cout << "AAAAAAAAA";
//  return 0;
//}

#include "live_slam_wrapper.h"

#include "util/settings.h"
#include "util/global_funcs.h"
#include "util/thread_mutex_object.h"
#include "io_wrapper/image_display.h"
#include "io_wrapper/Pangolin/PangolinOutput3DWrapper.h"
#include "slam_system.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/undistorter.h"
#include "opencv2/opencv.hpp"

#include "GUI.h"

std::vector<std::string> files;
int w, h, w_inp, h_inp;
ThreadMutexObject<bool> lsdDone(false);
GUI *gui;
int numFrames = 0;

std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}
std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}
std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}
int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
	std::ifstream f(source.c_str());

	if(f.good() && f.is_open())
	{
		while(!f.eof())
		{
			std::string l;
			std::getline(f,l);

			l = trim(l);

			if(l == "" || l[0] == '#')
				continue;

			files.push_back(l);
		}

		f.close();

		size_t sp = source.find_last_of('/');
		std::string prefix;
		if(sp == std::string::npos)
			prefix = "";
		else
			prefix = source.substr(0,sp);

		for(unsigned int i=0;i<files.size();i++)
		{
			if(files[i].at(0) != '/')
				files[i] = prefix + "/" + files[i];
		}

		return (int)files.size();
	}
	else
	{
		f.close();
		return -1;
	}

}

using namespace lsd_slam;

void run(SlamSystem * system, Undistorter* undistorter, Output3DWrapper* outputWrapper, Sophus::Matrix3f K)
{
    // get HZ
    double hz = 30;

    cv::Mat image = cv::Mat(h, w, CV_8U);
    cv::Mat imageRGB = cv::Mat(h, w, CV_8UC3);

    int runningIDX=0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; i < numFrames; i++)
    {
        if(lsdDone.getValue())
            break;

        cv::Mat imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat imageDistRGB = cv::imread(files[i], CV_LOAD_IMAGE_COLOR);

        if(imageDist.rows != h_inp || imageDist.cols != w_inp)
        {
          if(imageDist.rows * imageDist.cols == 0)
            printf("failed to load image %s! skipping.\n", files[i].c_str());
          else
            printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
              files[i].c_str(),
              w, h, imageDist.cols, imageDist.rows);
          continue;
        }

        assert(imageDist.type() == CV_8U);
        assert(imageDistRGB.type() == CV_8UC3);
        undistorter->undistort(imageDist, image);
        undistorter->undistort(imageDistRGB, imageRGB);
        assert(image.type() == CV_8U);
        assert(imageRGB.type() == CV_8UC3);

        Util::displayImage("MyVideo", image);
        Util::waitKey(4);

        if(runningIDX == 0)
        {
          system->randomInit(image.data, imageRGB.data, fakeTimeStamp, runningIDX);
        }
        else
        {
          system->trackFrame(image.data, imageRGB.data, runningIDX, hz == 0, fakeTimeStamp);
        }
        
        std::unique_lock<std::mutex>  lock(gui->pose_mutex);
        gui->pose = &system->getCurrentPoseEstimateScale();
        lock.unlock();

        runningIDX++;
        fakeTimeStamp+=0.03;

        //if(hz != 0)
        //	r.sleep();

        if(fullResetRequested)
        {
            printf("FULL RESET!\n");
            delete system;

            system = new SlamSystem(w, h, K, doSlam);
            system->setVisualization(outputWrapper);

            fullResetRequested = false;
            runningIDX = 0;
        }
    }

    lsdDone.assignValue(true);
}

int main()
{
	// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	std::string calibFile;
	Undistorter* undistorter = 0;

  const std::string fn = "C:/Users/KOMATSU/Documents/MyPrograms/lsd_slam_windows/LSD_room/cameraCalibration.cfg";
  undistorter = Undistorter::getUndistorterForFile(fn.c_str());

	if(undistorter == 0)
	{
		printf("need camera calibration file! (set using -c FILE)\n");
		exit(0);
	}

	w = undistorter->getOutputWidth();
	h = undistorter->getOutputHeight();

	w_inp = undistorter->getInputWidth();
	h_inp = undistorter->getInputHeight();

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	Resolution::getInstance(w, h);
	Intrinsics::getInstance(fx, fy, cx, cy);

  //////
  gui = new GUI();
	gui->initImages();

  Output3DWrapper* outputWrapper = new PangolinOutput3DWrapper(w, h, *gui);

	// make slam system
	SlamSystem * system = new SlamSystem(w, h, K, doSlam);
	system->setVisualization(outputWrapper);


	// open image files: first try to open as file.
  for(int i = 1; i < 2702; i++){
    std::ostringstream sout;
    sout << std::setfill('0') << std::setw(5) << i;
    std::string s = sout.str();
    std::string filename = "C:/Users/KOMATSU/Documents/MyPrograms/lsd_slam_windows/LSD_room/images/" + s + ".png";
    files.push_back(filename);
  }
  numFrames = (int)files.size();
	std::thread lsdThread(run, system, undistorter, outputWrapper, K);

	while(!pangolin::ShouldQuit())
	{
	    if(lsdDone.getValue() && !system->finalized)
	    {
	        system->finalize();
	    }
      CheckGlDieOnError();
	    gui->preCall();
      CheckGlDieOnError();
	    gui->drawKeyframes();
      CheckGlDieOnError();
	    gui->drawFrustum();
      CheckGlDieOnError();
	    gui->drawImages();
      CheckGlDieOnError();
	    gui->postCall();
	}

	lsdDone.assignValue(true);

	lsdThread.join();

	delete system;
	delete undistorter;
	delete outputWrapper;
	return 0;
}
