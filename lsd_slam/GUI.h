/*
 * GUI.h
 *
 *  Created on: 15 Aug 2014
 *      Author: thomas
 */

#ifndef GUI_H_
#define GUI_H_

#define GLM_FORCE_RADIANS

#include <pangolin/pangolin.h>
#include <map>
#include "util/resolution.h"
#include "util/intrinsics.h"
#include "io_wrapper/Pangolin/Keyframe.h"
#include "util/thread_mutex_object.h"
#include "model/frame.h"
#include <mutex>

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

class GUI
{
    public:
        GUI();

        virtual ~GUI();

        void initImages();

        void preCall();

        void drawFrustum();

        void postCall();

        void addKeyframe(Keyframe * newFrame);

        void updateImage(unsigned char * data);

        void updateKeyframePoses(GraphFramePose* framePoseData, int num);

        void drawKeyframes();

        void drawImages();

        Sophus::Sim3f *pose;
        std::mutex pose_mutex;

    private:
        void drawGrid();

        pangolin::GlTexture * depthImg;

        ThreadMutexObject<unsigned char * > depthImgBuffer;

        pangolin::Var<int> * gpuMem;

        pangolin::Var<std::string> * totalPoints;

        pangolin::OpenGlRenderState s_cam;

        ThreadMutexObject<std::map<int, Keyframe *> > keyframes;
        //ThreadMutexObject<std::map<int, Keyframe*, std::less<int>,
        //  Eigen::aligned_allocator<std::pair<const int, Keyframe*>>>> keyframes;

};


#endif /* GUI_H_ */
