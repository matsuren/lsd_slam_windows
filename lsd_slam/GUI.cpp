/*
 * GUI.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#include "GUI.h"

GUI::GUI()
 : depthImg(0),
   depthImgBuffer(0)
{
    //pangolin::CreateGlutWindowAndBind("Main", 1280 + 180, 960, GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
    pangolin::CreateWindowAndBind("Main", 1000 + 180, 720);

    glDisable(GL_MULTISAMPLE);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                                        pangolin::ModelViewLookAt(-1, -5, -1, 0, 0, 0, pangolin::AxisNegY));

    pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
                            .SetHandler(new pangolin::Handler3D(s_cam));

    // Create Image View in window
    pangolin::Display("Image")
      .SetBounds(2 / 3.0f, 1.0f, 1 / 2.0f, 1.0, 640.0 / 480)
      .SetLock(pangolin::LockRight, pangolin::LockTop);
    //pangolin::Display("Image").SetAspect(640.0f / 480.0f);

    //pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), pangolin::Attach::Pix(360), pangolin::Attach::Pix(180), 1.0)
    //                          .SetLayout(pangolin::LayoutEqualVertical)
    //                          .AddDisplay(pangolin::Display("Image"));

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

    gpuMem = new pangolin::Var<int>("ui.GPU memory free", 0);

    totalPoints = new pangolin::Var<std::string>("ui.Total points", "0");

    pose = new Sophus::Sim3f;
}

GUI::~GUI()
{
    if(depthImg)
        delete depthImg;

    if(depthImgBuffer.getValue())
        delete [] depthImgBuffer.getValue();

    std::unique_lock<std::mutex> lock(keyframes.getMutex());

    for(auto i = keyframes.getReference().begin(); i != keyframes.getReference().end(); ++i)
    {
        delete i->second;
    }

    keyframes.getReference().clear();

    lock.unlock();

    delete totalPoints;
    delete gpuMem;
}

void GUI::initImages()
{
    depthImg = new pangolin::GlTexture(Resolution::getInstance().width(), Resolution::getInstance().height(), GL_RGB, true, 0, GL_RGB, GL_UNSIGNED_BYTE);

    depthImgBuffer.assignValue(new unsigned char[Resolution::getInstance().numPixels() * 3]);
}

void GUI::updateImage(unsigned char * data)
{
    std::unique_lock<std::mutex> lock(depthImgBuffer.getMutex());

    memcpy(depthImgBuffer.getReference(), data, Resolution::getInstance().numPixels() * 3);

    lock.unlock();
}

void GUI::preCall()
{
    glClearColor(0.05, 0.05, 0.3, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pangolin::Display("cam").Activate(s_cam);

    drawGrid();
}

void GUI::addKeyframe(Keyframe * newFrame)
{
    std::unique_lock<std::mutex>  lock(keyframes.getMutex());

    //Exists
    if(keyframes.getReference().find(newFrame->id) != keyframes.getReference().end())
    {
        keyframes.getReference()[newFrame->id]->updatePoints(newFrame);

        delete newFrame;
    }
    else
    {
        newFrame->initId = keyframes.getReference().size();
        keyframes.getReference()[newFrame->id] = newFrame;
    }

    lock.unlock();
}

void GUI::updateKeyframePoses(GraphFramePose* framePoseData, int num)
{
    std::unique_lock<std::mutex>  lock(keyframes.getMutex());

    for(int i = 0; i < num; i++)
    {
        if(keyframes.getReference().find(framePoseData[i].id) != keyframes.getReference().end())
        {
            memcpy(keyframes.getReference()[framePoseData[i].id]->camToWorld.data(), &framePoseData[i].camToWorld[0], sizeof(float) * 7);
        }
    }

    lock.unlock();
}

void GUI::drawImages()
{
    std::unique_lock<std::mutex>  lock(depthImgBuffer.getMutex());

    depthImg->Upload(depthImgBuffer.getReference(), GL_RGB, GL_UNSIGNED_BYTE);

    lock.unlock();

    pangolin::Display("Image").Activate();

    depthImg->RenderToViewport(true);
}

void GUI::drawKeyframes()
{
    std::unique_lock<std::mutex>  lock(keyframes.getMutex());

    glEnable(GL_MULTISAMPLE);

    //glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
    for(std::map<int, Keyframe *>::iterator i = keyframes.getReference().begin(); i != keyframes.getReference().end(); ++i)
    {
        //Don't render first five, according to original code
        if(i->second->initId >= 5)
        {
            if(!i->second->hasVbo || i->second->needsUpdate)
            {
                i->second->computeVbo();
            }
            i->second->drawPoints();
            i->second->drawCamera();
        }
    }

    glDisable(GL_MULTISAMPLE);
    lock.unlock();
}

void GUI::drawFrustum()
{
    glPushMatrix();
    std::unique_lock<std::mutex>  lock(pose_mutex);
    Sophus::Matrix4f m = pose->matrix();
    lock.unlock();
    glMultMatrixf((GLfloat*) m.data());
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0.05 * (0 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (0 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0, 0, 0);
        glVertex3f(0.05 * (0 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (Resolution::getInstance().height() - 1 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0, 0, 0);
        glVertex3f(0.05 * (Resolution::getInstance().width() - 1 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (Resolution::getInstance().height() - 1 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0, 0, 0);
        glVertex3f(0.05 * (Resolution::getInstance().width() - 1 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (0 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0.05 * (Resolution::getInstance().width() - 1 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (0 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0.05 * (Resolution::getInstance().width() - 1 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (Resolution::getInstance().height() - 1 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0.05 * (Resolution::getInstance().width() - 1 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (Resolution::getInstance().height() - 1 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0.05 * (0 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (Resolution::getInstance().height() - 1 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0.05 * (0 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (Resolution::getInstance().height() - 1 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0.05 * (0 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (0 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0.05 * (0 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (0 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
        glVertex3f(0.05 * (Resolution::getInstance().width() - 1 - Intrinsics::getInstance().cx()) / Intrinsics::getInstance().fx(), 0.05 * (0 - Intrinsics::getInstance().cy()) / Intrinsics::getInstance().fy(), 0.05);
    glEnd();
    glPopMatrix();
    glColor3f(1, 1, 1);
}

void GUI::drawGrid()
{
    //set pose
    glPushMatrix();

    Eigen::Matrix4f m;
    m <<  0,  0, 1, 0,
         -1,  0, 0, 0,
          0, -1, 0, 0,
          0,  0, 0, 1;
    glMultTransposeMatrixf((float*)m.data());

    glLineWidth(1);

    glBegin(GL_LINES);

    // Draw a larger grid around the outside..
    double dGridInterval = 0.1;

    double dMin = -100.0 * dGridInterval;
    double dMax = 100.0 * dGridInterval;

    double height = -4;

    for(int x = -10; x <= 10; x += 1)
    {
        if(x == 0)
            glColor3f(1, 1, 1);
        else
            glColor3f(0.3, 0.3, 0.3);
        glVertex3d((double) x * 10 * dGridInterval, dMin, height);
        glVertex3d((double) x * 10 * dGridInterval, dMax, height);
    }

    for(int y = -10; y <= 10; y += 1)
    {
        if(y == 0)
            glColor3f(1, 1, 1);
        else
            glColor3f(0.3, 0.3, 0.3);
        glVertex3d(dMin, (double) y * 10 * dGridInterval, height);
        glVertex3d(dMax, (double) y * 10 * dGridInterval, height);
    }

    glEnd();

    glBegin(GL_LINES);
    dMin = -10.0 * dGridInterval;
    dMax = 10.0 * dGridInterval;

    for(int x = -10; x <= 10; x++)
    {
        if(x == 0)
            glColor3f(1, 1, 1);
        else
            glColor3f(0.5, 0.5, 0.5);

        glVertex3d((double) x * dGridInterval, dMin, height);
        glVertex3d((double) x * dGridInterval, dMax, height);
    }

    for(int y = -10; y <= 10; y++)
    {
        if(y == 0)
            glColor3f(1, 1, 1);
        else
            glColor3f(0.5, 0.5, 0.5);
        glVertex3d(dMin, (double) y * dGridInterval, height);
        glVertex3d(dMax, (double) y * dGridInterval, height);
    }

    glColor3f(1, 0, 0);
    glVertex3d(0, 0, height);
    glVertex3d(1, 0, height);
    glColor3f(0, 1, 0);
    glVertex3d(0, 0, height);
    glVertex3d(0, 1, height);
    glColor3f(1, 1, 1);
    glVertex3d(0, 0, height);
    glVertex3d(0, 0, height + 1);
    glEnd();

    glPopMatrix();
}

void GUI::postCall()
{
    GLint cur_avail_mem_kb = 0;
#ifdef CUDA
    glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX, &cur_avail_mem_kb);
#endif

    int memFree = cur_avail_mem_kb / 1024;

    gpuMem->operator=(memFree);

    pangolin::FinishFrame();
}
