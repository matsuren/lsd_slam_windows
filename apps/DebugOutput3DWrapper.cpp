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

#include "DebugOutput3DWrapper.h"
#include "lsd_slam/util/sophus_util.h"
#include "lsd_slam/util/settings.h"

//#include "lsd_slam_viewer/keyframeGraphMsg.h"
//#include "lsd_slam_viewer/keyframeMsg.h"

#include "lsd_slam/model/frame.h"
#include "lsd_slam/global_mapping/key_frame_graph.h"
#include "sophus/sim3.hpp"
#include "lsd_slam/global_mapping/g2o_type_sim3_sophus.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>

namespace lsd_slam
{


DebugOutput3DWrapper::DebugOutput3DWrapper(int width, int height)
{
	cv::namedWindow("Tracking_output", 1); //Create window
	this->width = width;
	this->height = height;

	/*liveframe_channel = nh_.resolveName("lsd_slam/liveframes");
	liveframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(liveframe_channel,1);

	keyframe_channel = nh_.resolveName("lsd_slam/keyframes");
	keyframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(keyframe_channel,1);

	graph_channel = nh_.resolveName("lsd_slam/graph");
	graph_publisher = nh_.advertise<lsd_slam_viewer::keyframeGraphMsg>(graph_channel,1);

	debugInfo_channel = nh_.resolveName("lsd_slam/debug");
	debugInfo_publisher = nh_.advertise<std_msgs::Float32MultiArray>(debugInfo_channel,1);

	pose_channel = nh_.resolveName("lsd_slam/pose");
	pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel,1);*/

	tracker_display = cv::Mat::ones(640, 480, CV_8UC1);
	cv::circle(tracker_display, cv::Point(100,100), 20, cv::Scalar(0, 255, 0));
	cv::imshow("Tracking_output", tracker_display);
	cvWaitKey(10);
	publishLvl=0;

	plyout.open("c:\\temp\test.ply", std::ios::in | std::ios::out | std::ios::ate);

}

struct PlyVertex
{
	Sophus::Vector3f p;
	uchar argb[4];

	PlyVertex(const Sophus::Vector3f pos, uchar a, uchar r, uchar g, uchar b)
		: p(pos)
	{
		argb[0] = a;
		argb[1] = r;
		argb[2] = g;
		argb[3] = b;
	}
};


class PlyWriter
{
public:
	static void Write(const std::string& fn, std::vector<PlyVertex> vertices)
	{
		std::ofstream f;
		f.open(fn, std::ios::out);
	
		// create header
		f << "ply" << std::endl;
		f << "format ascii 1.0" << std::endl;
		f << "comment bla" << std::endl;
		f << "element vertex " << vertices.size() << std::endl;
		f << "property float x" << std::endl;
		f << "property float y" << std::endl;
		f << "property float z" << std::endl;
		f << "property uchar intensity" << std::endl;
		f << "property uchar red" << std::endl;
		f << "property uchar green" << std::endl;
		f << "property uchar blue" << std::endl;
		f << "end_header" << std::endl;
		
		// write vertices
		for (auto v : vertices)
		{
			float x = v.p.x();
			float y = v.p.y();
			float z = v.p.z();
			int a = v.argb[0];
			int r = v.argb[1];
			int g = v.argb[2];
			int b = v.argb[3];
			f << x << " " << y << " " << z << " " << a << " " << r << " " << g << " " << b << std::endl;
		}

		f.close();
	}

};

DebugOutput3DWrapper::~DebugOutput3DWrapper()
{
}



void DebugOutput3DWrapper::publishKeyframe(Frame* f)
{
	KeyFrameMessage fMsg;


	std::shared_lock<std::shared_timed_mutex> lock = f->getActiveLock();

	fMsg.id = f->id();
	fMsg.time = f->timestamp();
	fMsg.isKeyframe = true;

	int w = f->width(publishLvl);
	int h = f->height(publishLvl);

	memcpy(fMsg.camToWorld.data(), f->getScaledCamToWorld().cast<float>().data(), sizeof(float)*7);
	fMsg.fx = f->fx(publishLvl);
	fMsg.fy = f->fy(publishLvl);
	fMsg.cx = f->cx(publishLvl);
	fMsg.cy = f->cy(publishLvl);
	fMsg.width = w;
	fMsg.height = h;

	float fx = fMsg.fx;
	float fy = fMsg.fy;
	float cx = fMsg.cx;
	float cy = fMsg.cy;

	float fxi = 1 / fx;
	float fyi = 1 / fy;
	float cxi = -cx / fx;
	float cyi = -cy / fy;

	fMsg.pointcloud.resize(w*h*sizeof(InputPointDense));

	InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data();

	const float* idepth = f->idepth(publishLvl);
	const float* idepthVar = f->idepthVar(publishLvl);
	const float* color = f->image(publishLvl);
	const float* colorRGB = f->imageRGB(publishLvl);
	
	// create vertex buffer
	std::vector<PlyVertex> vb;

	const Sophus::Sim3f& camToWorld = fMsg.camToWorld;
	int idx = 0;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x, ++idx)
		{
			pc[idx].idepth = idepth[idx];
			pc[idx].idepth_var = idepthVar[idx];
			pc[idx].color[0] = color[x + y*w] * 255;
			pc[idx].color[1] = colorRGB[(x + y*w) * 3 + 2] * 255;
			pc[idx].color[2] = colorRGB[(x + y*w) * 3 + 1] * 255;
			pc[idx].color[3] = colorRGB[(x + y*w) * 3 + 0] * 255;

			if (idepth[idx] > 0)
			{
				float depth = 1 / idepth[idx];
				float depth4 = depth*depth; depth4 *= depth4;

				if (idepthVar[idx] * depth4 < 1E-3)
				{
					// transform ..
					Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);
					int my_minNearSupport = 4;
					int nearSupport = 0;
					if (my_minNearSupport > 1)
					{

						for (int dx = -1; dx < 2; dx++)
							for (int dy = -1; dy < 2; dy++)
							{
								int idx = x + dx + (y + dy)*width;
								if (idepth[idx] > 0)
								{
									float diff = idepth[idx] - 1.0f / depth;
									if (diff*diff < 2 * idepthVar[x + y*width])
										nearSupport++;
								}
							}

					}
					if (nearSupport > my_minNearSupport)
					{
						vb.push_back(PlyVertex(pt, pc[idx].color[0], pc[idx].color[1], pc[idx].color[2], pc[idx].color[3]));
					}
				}
			}
		}
	}

	static int seq = 0;
	PlyWriter::Write(std::string("c:\\temp\\") + "test" + std::to_string(seq) + ".ply", vb);
	++seq;
	
	// writePointCloud(fMsg, );


	std::cout << "PublishKeyframe" << std::endl;
}

void DebugOutput3DWrapper::publishTrackedFrame(Frame* kf)
{
	KeyFrameMessage fMsg;


	fMsg.id = kf->id();
	fMsg.time = kf->timestamp();
	fMsg.isKeyframe = false;


	memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	fMsg.fx = kf->fx(publishLvl);
	fMsg.fy = kf->fy(publishLvl);
	fMsg.cx = kf->cx(publishLvl);
	fMsg.cy = kf->cy(publishLvl);
	fMsg.width = kf->width(publishLvl);
	fMsg.height = kf->height(publishLvl);

	/*fMsg.pointcloud.clear();

	liveframe_publisher.publish(fMsg);*/


	SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());

	/*geometry_msgs::PoseStamped pMsg;

	pMsg.pose.position.x = camToWorld.translation()[0];
	pMsg.pose.position.y = camToWorld.translation()[1];
	pMsg.pose.position.z = camToWorld.translation()[2];
	pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
	pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
	pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
	pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

	if (pMsg.pose.orientation.w < 0)
	{
		pMsg.pose.orientation.x *= -1;
		pMsg.pose.orientation.y *= -1;
		pMsg.pose.orientation.z *= -1;
		pMsg.pose.orientation.w *= -1;
	}

	pMsg.header.stamp = ros::Time(kf->timestamp());
	pMsg.header.frame_id = "world";
	pose_publisher.publish(pMsg);*/

	
	cv::circle(tracker_display, cv::Point(320+camToWorld.translation()[0]*640, -240 + camToWorld.translation()[1]*480), 2, cv::Scalar(255, 0, 0),4);
	cv::imshow("Tracking_output", tracker_display);
	std::cout << "PublishTrackedKeyframe: " << camToWorld.translation()[0] << " " << camToWorld.translation()[1] << "  " << camToWorld.translation()[2] << std::endl;
}



void DebugOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{
	/*lsd_slam_viewer::keyframeGraphMsg gMsg;

	graph->edgesListsMutex.lock();
	gMsg.numConstraints = graph->edgesAll.size();
	gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
	GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data();
	for(unsigned int i=0;i<graph->edgesAll.size();i++)
	{
		constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
		constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
		Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
		constraintData[i].err = sqrt(err.dot(err));
	}
	graph->edgesListsMutex.unlock();

	graph->keyframesAllMutex.lock_shared();
	gMsg.numFrames = graph->keyframesAll.size();
	gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
	GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
	for(unsigned int i=0;i<graph->keyframesAll.size();i++)
	{
		framePoseData[i].id = graph->keyframesAll[i]->id();
		memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	}
	graph->keyframesAllMutex.unlock_shared();

	graph_publisher.publish(gMsg);*/
}

void DebugOutput3DWrapper::publishTrajectory(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void DebugOutput3DWrapper::publishTrajectoryIncrement(const Eigen::Matrix<float, 3, 1>& pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void DebugOutput3DWrapper::publishDebugInfo(const Eigen::Matrix<float, 20, 1>& data)
{
	//std_msgs::Float32MultiArray msg;
	for(int i=0;i<20;i++)
		std::cout << (float)(data[i]) << std::endl;

	//debugInfo_publisher.publish(msg);
}

//void draw_target(cv::Mat& rgb_img, look3d::PanoramicTracker& tracker) {
//	const Eigen::Vector4d point_x(0.1, 0, 1, 1);
//	const Eigen::Vector4d point_y(0, 0.1, 1, 1);
//	const Eigen::Vector4d point_z(0, 0, 1.1, 1);
//	const Eigen::Vector4d point_target(0, 0, 1.0, 1);
//
//	Eigen::Matrix<double, 3, 4> proj = get_projection(tracker);
//
//	Eigen::Vector3d point_cam = proj * point_target;
//	Eigen::Vector3d pointx_cam = proj * point_x;
//	Eigen::Vector3d pointy_cam = proj * point_y;
//	Eigen::Vector3d pointz_cam = proj * point_z;
//
//	cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]), cv::Point(pointx_cam[0], pointx_cam[1]), cv::Scalar(255, 0, 0), 3);
//	cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]), cv::Point(pointy_cam[0], pointy_cam[1]), cv::Scalar(0, 255, 0), 3);
//	cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]), cv::Point(pointz_cam[0], pointz_cam[1]), cv::Scalar(0, 0, 255), 3);
//}

}
