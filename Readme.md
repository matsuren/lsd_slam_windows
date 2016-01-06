# LSD-SLAM for Windows Fork

This fork simplifies building LSD-SLAM on Windows with [Visual Studio 2013 (VS12)](http://www.visualstudio.com).

# Prerequisites

Install pre-build binaries of [OpenCV 3.0](http://sourceforge.net/projects/opencvlibrary/files/opencv-win/3.0.0/opencv-3.0.0.exe/download) 
and [Boost Library 1.59.0 x64 fro VS 12](http://sourceforge.net/projects/boost/files/boost-binaries/1.59.0/boost_1_59_0-msvc-12.0-64.exe/download).

# Building the Solution

Open LSD-SLAM.sln in Visual Studio 2013 and correct the paths in the LSD-SLAM.props which can be found in the 'Settings' folder of the solution.

# LSD-SLAM: Large-Scale Direct Monocular SLAM

LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct (i.e. does not use keypoints / features) and creates large-scale, 
semi-dense maps in real-time on a laptop. For more information see
[http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
where you can also find the corresponding publications and Youtube videos, as well as some 
example-input datasets, and the generated output as rosbag or .ply point cloud.

### Related Papers

* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13

# License
LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

For commercial purposes, the original lsd slam authors also offer a professional version under different licencing terms.
