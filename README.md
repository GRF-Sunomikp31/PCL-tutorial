## PCL

## 1.Clion配置PCL环境

（1）.查看PCL版本

在/usr/share下查看到pcl文件夹为：pcl-1.12

（2）.配置Clion中pcl的cmakelist的环境

```cmake
cmake_minimum_required(VERSION 3.23)
project(Pcl_test)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.3.7 REQUIRED)
find_package(PCL 1.12 REQUIRED)
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Pcl_test main.cpp)
target_link_libraries (Pcl_test
        ${PCL_LIBRARIES}
        ${EIGEN3_LIBS}
)
```

PCL和Eigen的Cmake配置。

## 2.点云格式文件

一般输出的点云文件格式为.ply

软件：

- cloudcompare：
- meshlab：

点云格式：彼此之间是可以相互转换的

- .ply：ply代表Polygon（多边形），以多边形的方式保存了空间点位信息；PLY 不支持保存成有序点云格式。
- .pcd：保存有序点云格式
- txt：
- csv：

## 3.启动CloudCompare

Input：`cloudcompare.CloudCompare`

## 4.常用代码

#### （1）点云可视化

```c++
//PCL的可视化可以输入 pcd文件和ply文件。

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>         // std::this_thread::sleep_for

using namespace pcl;
using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //保存转换的输入点云
    pcl::io::loadPCDFile("2.pcd", *point_cloud);
    cout << "加载完成点云" << endl; 
    
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(point_cloud, "z"); // 按照z字段进行渲染
	viewer->addPointCloud<pcl::PointXYZ>(point_cloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小
	while (!viewer->wasStopped())
	{
    	viewer->spinOnce(100);
    	std::this_thread::sleep_for(100ms);
	}

	return 0;
}
```
#### （2）点云格式转换

```c++
//ply文件转pcd文件
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
	pcl::PCLPointCloud2 cloud;
	//加载ply文件
	pcl::PLYReader reader;
	reader.read("1.ply", cloud);
	//将ply文件保存为pcd文件
	pcl::PCDWriter writer;
	writer.write("1.pcd", cloud);
	
	return 0;
}
```

其他参考：https://blog.csdn.net/qq_39748832/article/details/108719277

## 5.点云库

open3d：Python和C++均可；

PLC：C++下