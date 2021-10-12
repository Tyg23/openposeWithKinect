#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>


using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudFromText(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(filename);
    assert(file.is_open());
    std::string line;
    
    while (std::getline(file,line))
    {
        pcl::PointXYZ point;
        std::stringstream ss(line);
        ss>>point.x>>point.y>>point.z;
        cloud->push_back(point);
    }
    return cloud;
}

bool pointPlane(float a, float b, float c, float d, float x, float y, float z, float distance)
{
    float dis=abs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);
    if(dis<=distance)
        return true;
    else
        return false;
}

bool clip()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
    cloud=getCloudFromText("segedArm.txt");


    std::ifstream file("clipXYZ.txt");
    assert(file.is_open());
    std::string line;
    int n=0;
    for (size_t i = 0; i < 16; i++)
    {
        std::getline(file,line);
        pcl::PointXYZ point;
        std::stringstream ss(line);
        ss>>point.x>>point.y>>point.z;
        cloud1->push_back(point);
    }
    for (size_t i = 0; i < 16; i++)
    {
        std::getline(file,line);
        pcl::PointXYZ point;
        std::stringstream ss(line);
        ss>>point.x>>point.y>>point.z;
        cloud2->push_back(point);
    }
    for (size_t i = 0; i < 16; i++)
    {
        std::getline(file,line);
        pcl::PointXYZ point;
        std::stringstream ss(line);
        ss>>point.x>>point.y>>point.z;
        cloud3->push_back(point);
    }
    for (size_t i = 0; i < 16; i++)
    {
        std::getline(file,line);
        pcl::PointXYZ point;
        std::stringstream ss(line);
        ss>>point.x>>point.y>>point.z;
        cloud4->push_back(point);
    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    float* A=new float[3];
    float* B=new float[3];
    float* C=new float[3];
    float* D=new float[3];

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5);

    seg.setInputCloud(cloud1);
    seg.segment (*inliers, *coefficients);
    A[0]=coefficients->values[0];
    B[0]=coefficients->values[1];
    C[0]=coefficients->values[2];
    D[0]=coefficients->values[3];    
    for (size_t i = 0; i < 4; i++)
    {
        std::cout << coefficients->values[i] << std::endl;
    }

    seg.setInputCloud(cloud2);
    seg.segment (*inliers, *coefficients);
    A[1]=coefficients->values[0];
    B[1]=coefficients->values[1];
    C[1]=coefficients->values[2];
    D[1]=coefficients->values[3];
    for (size_t i = 0; i < 4; i++)
    {
        std::cout << coefficients->values[i] << std::endl;
    }

    seg.setInputCloud(cloud3);
    seg.segment (*inliers, *coefficients);
    A[2]=coefficients->values[0];
    B[2]=coefficients->values[1];
    C[2]=coefficients->values[2];
    D[2]=coefficients->values[3];
    for (size_t i = 0; i < 4; i++)
    {
        std::cout << coefficients->values[i] << std::endl;
    }

    seg.setInputCloud(cloud4);
    seg.segment (*inliers, *coefficients);
    A[3]=coefficients->values[0];
    B[3]=coefficients->values[1];
    C[3]=coefficients->values[2];
    D[3]=coefficients->values[3];
    for (size_t i = 0; i < 4; i++)
    {
        std::cout << coefficients->values[i] << std::endl;
    }

    std::ifstream file2("featureXYZ.txt");
    assert(file2.is_open());
    std::string line2;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZ>);

    while (std::getline(file2,line2))
    {
        pcl::PointXYZ point;
        std::stringstream ss(line2);
        ss>>point.x>>point.y>>point.z;
        cloud5->push_back(point);
    }

    int index1=0,index2=1;
    float distance=sqrt(pow((cloud5->points[index1].x-cloud5->points[index2].x),2)+pow((cloud5->points[index1].y-cloud5->points[index2].y),2)
    +pow((cloud5->points[index1].z-cloud5->points[index2].z),2));
    std::cout<<"distance: "<<distance<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr segedcloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->size(); i++)
    {
        if(pointPlane(A[0],B[0],C[0],D[0],cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,distance)
        &&pointPlane(A[1],B[1],C[1],D[1],cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,distance)
        )
        {
            segedcloud->push_back(cloud->points[i]);
        }
    }

    int index3=1,index4=2;
    float distance2=sqrt(pow((cloud5->points[index3].x-cloud5->points[index4].x),2)+pow((cloud5->points[index3].y-cloud5->points[index4].y),2)
    +pow((cloud5->points[index3].z-cloud5->points[index4].z),2));
    std::cout<<"distance2: "<<distance2<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr segedcloud2(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->size(); i++)
    {
        if(pointPlane(A[2],B[2],C[2],D[2],cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,distance2)
        &&pointPlane(A[3],B[3],C[3],D[3],cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,distance2)
        )
        {
            segedcloud2->push_back(cloud->points[i]);
        }
    }
    pcl::io::savePLYFile("segedArm1.ply",*segedcloud);
    pcl::io::savePLYFile("segedArm2.ply",*segedcloud2);
    return true;
}
    