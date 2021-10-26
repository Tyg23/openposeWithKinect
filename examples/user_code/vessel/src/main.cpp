// #include "tools/io_mesh.h"
#include "tools/OmpHelper.h"
#include "NonRigidreg.h"
#include <tools/io_mesh.h>
#include <detectBody.h>
#include <workutils.h>
#include <vtkOBJReader.h>
#include <vtkTransformFilter.h>
#include <vtkTransform.h>
#include <vtkSTLWriter.h>
#include <vtkOBJExporter.h>
#include <vtkMatrix4x4.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <fstream>
#include <Eigen/Eigen>

void match(std::string src, std::string tar,std::string matchName,int isUpperArm)
{
    Mesh src_mesh;
    Mesh tar_mesh;
    std::string src_file;
    std::string tar_file;
    std::string out_file, outpath;
    std::string landmark_file;
    RegParas paras;
    // Setting paras
    paras.alpha = 50.0;
    paras.beta = 50.0;
    paras.gamma = 1e8;
    paras.uni_sample_radio = 20.0; //重要参数

    paras.use_distance_reject = true;
    // paras.distance_threshold = 0.05;
    paras.distance_threshold = 0.5;
    paras.use_normal_reject = false;
    paras.normal_threshold = M_PI / 3;
    paras.use_Dynamic_nu = true;
    paras.rigid_iters = 100;

    src_file = src;
    tar_file = tar;
    outpath = "";
    paras.out_gt_file = outpath + matchName.c_str()+ "_res.txt";
    out_file = outpath +matchName.c_str()+ "res.obj";
    NonRigidreg *reg;
    reg = new NonRigidreg;
    reg->name1 = src_file.c_str();
    reg->name2 = tar_file.c_str();

    read_data(src_file, src_mesh); //读取source mesh数据
    read_data(tar_file, tar_mesh); //读取target mesh数据
    if (src_mesh.n_vertices() == 0 || tar_mesh.n_vertices() == 0)
        exit(0);
    if (src_mesh.n_vertices() != tar_mesh.n_vertices())
        paras.calc_gt_err = false;
    if (paras.use_landmark)
        read_landmark(landmark_file.c_str(), paras.landmark_src, paras.landmark_tar);
    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
    transform=workutils::initialAlign(tar_file,isUpperArm);
    std::string fileName;
    if(isUpperArm==1)
    {
        fileName="transformedTargetUpper.ply";
    }
    else
    {
        fileName="transformedTargetLower.ply";
    }
    workutils::read_data(fileName,tar_mesh);
    double scale = mesh_scaling(src_mesh, tar_mesh);

    Timer time;
    std::cout << "\nrigid registration to initial..." << std::endl;
    Timer::EventID time1 = time.get_time();
    reg->rigid_init(src_mesh, tar_mesh, paras);
    reg->DoRigid();
    Timer::EventID time2 = time.get_time();
    std::cout << "rgid registration ended... " << std::endl;
    // non-rigid initialize
    std::cout << "non-rigid registration to initial..." << std::endl;
    Timer::EventID time3 = time.get_time();
    reg->Initialize();
    Timer::EventID time4 = time.get_time();
    reg->pars_.non_rigid_init_time = time.elapsed_time(time3, time4);
    std::cout << "non-rigid registration... " << std::endl;
    reg->DoNonRigid();
    Timer::EventID time5 = time.get_time();

    std::string fileName1=matchName+"source.txt";
    std::string fileName2=matchName+"target.txt";
    //记录点云对应关系
    std::ofstream file1(fileName1);
    std::ofstream file2(fileName2);
    Mesh source_mesh, target_mesh;
    read_data(src_file, source_mesh);
    read_data(fileName, target_mesh);
    std::cout<<reg->correspondence_pairs_.size()<<std::endl;
    for (auto it = reg->correspondence_pairs_.begin(); it != reg->correspondence_pairs_.end(); it++)
    {
        // file2<<it->position(0)<<" "<<it->position(1)<<" "<<it->position(2)<<std::endl;
        // std::cout<<it->tar_idx<<std::endl;
        // std::cout<<target_mesh.point(target_mesh.vertex_handle(it->tar_idx))[0]<<std::endl;
        file1 << source_mesh.point(source_mesh.vertex_handle(it->src_idx))[0] << " "
              << source_mesh.point(source_mesh.vertex_handle(it->src_idx))[1] << " "
              << source_mesh.point(source_mesh.vertex_handle(it->src_idx))[2] << std::endl;
        Eigen::Matrix4Xf point(4,1);
        point<<target_mesh.point(target_mesh.vertex_handle(it->tar_idx))[0],
               target_mesh.point(target_mesh.vertex_handle(it->tar_idx))[1],
               target_mesh.point(target_mesh.vertex_handle(it->tar_idx))[2],
               1;
        point=transform.inverse()*point;
        // file2 << target_mesh.point(target_mesh.vertex_handle(it->tar_idx))[0]<< " "
        //       << target_mesh.point(target_mesh.vertex_handle(it->tar_idx))[1]<< " "
        //       << target_mesh.point(target_mesh.vertex_handle(it->tar_idx))[2]<< std::endl;
        file2<<point(0,0)<<" "<<point(1,0)<<" "<<point(2,0)<<std::endl;
    }


    std::cout << "Registration done!\nrigid_init time : "
              << time.elapsed_time(time1, time2) << " s \trigid-reg run time = " << time.elapsed_time(time2, time3)
              << " s \nnon-rigid init time = "
              << time.elapsed_time(time3, time4) << " s \tnon-rigid run time = "
              << time.elapsed_time(time4, time5) << " s\n"
              << std::endl;
    write_data(out_file.c_str(), src_mesh, scale);
    std::cout << "write result to " << out_file << std::endl;
    workutils::LocateTrajectoryOnSurface(isUpperArm);

    vtkSmartPointer<vtkOBJReader> reader=vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(out_file.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyData> polydata=vtkSmartPointer<vtkPolyData>::New();
    polydata->DeepCopy(reader->GetOutput());
    vtkSmartPointer<vtkMatrix4x4> mm=vtkSmartPointer<vtkMatrix4x4>::New();
    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            mm->SetElement(i,j,transform(i,j));
        }        
    }    
    vtkSmartPointer<vtkTransform> vtktransform=vtkSmartPointer<vtkTransform>::New();
    mm->Invert();
    vtktransform->SetMatrix(mm);
    vtkSmartPointer<vtkTransformFilter> transformfilter=vtkSmartPointer<vtkTransformFilter>::New();
    transformfilter->SetInputData(polydata);
    transformfilter->SetTransform(vtktransform);
    transformfilter->Update();
    vtkSmartPointer<vtkSTLWriter> exporter=vtkSmartPointer<vtkSTLWriter>::New();
    std::string name=outpath +matchName.c_str()+"res.stl";
    exporter->SetFileName(name.c_str());
    exporter->SetInputData(transformfilter->GetOutput());
    exporter->Update();

    delete reg;
}

//从手腕到大臂的扫描，右手
void generateTrajectory()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr WholeArm(new pcl::PointCloud<pcl::PointXYZ>);
	WholeArm=getCloudFromText("segedArm.txt");

	pcl::PointCloud<pcl::PointXYZ>::Ptr vesselCloud(new pcl::PointCloud<pcl::PointXYZ>);
	vesselCloud=getCloudFromText("result.txt");
        
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(vesselCloud);
	ne.setSearchSurface(WholeArm);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(10);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*normals);

    std::vector<Eigen::Vector3f> vec_y;//y
	std::vector<float> distance;
	for (size_t i = 0; i < vesselCloud->size()-1; i++)
	{
		Eigen::Vector3f v;
		v<<vesselCloud->points[i+1].x-vesselCloud->points[i].x,
		vesselCloud->points[i+1].y-vesselCloud->points[i].y,
		vesselCloud->points[i+1].z-vesselCloud->points[i].z;
		distance.push_back(v.norm());
		v.normalize();
		vec_y.push_back(v);
	}

	std::vector<Eigen::Vector3f> vecz_;//z'
	for (size_t i = 0; i < vesselCloud->size()-1; i++)
	{
		Eigen::Vector3f v;
		v<<normals->at(i).normal_x,normals->at(i).normal_y,normals->at(i).normal_z;
		vecz_.push_back(v);
	}

    std::vector<Eigen::Vector3f> vec_x;//x
	for (size_t i = 0; i < vec_y.size(); i++)
	{
		vec_x.push_back(vec_y[i].cross(vecz_[i]));
	}

	std::vector<Eigen::Vector3f> vec_z;//z
	for (size_t i = 0; i < vec_y.size(); i++)
	{
		vec_z.push_back(vec_x[i].cross(vec_y[i]));
	}

    std::ofstream trajectoryfile("trajectory.txt");
    for (size_t i = 0; i < vec_y.size(); i++)
    {
        trajectoryfile<<vec_x[i][0]<<" "<<vec_x[i][0]<<" "<<vec_x[i][0]<<" "<<vesselCloud->points[0].x;
        trajectoryfile<<vec_x[i][1]<<" "<<vec_x[i][1]<<" "<<vec_x[i][1]<<" "<<vesselCloud->points[0].y;
        trajectoryfile<<vec_x[i][2]<<" "<<vec_x[i][2]<<" "<<vec_x[i][2]<<" "<<vesselCloud->points[0].z;
        trajectoryfile<<0<<" "<<0<<" "<<0<<1<<std::endl;
    }

    //-----------------------visualization------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr TX(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr TY(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr TZ(new pcl::PointCloud<pcl::PointXYZ>);
	int scale=4;
    for (size_t i = 0; i < vec_x.size(); i++)
    {
        pcl::PointXYZ px;
        px.x = vesselCloud->points[i].x + scale*vec_x[i](0);
        px.y = vesselCloud->points[i].y + scale*vec_x[i](1);
        px.z = vesselCloud->points[i].z + scale*vec_x[i](2);
        TX->push_back(px);
        pcl::PointXYZ py;
        py.x = vesselCloud->points[i].x + scale*vec_y[i](0);
        py.y = vesselCloud->points[i].y + scale*vec_y[i](1);
        py.z = vesselCloud->points[i].z + scale*vec_y[i](2);
        TY->push_back(py);
        pcl::PointXYZ pz;
        pz.x = vesselCloud->points[i].x + scale*vec_z[i](0);
        pz.y = vesselCloud->points[i].y + scale*vec_z[i](1);
        pz.z = vesselCloud->points[i].z + scale*vec_z[i](2);
        TZ->push_back(pz);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normal viewer"));
    //设置背景颜色
    viewer->setBackgroundColor(0.3, 0.3, 0.3);
    viewer->addText("Normal", 10, 10, "text");
    //设置点云颜色

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> arm_color(WholeArm, 125, 125, 125);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> vessel_color(vesselCloud, 255, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> x_color(TX, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> y_color(TY, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> z_color(TZ, 0, 0, 255);

    //添加坐标系
    //viewer->addCoordinateSystem(0.1);
    viewer->addPointCloud<pcl::PointXYZ>(WholeArm, arm_color, "arm");
    viewer->addPointCloud<pcl::PointXYZ>(vesselCloud, vessel_color, "vessel");
    viewer->addPointCloud<pcl::PointXYZ>(TX, x_color, "x");
    viewer->addPointCloud<pcl::PointXYZ>(TY, y_color, "y");
    viewer->addPointCloud<pcl::PointXYZ>(TZ, z_color, "z");

    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(vesselCloud, normals, 1, 10, "normals");

    //设置点云大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "arm");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "vessel");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "x");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "y");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "z");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main()
{
    work();
    match("AtlasUpper.obj","segedArm1.ply","Upper",1);
    match("AtlasLower.obj","segedArm2.ply","Lower",0);
    workutils::fitSpline();
    generateTrajectory();
    return 0;
}
