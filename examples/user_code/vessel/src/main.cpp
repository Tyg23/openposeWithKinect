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

int main()
{
    work();
    match("AtlasUpper.obj","segedArm1.ply","Upper",1);
    match("AtlasLower.obj","segedArm2.ply","Lower",0);
    workutils::fitSpline();
    return 0;
}
