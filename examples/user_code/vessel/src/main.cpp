#include "tools/io_mesh.h"
#include "tools/OmpHelper.h"
#include "NonRigidreg.h"


int main(int argc, char **argv)
{
    // Mesh src_mesh;
    // Mesh tar_mesh;
    // std::string src_file;
    // std::string tar_file;
    // std::string out_file, outpath;
    // std::string landmark_file;
    // RegParas paras;


    // if(argc==3)
    // {
    //     src_file = argv[1];
    //     tar_file = argv[2];
    //     // outpath = argv[3];
    //     outpath ="/home/y/MyWorkSpace/Fast_RNRR-master/build/";
    // }
    // else if(argc==5)
    // {
    //     src_file = argv[1];
    //     tar_file = argv[2];
    //     outpath = argv[3];
    //     landmark_file = argv[4];
    //     paras.use_landmark = true;
    // }
    // else
    // {
    //     std::cout << "Usage: <srcFile> <tarFile> <outPath>\n    or <srcFile> <tarFile> <outPath> <landmarkFile>" << std::endl;
    //     exit(0);
    // }

    // // Setting paras
    // paras.alpha = 50.0;
    // paras.beta =  50.0;
    // paras.gamma = 1e8;
    // paras.uni_sample_radio = 50.0;//重要参数

    // paras.use_distance_reject = true;
    // // paras.distance_threshold = 0.05;
    // paras.distance_threshold = 500;
    // paras.use_normal_reject = false;
    // paras.normal_threshold = M_PI/3;
    // paras.use_Dynamic_nu = true;
    // paras.rigid_iters=100;

    // paras.out_gt_file = outpath + "_res.txt";
    // out_file = outpath + "res.obj";

 
    // read_data(src_file, src_mesh);//读取source mesh数据
    // read_data(tar_file, tar_mesh);//读取target mesh数据
    // if(src_mesh.n_vertices()==0 || tar_mesh.n_vertices()==0)
    //     exit(0);

    // if(src_mesh.n_vertices() != tar_mesh.n_vertices())
    //     paras.calc_gt_err = false;

    // if(paras.use_landmark)
    //     read_landmark(landmark_file.c_str(), paras.landmark_src, paras.landmark_tar);
    // double scale = mesh_scaling(src_mesh, tar_mesh);
    
    // NonRigidreg* reg;
    // reg = new NonRigidreg;

    // reg->name1=src_file.c_str();
    // reg->name2=tar_file.c_str();

    // Timer time;
    // std::cout << "\nrigid registration to initial..." << std::endl;
    // Timer::EventID time1 = time.get_time();
    // reg->rigid_init(src_mesh, tar_mesh, paras);
    // reg->DoRigid();
    // Timer::EventID time2 = time.get_time();
    // std::cout << "rgid registration ended... " << std::endl;
    // // non-rigid initialize
    // std::cout << "non-rigid registration to initial..." << std::endl;
    // Timer::EventID time3 = time.get_time();
    // reg->Initialize();
    // Timer::EventID time4 = time.get_time();
    // reg->pars_.non_rigid_init_time = time.elapsed_time(time3, time4);
    // std::cout << "non-rigid registration... " << std::endl;
    // reg->DoNonRigid();
    // Timer::EventID time5 = time.get_time();

    // //记录点云对应关系
    // std::ofstream file1("pcdsource.txt");
    // std::ofstream file2("pcdtarget.txt");
    // Mesh source_mesh,target_mesh;
    // read_data(src_file, source_mesh);
    // read_data(tar_file, target_mesh);
    // Mesh* source_mesh_;
    // Mesh* target_mesh_;
    // source_mesh_=&source_mesh;
    // target_mesh_=&target_mesh;
    // for(auto it=reg->correspondence_pairs_.begin();it!=reg->correspondence_pairs_.end();it++)
    // {
    //     // file2<<it->position(0)<<" "<<it->position(1)<<" "<<it->position(2)<<std::endl;
    //     file2<<target_mesh_->point(target_mesh_->vertex_handle(it->tar_idx))[0]+0.1<<" "
    //     <<target_mesh_->point(target_mesh_->vertex_handle(it->tar_idx))[1]+0.1<<" "
    //     <<target_mesh_->point(target_mesh_->vertex_handle(it->tar_idx))[2]+0.1<<std::endl;
    //     file1<<source_mesh_->point(source_mesh_->vertex_handle(it->src_idx))[0]<<" "
    //     <<source_mesh_->point(source_mesh_->vertex_handle(it->src_idx))[1]<<" "
    //     <<source_mesh_->point(source_mesh_->vertex_handle(it->src_idx))[2]<<std::endl;
    // }

    // std::cout << "Registration done!\nrigid_init time : "
    //           << time.elapsed_time(time1, time2) << " s \trigid-reg run time = " << time.elapsed_time(time2, time3)
    //           << " s \nnon-rigid init time = "
    //           << time.elapsed_time(time3, time4) << " s \tnon-rigid run time = "
    //           << time.elapsed_time(time4, time5) << " s\n" << std::endl;
    // write_data(out_file.c_str(), src_mesh, scale);
    // std::cout<< "write result to " << out_file << std::endl;
    // delete reg;

    return 0;
}
