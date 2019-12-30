/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#ifdef _OPENMP
#include <omp.h>
#endif

// Standard
#include <stdlib.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <string>
#include <vector>

// PRT
#include "util.hpp"
#include "types.hpp"
#include "Registrator.hpp"
#include "Visualizer.hpp"
#include "areapick.h"
#include "mirror.hpp"


// GFLAGS
#include <gflags/gflags.h>

DECLARE_bool(help);
DEFINE_bool(h, false, "Show help");
DEFINE_bool(gui, true, "launch GUI after registration");
DEFINE_string(batch_file, "", "path to batch processing file");
DEFINE_string(transformation_matrix_filepath, "[source_filename]_transform.txt", "filepath to output transformation matrix txt");
DEFINE_string(registered_pointcloud_filepath, "[source_filename]_registered.ply", "filepath to output registered point cloud PLY");
DEFINE_string(residual_histogram_image_filepath, "[source_filename]_histogram.png", "filepath to output histogram PNG");
DEFINE_string(fscore_filepath, "[source_filename]_fscore.txt", "filepath to output f-score TXT");

#ifdef _OPENMP
DEFINE_bool(no_parallel, false, "run single-threaded");
#endif

using namespace PRT;

//int main (int argc, char* argv[]) {
int main () { 
    string pic="/home/yons/PointCloudRegistrationTool/data/识别图+模型3.bmp";
    //vector<Vec3f> coords=getcoords(pic);
 
    std::string usage_message = "\nUsage:\n\n1. ./point_cloud_registration_tool [options] <source_point_cloud> <target_point_cloud>\n2. ./point_cloud_registration_tool [options] --batch_file <batch_filepath>\n\nA batch file is a two-column CSV. First column: source point cloud filepaths, second column: target point cloud filepaths.\n\nSee --help for optional arguments\n";
    //gflags::SetUsageMessage(usage_message);
    
    //gflags::ParseCommandLineFlags(&argc, &argv, true);
    
    if (FLAGS_descriptor_radius <= 0) {
        std::cout << "Error: Descriptor radius must be a positive value." << std::endl;
        return -1;
    }
    
    if (FLAGS_subsampling_radius <= 0) {
        std::cout << "Error: Subsampling radius must be a positive value." << std::endl;
        return -1;
    }
    
    if (FLAGS_consensus_inlier_threshold <= 0) {
        std::cout << "Error: Consensus inlier threshold must be a positive value." << std::endl;
        return -1;
    }
    
    if (FLAGS_icp_max_correspondence_distance <= 0) {
        std::cout << "Error: Maximum correspondence distance must be a positive value." << std::endl;
        return -1;
    }
    
    if (FLAGS_residual_threshold <= 0) {
        std::cout << "Error: Residual threshold must be a positive value." << std::endl;
        return -1;
    }
    
    if (FLAGS_h) {
        FLAGS_help = true;
        gflags::HandleCommandLineHelpFlags();
        exit(0);
    }
    /*
    if (argc < 3 && FLAGS_batch_file.empty()) {
        std::cout << usage_message << std::endl;
        return 0;
    }*/
    
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);    
    
    FilepairVectorPtr filepairs;
    if (!FLAGS_batch_file.empty()) {
        
        filepairs = util::readBatchProcessingFile(FLAGS_batch_file);
        
        if(filepairs->empty()) {
            std::cout << "Invalid batch file path, exiting." << std::endl;
            return -1;
        }
        
        FLAGS_gui = false;
        
    } 
    else 
    {
        filepair_t pair;
        pair.sourcefile = pic.substr(0,pic.find_last_of('.'))+".ply";
        pair.targetfile = "/home/yons/PointCloudRegistrationTool/data/coords_rcg/simplify_Segment.stl";
        filepairs = FilepairVectorPtr(new FilepairVector());
        filepairs->push_back(pair);
        
    }

    string targetfilename = "/home/yons/projects/PointCloudRegistrationTool/data/191213.ply";
    string inputfilename="/home/yons/projects/PointCloudRegistrationTool/data/simple_bone.ply";
	
    //string targetfilename = "/home/yons/projects/PointCloudRegistrationTool/data/wzx_skull.ply";
    //string inputfilename="/home/yons/projects/PointCloudRegistrationTool/data/mirror.ply";
    AreaPick targetareapick;
    AreaPick inputareapick;
    Pointspick inputpointspick;                                     
    
    inputpointspick.loadInputcloud(targetfilename);
    inputpointspick.simpleViewer(targetfilename);
    
    inputareapick.loadInputcloud(inputfilename);
    inputareapick.simpleViewer(inputfilename);
    targetareapick.loadInputcloud(targetfilename);
    targetareapick.simpleViewer(targetfilename);

    filepair_t pair;
    pair.sourcefile = inputfilename.substr(0,inputfilename.find_last_of('.')).append("_picked.ply");
    pair.targetfile = targetfilename.substr(0,targetfilename.find_last_of('.')).append("_picked.ply");
    filepairs = FilepairVectorPtr(new FilepairVector());
    filepairs->push_back(pair);
 

    #ifdef _OPENMP
    omp_set_nested(true);
    if (FLAGS_no_parallel)
        omp_set_num_threads(1);
    else
        if (FLAGS_verbose)
            std::cout << "Executing in parallel" << std::endl;
    #endif

    
    for(FilepairVector::size_type i = 0; i < filepairs->size(); i++) 
    {
        std::string source_cloud_filepath = filepairs->at(i).sourcefile;
        #if 0
        PointCloudT::Ptr target_cloud (new PointCloudT);
        target_cloud->clear();
        std::string target_cloud_filepath = filepairs->at(i).targetfile;
        
        PointCloudT::Ptr source_cloud (new PointCloudT);
        source_cloud->clear();
        std::string source_cloud_filepath = filepairs->at(i).sourcefile;
        bool reads_successful = true;
        
        PointCloudT::Ptr ori_source_cloud (new PointCloudT);
        ori_source_cloud->clear();
         PointCloudT::Ptr ori_target_cloud (new PointCloudT);
        ori_target_cloud->clear();

       	if (util::loadPointCloud(source_cloud_filepath, *source_cloud) != 0) {
            std::cout << "Invalid input:" << std::endl << source_cloud_filepath << std::endl;
            std::cout << "Skipping.." << std::endl;
            reads_successful = false;
        }

        if (util::loadPointCloud(target_cloud_filepath, *target_cloud) != 0) {
            std::cout << "Invalid input:" << std::endl << target_cloud_filepath << std::endl;
            std::cout << "Skipping.." << std::endl;
            reads_successful = false;
        }

        if (util::loadPointCloud(inputfilename, *ori_source_cloud) != 0) {
            std::cout << "Invalid input:" << std::endl << inputfilename << std::endl;
            std::cout << "Skipping.." << std::endl;
            reads_successful = false;
        }

        if (util::loadPointCloud(targetfilename, *ori_target_cloud) != 0) {
            std::cout << "Invalid input:" << std::endl << targetfilename << std::endl;
            std::cout << "Skipping.." << std::endl;
            reads_successful = false;
        }
        
        if(!reads_successful)
            continue;
        #endif 


        PointCloudT::Ptr target_cloud (new PointCloudT);
        PointCloudT::Ptr source_cloud (new PointCloudT);
        PointCloudT::Ptr ori_source_cloud (new PointCloudT);
        PointCloudT::Ptr ori_target_cloud (new PointCloudT);
        target_cloud->clear();
        source_cloud->clear();
        ori_target_cloud->clear();
        ori_source_cloud->clear();

        target_cloud=targetareapick.get_picked_area();
        source_cloud=inputareapick.get_picked_area();
        ori_target_cloud=targetareapick.get_input_cloud();
        ori_source_cloud=inputareapick.get_input_cloud();
        //Extract prefix for output files
        std::string prefix = util::removeFileExtension(filepairs->at(i).sourcefile);
        int index=util::find_nexttolast(filepairs->at(i).sourcefile);
        if(index==-1) return -1;
        //Set output file paths
        std::string transformation_matrix_filepath=filepairs->at(i).sourcefile.substr(0,index).append("/res/transform.txt");
        std::string registered_pointcloud_filepath;
        std::string residual_histogram_image_filepath;
        std::string fscore_filepath;
       
        #if 0
        if (FLAGS_transformation_matrix_filepath != "[source_filename]_transform.txt")
            transformation_matrix_filepath = FLAGS_transformation_matrix_filepath;
        else
            transformation_matrix_filepath = prefix + "_transform.txt";
        #endif 
        if (FLAGS_registered_pointcloud_filepath != "[source_filename]_registered.ply")
            registered_pointcloud_filepath = FLAGS_registered_pointcloud_filepath;
        else
            registered_pointcloud_filepath = prefix + "_registered.ply";
        
        if (FLAGS_residual_histogram_image_filepath != "[source_filename]_histogram.png")
            residual_histogram_image_filepath = FLAGS_residual_histogram_image_filepath;
        else
            residual_histogram_image_filepath = prefix + "_residual_histogram.png";
        
        if (FLAGS_fscore_filepath != "[source_filename]_fscore.txt")
            fscore_filepath = FLAGS_fscore_filepath;
        else
            fscore_filepath = prefix + "_fscore.txt";
       
        //Ensure unique output filepaths
        #if 0
        if (util::ensureUniqueFilepath(transformation_matrix_filepath) != 0) {
            std::cout << "Failed to create transformation matrix file." << std::endl;
            continue;
        }
       

        if (util::ensureUniqueFilepath(fscore_filepath) != 0) {
            std::cout << "Failed to create f-score file." << std::endl;
            continue;
        }
        
        if (util::ensureUniqueFilepath(registered_pointcloud_filepath) != 0) {
            std::cout << "Failed to create registered PLY file." << std::endl;
            continue;
        }
        
        if (util::ensureUniqueFilepath(residual_histogram_image_filepath) != 0) {
            std::cout << "Failed to create residual histogram file." << std::endl;
            continue;
        }
        #endif
        //Registration
        //Setup
        Mirror mir;
        Registrator::Ptr registrator (new Registrator());
        registrator->setNumKSearchNeighbors(FLAGS_num_ksearch_neighbors);
        registrator->setDescriptorRadius(FLAGS_descriptor_radius);
        registrator->setSubsamplingRadius(FLAGS_subsampling_radius);
        registrator->setConsensusInlierThreshold(FLAGS_consensus_inlier_threshold);
        registrator->setConsensusMaxIterations(FLAGS_consensus_max_iterations);
        registrator->setICPMaxIterations(FLAGS_icp_max_iterations);
        registrator->setICPMaxCorrespondenceDistance(FLAGS_icp_max_correspondence_distance);
        registrator->setResidualThreshold(FLAGS_residual_threshold);
        registrator->setTargetCloud(target_cloud);
        registrator->setSourceCloud(source_cloud);
        registrator->setOriginSourceCloud(ori_source_cloud);
        registrator->setOriginTargetCloud(ori_target_cloud);
        
        //Compute
        registrator->performRegistration(FLAGS_registration_technique);
        
        //Save Results
        registrator->saveResidualColormapPointCloud(registered_pointcloud_filepath);  //save residual color pointcloud
        if(registrator->saveFinalTransform(transformation_matrix_filepath)!=0)
            cout<<"Transform matrix saved error"<<endl;
        else
        {
            cout<<"saved transformation txt!"<<endl;
        }
    
        registrator->saveFScoreAtThreshold(fscore_filepath, FLAGS_residual_threshold);  //save Fscore
        
        std::cout << "Registration of " << source_cloud_filepath << " finished" << std::endl;
        //std::cout << "F-score: " << registrator->getFScoreAtThreshold() << std::endl;
        //std::cout << "Saved to: " << registered_pointcloud_filepath << std::endl;
        
        if (!FLAGS_gui)
            continue;


        #if 0  //add get symmetry panel coords
        string coordsfile="/home/yons/projects/PointCloudRegistrationTool/res/coords.txt";
        struct Coord clicked_p[3];
        struct Coord mirrored_p[3];
        Eigen::Matrix4f final_Matrix=registrator->getCombinedTransformation();
        ofstream out_clicked("/home/yons/projects/PointCloudRegistrationTool/res/mirror_coords.txt",ios::binary | ios::out);
        ifstream in_clicked(coordsfile);
        if(!in_clicked) cout<<"Can't open "<<coordsfile<<endl;
        else
        {
            for(int i=0;i<3;i++)
            {
                in_clicked>>clicked_p[i].x>>clicked_p[i].y>>clicked_p[i].z;
                mirrored_p[i].x=final_Matrix(0,0)*clicked_p[i].x+final_Matrix(0,1)*clicked_p[i].y+final_Matrix(0,2)*clicked_p[i].z+final_Matrix(0,3);
                mirrored_p[i].y=final_Matrix(1,0)*clicked_p[i].x+final_Matrix(1,1)*clicked_p[i].y+final_Matrix(1,2)*clicked_p[i].z+final_Matrix(1,3);
                mirrored_p[i].z=final_Matrix(2,0)*clicked_p[i].x+final_Matrix(2,1)*clicked_p[i].y+final_Matrix(2,2)*clicked_p[i].z+final_Matrix(2,3);
                cout<<clicked_p[i].x<<' '<<clicked_p[i].y<<' '<<clicked_p[i].z<<endl;
                cout<<mirrored_p[i].x<<' '<<mirrored_p[i].y<<' '<<mirrored_p[i].z<<endl;
                out_clicked<<(float)(clicked_p[i].x+mirrored_p[i].x)/2<<(float)(clicked_p[i].y+mirrored_p[i].y)/2<<(float)(clicked_p[i].z+mirrored_p[i].z)/2<<endl;
                cout<<endl;
            }
            out_clicked.close();
            in_clicked.close();
        }
        #endif
        
        //Visualization
        Visualizer visualizer ("Point Cloud Registration Tool");
        visualizer.setRegistrator(registrator);
        //visualizer.saveHistogramImage(residual_histogram_image_filepath);
        visualizer.visualize();

        inputpointspick.loadInputcloud(registered_pointcloud_filepath);
        inputpointspick.simpleViewer(registered_pointcloud_filepath);
    }
    return 0;
}
