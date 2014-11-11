#include "system.h"
#include "methods.h"

#include <tclap/CmdLine.h>

#include <vector>
#include <functional>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

#include "CloudModel.h"
#include "PCLMotionFilter.h"
#include "generate_cylinder_on_table.h"
#include "pcl_smoothing.h"
#include "depth_filter.h"
#include "object.h"

using TCLAP::ValueArg;
using TCLAP::SwitchArg;
using std::string;
using std::vector;
using std::cout;
using std::endl;

// uniform random double in [-1:1]
double rand11() {
    return (2*(double)rand()/RAND_MAX - 1);
}

// uniform random double in [0:1]
double rand01() {
    return (double)rand()/RAND_MAX;
}

// the command line arguments
ValueArg<string> input_arg(                 "i", "input"         , "the source of point clouds"                                ,  true, ""      , "string");
SwitchArg hide_input_switch(                "I", "hide_input"    , "don't show the input cloud"                                ,        false             );
SwitchArg hide_output_switch(               "O", "hide_output"   , "don't show the output cloud"                               ,        false             );
ValueArg<string> file_arg(                  "f", "file"          , "file to read input from (only for input method 'file')"    , false, ""      , "string");
ValueArg<string> method_arg(                "m", "method"        , "method to use for processing point clounds"                ,  true, ""      , "string");
SwitchArg motion_filter_switch(              "", "motion_filter" , "use motion filter"                                         ,        false             );
ValueArg<int> smoothing_arg(                 "", "smoothing"     , "use smoothing with box with <int> (uneven)"                , false, 0       , "int"   );
ValueArg<double> depth_arg(                  "", "depth"         , "use depth cut-off at depth <double>"                       , false, -1      , "double");
ValueArg<double> voxel_arg(                  "", "voxel"         , "down-sample at box size <double>"                          , false, -1      , "double");
ValueArg<int> cloud_model_size_arg(          "", "cloud_size"    , "size of cloud model"                                       , false, 10000   , "int");
ValueArg<double> cloud_model_dying_prob(     "", "cloud_die"     , "dying probability within cloud model"                      , false, 0.001   , "double");
ValueArg<double> cloud_model_sol_dying_prob( "", "cloud_sol_die" , "solitude dying probability within cloud model"             , false, 0.01    , "double");
ValueArg<int> cloud_model_persistence_arg(   "", "cloud_pers"    , "persistence within cloud model"                            , false, 100     , "double");
ValueArg<double> cloud_model_smoothing_arg(  "", "cloud_smooth"  , "smoothing of cloud model"                                  , false, 0.1     , "double");

vector<string> input_vector = { "file", "cyl_on_table", "kinect"};
vector<string> method_vector = { "none", "icp", "cloud_model"};

// check if argument value is within given vector and print messessage
template < typename T>
bool check_args(ValueArg<T> & argument, vector<T> & argument_value_vector) {
    bool argument_ok = false;
    string argument_value = argument.getValue();
    for(string argument_elem : argument_value_vector) {
        if(argument_elem==argument_value) {
            argument_ok = true;
            break;
        }
    }
    if(!argument_ok) {
        cout << "Argument '" << argument.getName() << "' has to be one of" << endl;
        for(string argument_elem : argument_value_vector) {
            cout << "    " << argument_elem << endl;
        }
    }
    return argument_ok;
}

//--------//
//  main  //
//--------//
int main(int argn, char ** args) {

    // random seed
    //srand48(time(nullptr));
    srand(time(nullptr));

    //------------------------//
    // read command line args //
    //------------------------//

    try {
	TCLAP::CmdLine cmd("A simple viewer...", ' ', "");

        cmd.add(cloud_model_size_arg);
        cmd.add(cloud_model_dying_prob);
        cmd.add(cloud_model_sol_dying_prob);
        cmd.add(cloud_model_persistence_arg);
        cmd.add(cloud_model_smoothing_arg);
        cmd.add(voxel_arg);
        cmd.add(depth_arg);
        cmd.add(motion_filter_switch);
        cmd.add(smoothing_arg);
        cmd.add(hide_output_switch);
        cmd.add(hide_input_switch);
        cmd.add(file_arg);
        cmd.add(method_arg);
        cmd.add(input_arg);

	// Parse the args array (throws execption in case of failure)
	cmd.parse(argn, args);

    } catch (TCLAP::ArgException &e) {
        // catch any exceptions
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }

    // check if args are ok
    if(!check_args<string>(input_arg,input_vector) ||
       !check_args<string>(method_arg,method_vector)) {
        return 0;
    }

    //-----------------//
    //  set up viewer  //
    //-----------------//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    //viewer->setCameraPosition(100,100,100,-100,-100,100);
    viewer->setBackgroundColor(0.3, 0.3, 0.3);

    //------------------------//
    //  get input point cloud //
    //------------------------//
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>input_cloud_handler(input_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(input_cloud, input_cloud_handler, "input cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "input cloud");
    std::function<void()> get_input_cloud = [&](){
        cout << "Error: no input method defined" << endl;
        input_cloud->clear();
    };
    PCL_ModuleSystem S; // for kinect
    if(input_arg.getValue()=="kinect") {
        // start
        engine().open(S);
        // define get method
        get_input_cloud = [&](){
            if(engine().shutdown.getValue()>0) {
                cout << "Error: no input method defined" << endl;
                input_cloud->clear();
            } else {
                S.kinect_points.var->waitForNextRevision();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_cloud = S.pcl_cloud.get();
                if(kinect_cloud) {
                    pcl::copyPointCloud(*kinect_cloud,*input_cloud);
                }
            }
        };
    } else if(input_arg.getValue()=="cyl_on_table") {
        get_input_cloud = [&](){
            input_cloud = generate_cylinder_on_table::get_point_cloud();
        };
    } else if(input_arg.getValue()=="file") {
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_arg.getValue(), *input_cloud) == -1) {
            std::cout << "Could not read file '" << file_arg.getValue() << "'" << std::endl;
            return(-1);
        }
        get_input_cloud = [&](){
            if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_arg.getValue(), *input_cloud) == -1) {
                std::cout << "Could not read file '" << file_arg.getValue() << "'" << std::endl;
            }
        };
    } else {
        cout << "input '" << input_arg.getValue() << "' not implemented" << endl;
        return(-1);
    }

    //----------------//
    //  apply method  //
    //----------------//
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>output_cloud_handler(output_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(output_cloud, output_cloud_handler, "output cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "output cloud");
    std::function<void()> get_output_cloud = [&](){
        cout << "Error: no method defined" << endl;
        output_cloud = pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    };
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; // ICP object
    CloudModel cloud_model;                                             // CloudModel object
    if(method_arg.getValue()=="none") {
        get_output_cloud = [&](){
            if(input_cloud) {
                output_cloud = input_cloud;
            }
        };
    } else if(method_arg.getValue()=="icp") {
        // generate misaligned cylinder model
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder = generate_cylinder_on_table::get_cylinder_model(); // for "icp" method
        double theta = 2*rand01()*M_PI;
        double transl_scale = 10;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() = Eigen::Vector3f(transl_scale*rand11(), transl_scale*rand11(), transl_scale*rand11());
        transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f(rand01(), rand01(), rand01())));
        cout << "Initial Transform:" << endl << transform.matrix() << endl;
        pcl::transformPointCloud(*cylinder, *cylinder, transform);
        output_cloud = cylinder;
        // define getter method
        get_output_cloud = [&](){
            if(!input_cloud) {
                cout << "Error: input cloud not defined" << endl;
                return;
            }
            if(!output_cloud) {
                cout << "Error: input cloud not defined" << endl;
                output_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                return;
            }
            icp = pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>();
            // set thresholds
            cout << "old thresholds (corr.dist./ransac) = (" <<
            icp.getMaxCorrespondenceDistance() << "/" <<
            icp.getRANSACOutlierRejectionThreshold() << ")" << endl;
            icp.setRANSACOutlierRejectionThreshold(10);
            icp.setMaxCorrespondenceDistance(10);
            cout << "new thresholds (corr.dist./ransac) = (" <<
            icp.getMaxCorrespondenceDistance() << "/" <<
            icp.getRANSACOutlierRejectionThreshold() << ")" << endl;
            // set max iterations
            cout << "old iterations (corr.dist./ransac) = (" <<
            icp.getMaximumIterations() << "/" <<
            icp.getRANSACIterations() << ")" << endl;
            icp.setMaximumIterations(1);
            icp.setRANSACIterations(0);
            cout << "new iterations (corr.dist./ransac) = (" <<
            icp.getMaximumIterations() << "/" <<
            icp.getRANSACIterations() << ")" << endl;
            // set input and target
            icp.setInputCloud(output_cloud);
            icp.setInputTarget(input_cloud);
            // perform alignment
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
            icp.align(*out);
            output_cloud = out;
            // print some info
            cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << endl;
            cout << "Final Transform:" << endl << icp.getFinalTransformation() << endl;
        };
    } else if(method_arg.getValue()=="cloud_model") {
        cloud_model.setModelSize(cloud_model_size_arg.getValue());
        cloud_model.setDyingProb(cloud_model_dying_prob.getValue());
        cloud_model.setSolDyingProb(cloud_model_sol_dying_prob.getValue());
        cloud_model.setPersistence(cloud_model_persistence_arg.getValue());
        cloud_model.setSmoothing(cloud_model_smoothing_arg.getValue());
        output_cloud = cloud_model.getModelCloud();
        get_output_cloud = [&](){
            cloud_model.update_model(input_cloud);
            //cloud_model.getModelCloud(output_cloud);
        };
    } else {
        cout << "method '" << method_arg.getValue() << "' not implemented" << endl;
        return(-1);
    }

    //----------------//
    //  display loop  //
    //----------------//
    bool hide_input = hide_input_switch.getValue();
    bool hide_output = hide_output_switch.getValue();
    PCLMotionFilter motion_filter;
    if(hide_input) {
        viewer->removePointCloud("input cloud");
    }
    if(hide_output) {
        viewer->removePointCloud("output cloud");
    }
    while(!viewer->wasStopped()) {
        // input
        get_input_cloud();
        if(input_cloud && !hide_input){
            viewer->updatePointCloud(input_cloud, "input cloud");
        }
        // filters
        if(input_cloud) {
            if(depth_arg.getValue()>0) {
                depth_filter(depth_arg.getValue(),input_cloud);
            }
            if(smoothing_arg.getValue()>0) {
                box_smoothing(smoothing_arg.getValue(),input_cloud);
                box_smoothing(smoothing_arg.getValue(),input_cloud);
                box_smoothing(smoothing_arg.getValue(),input_cloud);
            }
            if(voxel_arg.getValue()>0) {
                voxelFilter(input_cloud,input_cloud,voxel_arg.getValue());
            } 
            if(motion_filter_switch.getValue()) {
                motion_filter.new_input(input_cloud);
                motion_filter.get_cloud(input_cloud);
            }
        }
        // output/methods
        get_output_cloud();
        if(output_cloud && !hide_output){
            viewer->updatePointCloud(output_cloud, "output cloud");
        }
        if(input_cloud && output_cloud) {
            viewer->spinOnce(100);
        }
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }

    //------------//
    //  clean up  //
    //------------//
    if(input_arg.getValue()=="kinect") {
        engine().close(S);
    }

    return 0;
}
