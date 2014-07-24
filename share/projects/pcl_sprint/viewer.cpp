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

#include "test_method.h"
#include "generate_cylinder_on_table.h"

using TCLAP::ValueArg;
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
ValueArg<string> input_arg(  "i", "input" , "the source of point clouds"                                ,  true, "" , "string");
ValueArg<string> file_arg(   "f", "file"  , "file to read input from (only for input method 'file')"    , false, "" , "string");
ValueArg<string> method_arg( "m", "method", "method to use for processing point clounds"                ,  true, "" , "string");
vector<string> input_vector = { "file", "cyl_on_table", "kinect"};
vector<string> method_vector = { "none", "test", "icp" };

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

        cmd.add(input_arg);
        cmd.add(file_arg);
        cmd.add(method_arg);

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
                input_cloud = S.pcl_cloud.get();
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>output_cloud_handler(output_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(output_cloud, output_cloud_handler, "output cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "output cloud");
    std::function<void()> get_output_cloud = [&](){
        cout << "Error: no method defined" << endl;
        output_cloud->clear();
    };
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; // ICP object
    if(method_arg.getValue()=="none") {
        get_output_cloud = [&](){
            output_cloud = input_cloud;
        };
    } else if(method_arg.getValue()=="test") {
        get_output_cloud = [&](){
            TestMethod::process(input_cloud,output_cloud);
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
        pcl::copyPointCloud(*cylinder, *output_cloud);
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
            icp.align(*output_cloud);
            // print some info
            cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << endl;
            cout << "Final Transform:" << endl << icp.getFinalTransformation() << endl;
        };
    } else {
        cout << "method '" << method_arg.getValue() << "' not implemented" << endl;
        return(-1);
    }

    //----------------//
    //  display loop  //
    //----------------//
    while(!viewer->wasStopped()) {
        get_input_cloud();
        if(input_cloud){
            viewer->updatePointCloud(input_cloud, "input cloud");
        }
        get_output_cloud();
        if(output_cloud){
            viewer->updatePointCloud(output_cloud, "output cloud");
        }
        if(input_cloud && output_cloud) {
            viewer->spinOnce(100);
        }
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    //------------//
    //  clean up  //
    //------------//
    if(input_arg.getValue()=="kinect") {
        engine().close(S);
    }

    return 0;
}
