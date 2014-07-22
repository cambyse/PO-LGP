#include <tclap/CmdLine.h>

#include <vector>

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

// process keyboard events
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if(event.getKeySym() == "r" && event.keyDown()) {
        cout << "r was pressed" << endl;
    }
}

// process mouse events
void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease) {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
    }
}

// the command line arguments
ValueArg<string> input_arg(  "i", "input" , "the source of point clouds"                                , false, "default" , "string");
ValueArg<string> file_arg(   "f", "file"  , "file to read input from (only for input method 'file')"    , false, ""        , "string");
ValueArg<string> method_arg( "m", "method", "method to use for processing point clounds"                , false, "none"    , "string");
vector<string> input_vector = { "default", "file", "cyl_on_table"};
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
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setBackgroundColor (0.3, 0.3, 0.3);
    // callbacks
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);

    //------------------------//
    //  get input point cloud //
    //------------------------//
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    {
        if(input_arg.getValue()=="default") {
            uint8_t r(255), g(15), b(15);
            for (float z(-1.0); z <= 1.0; z += 0.05) {
                for (float angle(0.0); angle <= 360.0; angle += 5.0) {
                    pcl::PointXYZ basic_point;
                    basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
                    basic_point.y = sinf (pcl::deg2rad(angle));
                    basic_point.z = z;

                    pcl::PointXYZRGB point;
                    point.x = basic_point.x;
                    point.y = basic_point.y;
                    point.z = basic_point.z;
                    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                    point.rgb = *reinterpret_cast<float*>(&rgb);
                    input_cloud->points.push_back(point);
                }
                if(z < 0.0) {
                    r -= 12;
                    g += 12;
                } else {
                    g -= 12;
                    b += 12;
                }
            }
            input_cloud->width = (int)input_cloud->points.size();
            input_cloud->height = 1;
        } else if(input_arg.getValue()=="cyl_on_table") {
            input_cloud = generate_cylinder_on_table::get_point_cloud();
        } else if(input_arg.getValue()=="file") {
            if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_arg.getValue(), *input_cloud) == -1) {
                std::cout << "Could not read file '" << file_arg.getValue() << "'" << std::endl;
                return(-1);
            }
        } else {
            cout << "input '" << input_arg.getValue() << "' not implemented" << endl;
            return(-1);
        }
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb_input(input_cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(input_cloud, rgb_input, "input cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "input cloud");
    }

    //----------------//
    //  apply method  //
    //----------------//
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    {
        if(method_arg.getValue()=="none") {
            output_cloud = input_cloud;
        } else if(method_arg.getValue()=="test") {
            TestMethod::process(input_cloud,output_cloud);
        } else if(method_arg.getValue()=="icp") {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder = generate_cylinder_on_table::get_cylinder_model(); // for "icp" method
            // random-transform cylinder
            double theta = 2*rand01()*M_PI;
            double transl_scale = 10;
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.translation() = Eigen::Vector3f(transl_scale*rand11(), transl_scale*rand11(), transl_scale*rand11());
            transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f(rand01(), rand01(), rand01())));
            cout << "Initial Transform:" << endl << transform.matrix() << endl;
            pcl::transformPointCloud(*cylinder, *cylinder, transform);
            pcl::copyPointCloud(*cylinder, *output_cloud);
        } else {
            cout << "method '" << method_arg.getValue() << "' not implemented" << endl;
            return(-1);
        }
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_output(output_cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(output_cloud, rgb_output, "output cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "output cloud");
    }

    //----------------//
    //  display loop  //
    //----------------//
    if(method_arg.getValue()=="icp") {
        //--------------//
        //  set up ICP  //
        //--------------//
        // create icp object
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
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

        //---------------//
        //  viewer loop  //
        //---------------//
        while(!viewer->wasStopped()) {
            //-----------//
            //  run ICP  //
            //-----------//
            // set input and target
            icp.setInputCloud(output_cloud);
            icp.setInputTarget(input_cloud);
            // perform alignment
            icp.align(*output_cloud);
            // print some info
            cout << "has converged:" << icp.hasConverged() << " score: " <<
                icp.getFitnessScore() << endl;
            cout << "Final Transform:" << endl << icp.getFinalTransformation() << endl;

            //---------------//
            // update viewer //
            //---------------//
            viewer->updatePointCloud(input_cloud, "input cloud");
            viewer->updatePointCloud(output_cloud, "output cloud");
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    } else {
        while(!viewer->wasStopped()) {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

    return 0;
}
