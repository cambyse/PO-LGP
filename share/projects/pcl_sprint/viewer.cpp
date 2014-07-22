#include <tclap/CmdLine.h>

#include <vector>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "test_method.h"
#include "generate_cylinder_on_table.h"
#include "icp.h"

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if(event.getKeySym() == "r" && event.keyDown()) {
        cout << "r was pressed" << endl;
    }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease) {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
    }
}

//------------------------------------------------------------------------------------------//

using TCLAP::ValueArg;
using std::string;
using std::vector;
using std::cout;
using std::endl;

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

    //------------------//
    //  get point cloud //
    //------------------//
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
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


    //----------------//
    //  apply method  //
    //----------------//
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(method_arg.getValue()=="none") {
        output_cloud = input_cloud;
    } else if(method_arg.getValue()=="test") {
        TestMethod::process(input_cloud,output_cloud);
    } else if(method_arg.getValue()=="icp") {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder = generate_cylinder_on_table::get_cylinder_model();

        double theta = 2*drand48()*M_PI*0.3;
        double transl_scale = 5;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() = Eigen::Vector3f(transl_scale*2*drand48()-1, transl_scale*2*drand48()-1, transl_scale*2*drand48()-1);
        transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f(drand48(), drand48(), drand48())));
        pcl::transformPointCloud(*cylinder, *cylinder, transform);
        ICP::apply(cylinder, input_cloud, output_cloud);
    } else {
        cout << "method '" << method_arg.getValue() << "' not implemented" << endl;
        return(-1);
    }

    //-----------//
    //  display  //
    //-----------//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // add point clouds
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_input(input_cloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_output(output_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (input_cloud, rgb_input, "input cloud");
    viewer->addPointCloud<pcl::PointXYZRGB> (output_cloud, rgb_output, "output cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "output cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setBackgroundColor (0.3, 0.3, 0.3);
    // callbacks
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);
    // main loop
    while(!viewer->wasStopped()) {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
