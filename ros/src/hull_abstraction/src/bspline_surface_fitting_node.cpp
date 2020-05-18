#include "bspline_surface_fitting_node/bspline_surface_fitting.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bspline_surface_fitting"); // Initilize the node
    bspline_surface_fitting_node::BsplineSurfaceFitting bspline_surface_fitting; 
    bspline_surface_fitting.run();
    ros::spin();
    return 0;
}
