#include "marching_cubes_node/marching_cubes.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "marching_cubes");
    marching_cubes::MarchingCubes marching_cubes;
    marching_cubes.run();
    ros::spin();
    return 0;
}