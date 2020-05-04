#include "poisson_reconstruction_node/poisson_reconstruction.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poisson_reconstruction");
    poisson_reconstruction::PoissonReconstruction poisson_reconstruction;
    poisson_reconstruction.run();
    ros::spin();
    return 1;
}