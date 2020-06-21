#include "random_sampling_node/random_sampling.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "random_sampling");
    random_sampling::RandomSampling random_sampling;
    random_sampling.run();
    ros::spin();
    return 0;
}