#include "greedy_triangulation_node/greedy_triangulation.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "greedy_triangulation");
    greedy_triangulation::GreedyTriangulation greedy_triangulation;
    greedy_triangulation.run();
    ros::spin();
    return 0;
}