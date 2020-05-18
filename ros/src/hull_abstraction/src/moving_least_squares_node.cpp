#include "moving_least_squares_node/moving_least_squares.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moving_least_squares");
    moving_least_squares_node::MovingLeastSquares moving_least_squares;
    moving_least_squares.run();
    ros::spin();
    return 0;
}

