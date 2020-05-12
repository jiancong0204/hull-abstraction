#include "load_pcd_node/load_pcd.h"

int main (int argc, char **argv)
{
    ros::init (argc, argv, "load_pcd"); // Node name
    load_pcd_node::LoadPCD load_pcd;
    load_pcd.run();
    return 1;
}
