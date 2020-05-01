#include "load_pcd_node/load_pcd.h"

int main (int argc, char **argv)
{
    load_pcd::LoadPCD load_pcd;
    load_pcd.run(argc, argv);
    return 1;
}
