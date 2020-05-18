#include "display_mesh_node/display_mesh.h"

int main (int argc, char **argv)
{
    ros::init (argc, argv, "display_mesh");
    display_mesh::DisplayMesh display_mesh;
    display_mesh.run();
    ros::spin();
    return 0;
}