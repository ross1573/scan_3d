#include "scan_3d.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_3d");
    Scan3D scan3d;
    ros::spin();
    return 0;
}
