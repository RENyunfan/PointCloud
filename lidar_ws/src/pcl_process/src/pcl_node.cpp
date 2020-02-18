#include "pcl_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_process");

    ros::NodeHandle nh;

    PclTestCore core(nh);
    return 0;
}
