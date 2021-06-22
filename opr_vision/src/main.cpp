#include <ros/ros.h>
#include "vision.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "opr_vision");
    ros::NodeHandle nh;
    CitBrains::Vision vision(1);

    ros::Rate rate(10);

    while(ros::ok())
    {
        int w, h, bpp;
        vision.getImageInfo(w, h, bpp);
        char *buf = new char[w * h * bpp];
        vision.getImageData(buf);
        vision.getColorResultData(buf);

        ros::spinOnce();
        rate.sleep();
    }
}
