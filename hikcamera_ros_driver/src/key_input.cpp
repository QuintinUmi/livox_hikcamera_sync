#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>

#include "kbhit.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "key_input");
    ros::NodeHandle rosHandle;

    ros::Publisher key_ascii_pub = rosHandle.advertise<std_msgs::Int8>("/hikcamera/key_input", 1);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        if(kbhit()){
            // printf("\n----------------------------------------------------------------------------------\n");
            char c = fgetc(stdin);
            std_msgs::Int8 ascii;
	    std_msgs::Int8 ascii0;
            ascii.data = int(c);
	    ascii0.data = int(0);
            key_ascii_pub.publish(ascii);
	    // key_ascii_pub.publish(ascii0);
	}
        loop_rate.sleep();
    }

    return 0;
}
