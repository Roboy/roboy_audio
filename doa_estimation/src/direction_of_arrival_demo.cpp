/*
 * Copyright 2016 <Admobilize>
 * All rights reserved.
 */
#include <gflags/gflags.h>
#include <fftw3.h>
#include <stdint.h>
#include <string.h>

#include <wiringPi.h>

#include <fstream>
#include <iostream>
#include <string>
#include <valarray>

#include "direction_of_arrival.h"
#include "everloop.h"
#include "everloop_image.h"
#include "microphone_array.h"
#include "wishbone_bus.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "doa_estimation/DirVec.h"

DEFINE_bool(big_menu, true, "Include 'advanced' options in the menu listing");
DEFINE_int32(sampling_frequency, 16000, "Sampling Frequency");

namespace hal = matrix_hal;

int led_offset[] = {23, 27, 32, 1, 6, 10, 14, 19};
int lut[] = {1, 2, 10, 200, 10, 2, 1};

int main(int argc, char *argv[]) {

    // start with init
    ros::init(argc, argv, "talker");
    // create the NodeHandle
    ros::NodeHandle n;
    // create the publisher
    ros::Publisher chatter_pub = n.advertise<doa_estimation::DirVec>("/chatter", 1000);

    ros::Rate loop_rate(10);
    google::ParseCommandLineFlags(&argc, &argv, true);

    hal::WishboneBus bus;
    bus.SpiInit();

    hal::MicrophoneArray mics;
    mics.Setup(&bus);

    hal::Everloop everloop;
    everloop.Setup(&bus);

    hal::EverloopImage image1d;

    int sampling_rate = FLAGS_sampling_frequency;
    mics.SetSamplingRate(sampling_rate);
    mics.ShowConfiguration();

    hal::DirectionOfArrival doa(mics);
    doa.Init();

    float azimutal_angle;
    float polar_angle;
    int mic;

    while (ros::ok()) {
        mics.Read(); /* Reading 8-mics buffer from de FPGA */

        doa.Calculate();

        azimutal_angle = doa.GetAzimutalAngle() * 180 / M_PI;
        polar_angle = doa.GetPolarAngle() * 180 / M_PI;
        mic = doa.GetNearestMicrophone();

        /*std::cout << "azimutal angle = " << azimutal_angle
                  << ", polar angle = " << polar_angle << ", mic = " << mic
                  << std::endl;
        */
        doa_estimation::DirVec msg;

        std::stringstream ss;

        /*ss << "azimutal angle = " << azimutal_angle
           << ", polar angle = " << polar_angle << ", mic = " << mic
           << std::endl;*/
        msg.azimutal_angle = azimutal_angle;
        msg.polar_angle = polar_angle;


        //ROS_INFO("%s", msg.data.c_str());

        // publish the data through the publisher
        chatter_pub.publish(msg);

        for (hal::LedValue& led : image1d.leds) {
            led.blue = 0;
        }


        for (int i = led_offset[mic] - 3, j = 0; i < led_offset[mic] + 3;
             ++i, ++j) {
            if (i < 0) {
                image1d.leds[image1d.leds.size() + i].blue = lut[j];
            } else {
                image1d.leds[i % image1d.leds.size()].blue = lut[j];
            }

            everloop.Write(&image1d);
        }

        // the typical end of a ros while loop
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
