/*
 * Copyright 2016 <Admobilize>
 * All rights reserved.
 */
#include <gflags/gflags.h>

#include <wiringPi.h>
#include <valarray>

#include "direction_of_arrival.h"
#include "everloop.h"
#include "everloop_image.h"
#include "microphone_array.h"
#include "wishbone_bus.h"

#include "ros/ros.h"
#include "std_msgs/String.h"


#include "roboy_communication_cognition/DirVec.h"

DEFINE_bool(big_menu, true, "Include 'advanced' options in the menu listing");
DEFINE_int32(sampling_frequency, 16000, "Sampling Frequency");

namespace hal = matrix_hal;

int led_offset[] = {23, 27, 32, 1, 6, 10, 14, 19};
int lut[] = {1, 2, 10, 200, 10, 2, 1};

/*!
 * The main function from the direction of arrival demo. It uses the DirectionOfArrival
 * class, to estimate the doa. Afterwards it publishes the calculated Information
 * onto the rostopic '/roboy/cognition/audio/direction_of_arrival'. Like normal
 * ros nodes, it ends if ros::ok() is false.
 * @param argc the standard arguments given from the command line call
 * @param argv the standard arguments given from the command line call
 * @return
 */
int main(int argc, char *argv[]) {
    // start with init
    ros::init(argc, argv, "doa_estimater");

    // create the NodeHandle and the publisher
    ros::NodeHandle n;
    ros::Publisher doa_pub = n.advertise<roboy_communication_cognition::DirVec>("/roboy/cognition/audio/direction_of_arrival",
                                                                      1000);

    ros::Rate loop_rate(100000);

    google::ParseCommandLineFlags(&argc, &argv, true);

    // necessary setups of the bus, mics and leds
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

    // define the variables that'll contain the result
    float azimutal_angle;
    float polar_angle;
    int mic;
    roboy_communication_cognition::DirVec msg;

    while (ros::ok()) {
        mics.Read(); /* Reading 8-mics buffer from de FPGA */

        /*
        // execute the direction of arrival algorithm
        doa.Calculate();

        // get the result
        azimutal_angle = doa.GetAzimutalAngle() * 180 / M_PI;
        polar_angle = doa.GetPolarAngle() * 180 / M_PI;
        mic = doa.GetNearestMicrophone();
        */


        // new attempt
        doa.ImprovedCalculation();

        //get the result
        azimutal_angle = doa.GetSourxeX();
        polar_angle = doa.GetSourceY();
        mic = doa.GetNearestMicrophone();

        // write the angles into the ros message
        msg.azimutal_angle = azimutal_angle;
        msg.polar_angle = polar_angle;

        // publish the data with the corresponding publisher
        doa_pub.publish(msg);

        // fire up that LED that lies above the nearest mic
        for (hal::LedValue &led : image1d.leds) {
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
