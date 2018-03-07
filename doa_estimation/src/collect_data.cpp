//
// Created by parallels on 3/7/18.
//

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

#include <fstream>
#include <sstream>

DEFINE_bool(big_menu, true, "Include 'advanced' options in the menu listing");
DEFINE_int32(sampling_frequency, 16000, "Sampling Frequency");

namespace hal = matrix_hal;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "data_collecter");

    ros::NodeHandle n;

    google::ParseCommandLineFlags(&argc, &argv, true);

    // necessary setups of the bus, mics and leds
    hal::WishboneBus bus;
    bus.SpiInit();

    hal::MicrophoneArray mics;
    mics.Setup(&bus);

    int sampling_rate = FLAGS_sampling_frequency;
    mics.SetSamplingRate(sampling_rate);
    mics.ShowConfiguration();

    std::ofstream data_file[8];
    for(int i = 0; i < mics.Channels(); i++){
        std::stringstream file_name;
        file_name << "/home/roboy/workspace/src/roboy_audio/doa_estimation/data/mic_" << i << ".csv";
        data_file[i].open(file_name.str(), std::ofstream::out);
    }
    for(int i = 0; mics.Channels(); i++) {
        for(int s = 0; s < mics.NumberOfSamples(); s++)
            data_file[i] << s << ";";
        data_file[i] << std::endl;
    }

    for(int i = 0; i < 10; i++) {
        mics.Read();

        for(int c = 0; c < mics.Channels(); c++){
            for (int s = 0; s < mics.NumberOfSamples(); s++)
                data_file[c] << mics.At(s, c) << ";";
            data_file[c] << std::endl;
        }
    }

    for(int i = 0; mics.Channels(); i++)
        data_file[i].close();
}
