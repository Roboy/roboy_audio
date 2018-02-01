#include <fftw3.h>
#include <math.h>
#include <map>
#pragma once
#include <string>
#include <vector>
#include <queue>
#include <ncurses.h>
#include <ros/ros.h>
#include <ctime>
#include <chrono>
#include <bitset>
#include <thread>
#include <memory>
#include <mutex>
#include <algorithm>

// services
#include <roboy_communication_cognition/whichTones.h>
#include <roboy_communication_cognition/setLeds.h>
#include <roboy_communication_cognition/qualityOfTone.h>
#include <roboy_communication_cognition/qualityOfTones.h>
#include <roboy_communication_cognition/moveJointSequence.h>

// messages
#include "std_msgs/Float32.h"
#include "roboy_communication_cognition/FreqPower.h"

using namespace std;
using namespace std::chrono;

static const int MultiplyDeBruijnBitPosition[32] = {
    0, 9, 1, 10, 13, 21, 2, 29, 11, 14, 16, 18, 22, 25, 3, 30,
    8, 12, 20, 28, 15, 17, 24, 7, 19, 27, 23, 6, 26, 5, 4, 31
};

enum TONE{
    E4=0,
    F4,
    Fsharp4,
    G4,
    Gsharp4,
    A4,
    Asharp4,
    H4,
    C5,
    Csharp5,
    D5,
    Dsharp5,
    E5,
    F5,
    Fsharp5,
    G5
};

struct { //0x00RRGGBB
    int green = 0x00ff00;
    int lightblue = 0x00ffff;
    int blue = 0x0000ff;
    int white = 0x000000;
    int magenta = 0xff00ff;
    int red = 0xff0000;
    int orange = 0xff5500;
    int yellow= 0xffff00;
}colors;

class FreqOmeter{
public: 
    FreqOmeter();
    ~FreqOmeter();
    // fourier transform using fftw
    void fourierTransform(vector<int16_t> signal);
    void fourierTransform(vector<uint8_t> signal);
    // power spectrum
    void powr_spectrum();
    
    /**
     * This function initializes all array for the fft transform, generates the 
     * fftw plan and decodes freuency bins into real frequencies
     * @param n number of samples in one block to be fft'd
     * @param sampleFrequency sample frequency
     */
    void init(uint n, uint sampleFrequency);
    
    /**
     * This function detects all tones of the keyboard starting in the frequency range 
     * provided
     * @param freq_low lower end of frequency range
     * @param freq_high higher end of frequency range
     * @param threshold average power threshold
     */
    void detectTones();
    
    float *powr, *freq;
    float *sig;
    uint N,NnextPower2;
    //! Major and Minor tone names
    vector<string> tone{ 
        "E4","F4","Fsharp4","G4","Gsharp4","A4","Asharp4","H4",
                "C5","Csharp5","D5","Dsharp5","E5","F5","Fsharp5","G5"
    };
    //! only 16 notes, because Nao's microphone has a frequency range of 300Hz-8kHz
    vector<float> keyboardFrequency{
            330.0f, 349.0f, 369.9f, 393.0f, 416.0f, 442.0f, 467.0f, 493.0f,
            526.0f, 556.0f, 591.0f, 625.0f, 665.0f , 701.0f, 744.0f , 786.0f
    };
    //! Major tones
    vector<string> majorTones{
            "E4","F4","G4","A4","H4","C5","D5","E5","F5","G5"
    };
    //! only 16 notes, because Nao's microphone has a frequency range of 300Hz-8kHz
    vector<float> keyboardFrequencyMajor{
            330.0f, 349.0f, 393.0f, 442.0f, 493.0f, 526.0f, 591.0f, 665.0f , 701.0f , 786.0f
    };

    vector<int> ledColor{
            colors.yellow, colors.green, colors.blue,colors.lightblue,colors.blue,colors.white,colors.blue,colors.magenta,
            colors.red,colors.blue,colors.orange,colors.blue,colors.yellow,colors.green,colors.blue,colors.lightblue
    };
    
    void drawTones();
    //                                                   400000.0f
    float freq_low=300.0f, freq_high=800.0f, threshold = 00.0f, frequencyband=5.0f, minPower = 200000;
private:
    // ncurses helper function
    uint rows,cols;
    void printTone(uint row, uint col, uint color);
    void printNoTone(uint row, uint col);
    void print(uint row, uint startcol, uint length, const char* s);
    void clearAll();
    
    double mag(fftwf_complex& c);
    /**
     * Finds the next power of two of the input
     * @param v input integer
     * @return next power of two
     */
    uint nextPower2(uint v);
    
    /**
     * Service for tones with timestamps
     * @param req time in milliseconds for detecting the tones
     * @param res tones and timestamps
     */
    bool whichTonesService(roboy_communication_cognition::whichTones::Request  &req,
                     roboy_communication_cognition::whichTones::Response &res);
    /**
     * Service for a single tone, measuring the quality of a hit with timestamps
     * @param req tone and time in milliseconds for detecting the tone
     * @param res quality (0-1) and purity (value between -1 and +1, -1 is too low, 0 is pure, and +1 is too high)
     */
    bool qualityOfTone(roboy_communication_cognition::qualityOfTone::Request  &req,
                       roboy_communication_cognition::qualityOfTone::Response &res);

    /**
     * Service for all tones, measuring the quality of a hit with timestamps
     * @param req time in milliseconds for detecting the tone
     * @param res quality (0-1) and purity (value between -1 and +1, -1 is too low, 0 is pure, and +1 is too high)
     */
    bool qualityOfTones(roboy_communication_cognition::qualityOfTones::Request  &req,
                       roboy_communication_cognition::qualityOfTones::Response &res);
    /**
     * counts the number of bits set
     * @param i bitmask
     */
    int NumberOfSetBits(int i);
    
    fftwf_complex *signal_c;
    float sampleFreq = 0;
    fftwf_plan plan1;
    //! bool vector indicating detection of tones
    int16_t toneDetected;
    mutex mux;
    deque<int16_t> toneCircularBuffer;
    uint circularBufferSize; 
    
    ros::NodeHandle n;
    ros::Publisher avgPow_pub;
    ros::ServiceServer whichTones_srv, qualityOfTone_srv, qualityOfTones_srv;
    ros::ServiceClient setLeds_srv, move_jointRelativeSequence_srv;
};