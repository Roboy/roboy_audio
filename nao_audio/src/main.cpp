#include "WAVPlayer.hpp"
#include "freqOmeter.hpp"
#include "plot.hpp"
#include <map>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

#define RECORDINGS
#define PLOT_INPUTSIGNAL

#ifndef RECORDINGS
    std::vector<std::string> samples{"chirp_1Hz_500Hz","chrip1Hz_10000Hz","C4","D4","E4","F4","G4","A4","H4","C5"};
#else
    std::vector<std::string> samples;
#endif

std::map < std::string, float > samplesFreq{  
    {"C4", 261.626},
    {"D4", 293.665},
    {"E4", 329.628},
    {"F4", 349.228},
    {"G4", 391.995},
    {"A4", 440.000},
    {"H4", 493.883},
    {"C5", 523.251}
};

int main(int argc, char *argv[])
{
    
    openAL player;
    char str[100];
    
#ifndef RECORDINGS
    for (uint i=0;i<samples.size();i++){
        sprintf(str, "/home/hrs2015/catkin_ws/src/nao_audio/samples/%s.wav", samples[i].c_str());
#else
    for (uint i=0;i<30;i++){
	sprintf(str, "/home/hrs2015/catkin_ws/src/nao_audio/samples/glockenspiel/Voice%d.wav", i);
	samples.push_back(str);
#endif
        player.loadWav(str);
    }

#ifndef RECORDINGS
    std::vector<uint> nr = {0,1,2,3,4,5,6,7,8,9};
#else
    std::vector<uint> nr = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};
#endif
 
    std::vector<uint> bufferlen;
    bufferlen.resize(nr.size());
    player.info(nr,bufferlen);
    
    // ------------------------------ FFTW ---------------------------------------------
    FreqOmeter freqOmeter;
    
    std::string ss;
        
    for (uint s=0;s<nr.size();s++){
//        clock_t start2 = clock();
	
	printf("bufferlength %d\n", bufferlen[s]);

	freqOmeter.init(bufferlen[s],44100);
	std::vector<int16_t> buffer2;
	buffer2.resize(bufferlen[s]);
	
	player.playSource(s);
	
	player.getBuffer(s, buffer2);
                
        freqOmeter.fourierTransform(buffer2);
	freqOmeter.powr_spectrum();
	
#ifdef PLOT_INPUTSIGNAL
	sprintf(str, "signal %d", s);
	plot.array(buffer2, str, 0);
	sprintf(str, "signal %d windowed", s);
	plot.array(freqOmeter.sig, freqOmeter.NnextPower2, str, 0);
#endif
	
	sprintf(str, "power spectrum %s", samples[s].c_str());
	plot.array(freqOmeter.freq, freqOmeter.powr, freqOmeter.N/2, str, 1);
//	plot.set_ylogscale(1);
	plot.setYaxis(1,0,5000000);
        
//        clock_t finish2 = clock();
	std::cin >> ss;
    }
    while(1){};
    
    return 0;
}
 
