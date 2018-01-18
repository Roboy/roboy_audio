#include "plot.hpp"
//#include "sdft.hpp"
#include "freqOmeter.hpp"
#include <ros/ros.h>
//#include <naoqi_bridge_msgs/AudioBuffer.h>
#include "audio_common_msgs/AudioData.h"

class Microphone{
public:
    Microphone(){
	    ROS_INFO("Subscribing to microphone");
	    //sub = n.subscribe("/nao/nao_robot/microphone/naoqi_microphone/audio_raw", 1, &Microphone::microphoneCB, this);
        sub = n.subscribe("/mic0/audio", 1, &Microphone::microphoneCB, this);
	    freqOmeter.init(5460,16000);
    };
    
private:
//    void microphoneCB(naoqi_bridge_msgs::AudioBuffer::ConstPtr buf){
    void microphoneCB(audio_common_msgs::AudioData::ConstPtr buf){
//	plot.clear(0);
//	
//	plot.array(buf->data,"microphone data", 0);
//	plot.array(freqOmeter.sig , 5460, "hanning windowed signal", 0);
//	plot.setYaxis(0,-50000,50000);
        /*std::vector<int16_t> conv_data;
        for(auto const& value: buf->data.data)
        {
            conv_data.push_back((int16_t) value);
        }*/
        freqOmeter.fourierTransform(buf->data);
        freqOmeter.powr_spectrum();
        freqOmeter.detectTones();
        freqOmeter.drawTones();
	
//	plot.clear(1);
//	plot.array(freqOmeter.freq, freqOmeter.powr, 5460/2, "power spectrum", 1);
//	plot.setYaxis(1,0,1000000);
    };
    ros::NodeHandle n;
    ros::Subscriber sub;
    FreqOmeter freqOmeter;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nao_audio");
    
    Microphone mic;
    
    // this is for asyncronous ros callbacks
    ros::MultiThreadedSpinner spinner(5);
    spinner.spin();

    return 0;
}
 
