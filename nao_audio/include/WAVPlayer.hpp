#pragma once

#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <AL/al.h>
#include <AL/alc.h>
#include <audio/wave.h>
#include <inttypes.h>
#include <iostream>
#include <stdint.h>
#include <cstring>

struct Sample{
    ALuint  source;
    char*   bufferData = NULL;
    uint bufferLen;
    short   channels, bitsPerSample;
    char   filePath[200];
};

static void getALerror(){
    ALint error = alGetError();
    switch (error){
        case AL_INVALID_NAME:
            printf("ERROR: AL_INVALID_NAME\n");
            break;
        case AL_INVALID_ENUM:
            printf("ERROR: AL_INVALID_ENUM\n");
            break;
        case AL_INVALID_VALUE:
            printf("ERROR: AL_INVALID_VALUE\n");
            break;
        case AL_INVALID_OPERATION:
            printf("ERROR: AL_INVALID_OPERATION\n");
            break;
        case AL_OUT_OF_MEMORY:
            printf("ERROR: AL_OUT_OF_MEMORY\n");
            break;
    }
}

static inline ALenum to_al_format(short channels, short samples){
    bool stereo = (channels > 1);
    
    switch (samples) {
        case 16:
            if (stereo)
                return AL_FORMAT_STEREO16;
            else
                return AL_FORMAT_MONO16;
        case 8:
            if (stereo)
                return AL_FORMAT_STEREO8;
            else
                return AL_FORMAT_MONO8;
        default:
            return -1;
    }
}

void list_audio_devices(const ALCchar *devices)
{
    const ALCchar *device = devices, *next = devices + 1;
    size_t len = 0;
    
    fprintf(stdout, "Devices list:\n");
    fprintf(stdout, "----------\n");
    while (device && *device != '\0' && next && *next != '\0') {
        fprintf(stdout, "%s\n", device);
        len = strlen(device);
        device += (len + 1);
        next += (len + 2);
    }
    fprintf(stdout, "----------\n");
}

class openAL{
public:
    openAL(){
        ALboolean enumeration = alcIsExtensionPresent(NULL, "ALC_ENUMERATION_EXT");
        if (enumeration == AL_FALSE)
            fprintf(stderr, "enumeration extension not available\n");
        
        device = alcOpenDevice(NULL);
        if (!device) {
            fprintf(stderr, "unable to open default device\n");
        }else{
            printf( "Device: %s\n", alcGetString(device, ALC_DEVICE_SPECIFIER));
        }
        
        alGetError();
        
        context = alcCreateContext(device, NULL);
        if (!alcMakeContextCurrent(context)) {
            fprintf(stderr, "failed to make default context\n");
        }else{
            printf( "made default context\n");
        }
    }
    ~openAL(){
        alcCloseDevice(device);
        getALerror();
    }
    
    uint loadWav(char* filePath){
        ALuint buffer;
        WaveInfo *wave;
        char *bufferData;
        ALuint			source;
        uint ret;
        printf("loading wave file %s\n", filePath);
        //load the wave file
        wave = WaveOpenFileForReading(filePath);
        if (!wave) {
            fprintf(stderr, "failed to read wave file %s\n", filePath);
        }
        
        ret = WaveSeekFile(0, wave);
        if (ret) {
            fprintf(stderr, "failed to seek wave file\n");
        }
        
        bufferData = (char*)malloc(wave->dataSize);
        if (!bufferData) {
            fprintf(stderr,"malloc\n");
        }
        
        ret = WaveReadFile(bufferData, wave->dataSize, wave);
        if (ret != wave->dataSize) {
            fprintf(stderr, "short read: %d, want: %d\n", ret, wave->dataSize);
        }
        
        alGenBuffers(1, &buffer);
        
        //put the data into our sampleet buffer
        ALenum  format = to_al_format(wave->channels, wave->bitsPerSample);
        alBufferData(buffer, format, bufferData, wave->dataSize, wave->sampleRate); getALerror();
        
        //create a source
        alGenSources(1, &source); getALerror();
        //assign the buffer to this source
        alSourcei(source, AL_BUFFER, buffer); getALerror();
        
        sample.resize(sample.size()+1);
        sample.back().source        = source;
        sample.back().bufferData    = bufferData;
        sample.back().bufferLen     = wave->numSamples;
        sample.back().channels      = wave->channels;
        sample.back().bitsPerSample = wave->bitsPerSample;
        strcpy(sample.back().filePath,filePath);
        return sample.size()-1;
    }
    void replaceSource(uint nr, int16_t* bufferData,long bufferLen,ALsizei freq, char* name){
        uint source;
        uint buffer;
        
        uint channels=2;
        uint bitsPerSample=16;
        ALenum  format = to_al_format(channels,bitsPerSample);
        
        //create a source
        alGenSources(1, &source); getALerror();
        //create  buffer
        alGenBuffers(1, &buffer); getALerror();
        //put the bufferData into buffer
        alBufferData(buffer, format, bufferData, bufferLen*sizeof(int16_t)*2, freq); getALerror();
        //assign the buffer to this source
        alSourcei(source, AL_BUFFER, buffer); getALerror();
        
        //delete source
        alDeleteSources(1,&sample.at(nr).source); getALerror();
        
        sample.at(nr).source        = source;
        sample.at(nr).bufferData    = (char*)(void*)bufferData;
        sample.at(nr).bufferLen     = bufferLen;
        sample.at(nr).channels      = channels;
        sample.at(nr).bitsPerSample = bitsPerSample;
        strcpy(sample.at(nr).filePath,name);
    }
    
    int addSource(int16_t* bufferData,long bufferLen,ALsizei freq, char* name){
        for(uint i=0;i<sample.size();i++){
            if(strcmp(sample.at(i).filePath, name) == 0){
                replaceSource(i,bufferData,bufferLen,freq,name);
                return i;
            }
        }
        
        uint source;
        uint buffer;
        
        uint channels=2;    // output from cnn is mono!
        uint bitsPerSample=16;
        ALenum  format = to_al_format(channels,bitsPerSample);
        
        //create a source
        alGenSources(1, &source); getALerror();
        
        //create  buffer
        alGenBuffers(1, &buffer); getALerror();
        //put the bufferData into buffer
        alBufferData(buffer, format, bufferData, bufferLen*sizeof(int16_t)*2, freq); getALerror();
        //assign the buffer to this source
        alSourcei(source, AL_BUFFER, buffer); getALerror();    
        
        sample.resize(sample.size()+1);
        sample.back().source        = source;
        sample.back().bufferData    = (char*)(void*)bufferData;
        sample.back().bufferLen     = bufferLen;
        sample.back().channels      = channels;
        sample.back().bitsPerSample = bitsPerSample;
        strcpy(sample.back().filePath,name);
        return sample.size()-1;
    }
    
    /*! playSources
     * \param sources - sampls to play
     */
    void playSources(std::vector<uint> &sources){
        uint size = sources.size();
        ALuint* src=new ALuint[size];
        for (uint i=0;i<size;i++){ 
            uint s = sources[i];
            src[i]=sample[s].source;
        }
        alSourcePlayv(size,src);
        ALint state;
        for (uint i=0;i<size;i++){ 
            alGetSourcei(src[i],AL_SOURCE_STATE,&state);
            if (state==AL_PLAYING)
                printf("playback %d\n",sources[i]);
            else
                getALerror();
        }
        delete[] src;
    }

    /*! playSource
     * \param source - sampls to play
     */
    void playSource(uint s){
        ALuint src = sample[s].source;
        alSourcePlay(src);
        ALint state;
        alGetSourcei(src,AL_SOURCE_STATE,&state);
        if (state==AL_PLAYING)
            printf("playback %d\n",s);
        else
            getALerror();
    }
    
    /*! pauseSource
     * \param nr - sample to pause
     */
    void pauseSources(std::vector<uint> &sources){
        uint size = sources.size();
        ALuint* src=new ALuint[size];
        for (uint i=0;i<size;i++){ 
            uint s = sources[i];
            printf("pause sample %d\n", s);
            src[i]=sample[s].source;
        }
        alSourcePausev(size,src);
        delete[] src;
    }
    
    /*! rewindSource
     * \param nr - sample to stop and rewind
     */
    void rewindSources(std::vector<uint> &sources){
        uint size = sources.size();
        ALuint* src=new ALuint[size];
        for (uint i=0;i<size;i++){ 
            uint s = sources[i];
            printf("rewind sample %d\n", s);
            src[i]=sample[s].source;
        }
        alSourceRewindv(size,src);
        delete[] src;
    }
    
    /*! stopSource
     * \param nr - sample to stop
     */
    void stopSources(std::vector<uint> &sources){
        uint size = sources.size();
        ALuint* src=new ALuint[size];
        for (uint i=0;i<size;i++){ 
            uint s = sources[i];
            printf("stop sample %d\n", s);
            src[i]=sample[s].source;
        }
        alSourceStopv(size,src);
        delete[] src;
    }
    
    /*! setLoopingSource
     * \param nr - sample to loop
     * \param loop - (0) no looping (1) looping
     */
    void setLoopingSource(std::vector<uint> &sources, bool loop){
        uint size = sources.size();
        for (uint i=0;i<size;i++){
            uint s = sources[i];
            alSourcei(sample[s].source,AL_LOOPING,loop);
        }
    }
    
    /*! getPositionSec
     * \param sources - samples to get current position in seconds
     * \param seconds - float
     */
    void getPositionSec(std::vector<uint> &sources, std::vector<float> &seconds){
        uint size = sources.size();
        for (uint i=0;i<size;i++){ 
            float sec = 0.0;
            alGetSourcef(sample[sources[i]].source,AL_SEC_OFFSET,&sec);
            seconds[i]=sec;
        }
    }
    
    /*! getPositionSec
     * \param sources - samples to get current position in samples
     * \param N - sample offset
     */
    void getPositionSamples(std::vector<uint> &sources, std::vector<float> &N){
        uint size = sources.size();
        for (uint i=0;i<size;i++){ 
            ALint n = 0.0;
            alGetSourcei(sample[sources[i]].source,AL_SAMPLE_OFFSET,&n);
            N[i]=n;
        }
    }
    
    /*! changeSpeedPy
     * \param sources - sample to increase speed
     * \param speed - new speed (pitch)
     */
    void changeSpeed(std::vector<uint> sources, float speed){
        uint size = sources.size();
        for (uint i=0;i<size;i++){
            uint s = sources[i];
            alSourcef (sample[s].source, AL_PITCH, speed);
        }
    }
    
    /*! changeVolumePy
     * \param sources - sample to increase speed
     * \param volume - new volume
     */
    void changeVolume(std::vector<uint> sources, float volume){
        uint size = sources.size();
        for (uint i=0;i<size;i++){
            uint s = sources[i];
            alSourcef (sample[s].source, AL_GAIN, volume);
        }
    }
    
    
    void info(std::vector<uint> sources, std::vector<uint> &bufferLen){
        uint size = sources.size();
        for (uint i=0;i<size;i++){
            uint s = sources[i];
            printf("file %s:\n", sample[s].filePath);
            printf("channels:      %d\n", sample[s].channels);
            printf("bits:          %d\n", sample[s].bitsPerSample);
            printf("length:        %d\n", sample[s].bufferLen);
            bufferLen[s] = sample[s].bufferLen;
        }
    }
    
    void getBuffer(std::vector<uint> sources, std::vector<float*> &buffer){
        uint size = sources.size();
        for (uint i=0;i<size;i++){
            uint s = sources[i];
            if(sample[s].bitsPerSample == 16){
                int16_t* data = (int16_t*)sample[s].bufferData;
                for(uint j=0; j<sample[s].bufferLen; j++){
                    buffer[s][j] = (float)data[j];
                }
            }else if(sample[s].bitsPerSample == 8){
                int8_t* data = (int8_t*)sample[s].bufferData;
                for(uint j=0; j<sample[s].bufferLen; j++){
                    buffer[s][j] = (float)data[j];
                }
            }
        }
    }
    
    void getBuffer(std::vector<uint> sources, std::vector<std::vector<int16_t>> &buffer){
        uint size = sources.size();
        for (uint i=0;i<size;i++){
            uint s = sources[i];
            if(sample[s].bitsPerSample == 16){
                int16_t* data = (int16_t*)sample[s].bufferData;
                for(uint j=0; j<sample[s].bufferLen; j++){
                    buffer[s][j] = data[j];
                }
            }else if(sample[s].bitsPerSample == 8){
                int8_t* data = (int8_t*)sample[s].bufferData;
                for(uint j=0; j<sample[s].bufferLen; j++){
                    buffer[s][j] = data[j];
                }
            }
        }
    }
    
    void getBuffer(uint source, std::vector<int16_t> &buffer){
        if(sample[source].bitsPerSample == 16){
            int16_t* data = (int16_t*)sample[source].bufferData;
            for(uint j=0; j<buffer.size(); j++){
                buffer[j] = data[j];
            }
        }else if(sample[source].bitsPerSample == 8){
            int8_t* data = (int8_t*)sample[source].bufferData;
            for(uint j=0; j<buffer.size(); j++){
                buffer[j] = data[j];
            }
        }
    }

    std::vector<Sample> sample;
private:
    ALCenum error; 
    ALCcontext *context;    
    ALCdevice *device;      
};



