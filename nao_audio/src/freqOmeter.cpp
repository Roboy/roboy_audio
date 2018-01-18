#include "freqOmeter.hpp"

FreqOmeter::FreqOmeter(){
    //! start ncurses mode
    initscr();
    //! Start color functionality	
    start_color();
    init_pair(E4,	COLOR_YELLOW, COLOR_BLACK);
    init_pair(F4,	COLOR_GREEN, COLOR_BLACK);
    init_pair(G4,	COLOR_CYAN, COLOR_BLACK);
    init_pair(Gsharp4,	COLOR_BLUE, COLOR_BLACK);
    init_pair(A4,	COLOR_WHITE, COLOR_BLACK);
    init_pair(Asharp4,	COLOR_BLUE, COLOR_BLACK);
    init_pair(H4,	COLOR_MAGENTA, COLOR_BLACK);
    init_pair(C5,	COLOR_RED, COLOR_BLACK);
    init_pair(Csharp5,	COLOR_BLUE, COLOR_BLACK);
    init_pair(D5,	COLOR_WHITE, COLOR_BLACK);
    init_pair(Dsharp5,	COLOR_BLUE, COLOR_BLACK);
    init_pair(E5,	COLOR_YELLOW, COLOR_BLACK);
    init_pair(F5,	COLOR_GREEN, COLOR_BLACK);
    init_pair(Fsharp5,	COLOR_BLUE, COLOR_BLACK);
    init_pair(G5,	COLOR_CYAN, COLOR_BLACK);
    //! get the size of the terminal window
    getmaxyx(stdscr,rows,cols);    
    circularBufferSize = cols;
    
    whichTones_srv = n.advertiseService("/nao/awesome/whichTones", &FreqOmeter::whichTonesService, this);
    qualityOfTone_srv = n.advertiseService("/nao/awesome/qualityOfTone", &FreqOmeter::qualityOfTone, this);
    qualityOfTones_srv = n.advertiseService("/nao/awesome/qualityOfTones", &FreqOmeter::qualityOfTones, this);
    setLeds_srv = n.serviceClient<awesome_svcs::setLeds>("/nao/awesome/setLeds/");
    move_jointRelativeSequence_srv=n.serviceClient<awesome_svcs::moveJointSequence>("/nao/awesome/moveJointRelativeSequence/");

    // ros publisher
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    avgPow_pub = n.advertise<roboy_communication_cognition::FreqPower>("/roboy/cognition/audio/freqPower", 1000);
}
FreqOmeter::~FreqOmeter(){
    fftwf_free(signal_c);
    fftwf_free(sig);
    delete[] powr;
    fftwf_destroy_plan(plan1);
    endwin();
}

void FreqOmeter::fourierTransform(vector<int16_t> signal){
    for (uint i=0;i<NnextPower2;i++){
	if(i<N){
	    sig[i] = (float)signal[i];
	    // hanning window
	    sig[i] = sig[i] * 0.5 * (1 - cos(2*M_PI*i/N));
	}else{
	    sig[i] = 0.0;
	}
	
    }
    fftwf_execute(plan1);
}

void FreqOmeter::fourierTransform(vector<uint8_t> signal){
    for (uint i=0;i<NnextPower2;i++){
        if(i<N){
            sig[i] = (float)signal[i];
            // hanning window
            sig[i] = sig[i] * 0.5 * (1 - cos(2*M_PI*i/N));
        }else{
            sig[i] = 0.0;
        }

    }
    fftwf_execute(plan1);
}

void FreqOmeter::powr_spectrum(){
    for (uint i = 0; i < N/2; ++i) {
	powr[i] = mag(signal_c[i]);
    }
}

void FreqOmeter::init(uint n, uint sampleFrequency){
    N = n;
    NnextPower2 = nextPower2(N);
    sampleFreq = sampleFrequency;
    signal_c    = fftwf_alloc_complex(NnextPower2);
    sig         = fftwf_alloc_real(NnextPower2);
    powr        = new float[N/2];
    freq        = new float[N/2];
    // generate fftw plan
    plan1 = fftwf_plan_dft_r2c_1d(NnextPower2, sig, signal_c,FFTW_ESTIMATE);
    
    // decode fft frquency bins into real frequencies inHz
    for(uint j=0;j<N/2;j++){
	freq[j]=j*sampleFreq/NnextPower2;
    }
}

void FreqOmeter::detectTones(){
    roboy_communication_cognition::FreqPower freqPower;
    toneDetected = 0;
    for(uint16_t tone = 0; tone<keyboardFrequency.size(); tone++){
        if(keyboardFrequency[tone]>=freq_low && keyboardFrequency[tone]<=freq_high){
            float averagePower = 0;
            uint lowbin = (keyboardFrequency[tone]-frequencyband)*NnextPower2/sampleFreq,
                highbin = (keyboardFrequency[tone]+frequencyband)*NnextPower2/sampleFreq;
            for(uint bin = lowbin; bin < highbin; bin++){
            averagePower+=powr[bin];
            }
            averagePower/= (float)(highbin-lowbin);
            // publish this info, for further inspection
            freqPower.frequency.push_back(keyboardFrequency[tone]);
            freqPower.averagePower.push_back(averagePower);

            if(averagePower>threshold){
                toneDetected|=(int16_t)pow(2,tone);
                awesome_svcs::setLeds msg;
                msg.request.leds.push_back("eyes");
                msg.request.color = ledColor[tone];
                setLeds_srv.call(msg);
            }
        }
    }
    avgPow_pub.publish(freqPower);
}

bool FreqOmeter::whichTonesService(awesome_svcs::whichTones::Request  &req,
                     awesome_svcs::whichTones::Response &res){
    high_resolution_clock::time_point start = high_resolution_clock::now(), finish, ms;
    ros::Duration samplingTime(0.01), individualTone(0.5);
    bool newTone = false;
    do{
        ms = high_resolution_clock::now();
        {
            // locks toneCirularBuffer
            lock_guard<mutex> lock(mux);
            if (toneCircularBuffer.back()!=0){
                res.tones.push_back(toneCircularBuffer.back());
                auto timestamp = duration_cast<milliseconds>(ms-start);
                res.timestamps.push_back(timestamp.count());
                newTone=true;
            }
        }
        if(newTone) {
            individualTone.sleep();
            newTone = false;
        }else{
            samplingTime.sleep();
        }
	    finish = high_resolution_clock::now();
    }while((duration_cast<milliseconds>(finish - start).count()) < req.timeInMilliseconds);
    return true;
}

bool FreqOmeter::qualityOfTone(awesome_svcs::qualityOfTone::Request  &req,
                   awesome_svcs::qualityOfTone::Response &res){
    vector<string>::const_iterator it;
    bool iKnowThisTone = false;
    uint t, highestTone;
    if(!req.majorOnly) {
        it = std::find(tone.begin(), tone.end(), req.tone);
        iKnowThisTone = it!= tone.end() ? true : false;
    }else{
        it = std::find(majorTones.begin(), majorTones.end(), req.tone);
        iKnowThisTone = it!= majorTones.end() ? true : false;
    }
    if ( iKnowThisTone )
    {
        // I know this tone

        vector<float> toneAndNeighboringTones;
        vector<string> toneAndNeighboringTonesNames;
        if(!req.majorOnly) {
            t = it - tone.begin();
            highestTone = keyboardFrequency.size()-1;
            if (t == 0)// first tone on keyboard
            {
                toneAndNeighboringTones.push_back(keyboardFrequency[t]);
                toneAndNeighboringTones.push_back(keyboardFrequency[t + 1]);
                toneAndNeighboringTonesNames.push_back(tone[t]);
                toneAndNeighboringTonesNames.push_back(tone[t + 1]);
            } else if (t == highestTone)// last tone on keyboard
            {
                toneAndNeighboringTones.push_back(keyboardFrequency[t]);
                toneAndNeighboringTones.push_back(keyboardFrequency[t - 1]);
                toneAndNeighboringTonesNames.push_back(tone[t]);
                toneAndNeighboringTonesNames.push_back(tone[t - 1]);
            } else { // any tone in between
                toneAndNeighboringTones.push_back(keyboardFrequency[t]);
                toneAndNeighboringTones.push_back(keyboardFrequency[t + 1]);
                toneAndNeighboringTones.push_back(keyboardFrequency[t - 1]);
                toneAndNeighboringTonesNames.push_back(tone[t]);
                toneAndNeighboringTonesNames.push_back(tone[t + 1]);
                toneAndNeighboringTonesNames.push_back(tone[t - 1]);
            }
        }else{
            t = it - majorTones.begin();
            highestTone = keyboardFrequencyMajor.size()-1;
            if (t == 0)// first tone on keyboard
            {
                toneAndNeighboringTones.push_back(keyboardFrequencyMajor[t]);
                toneAndNeighboringTones.push_back(keyboardFrequencyMajor[t + 1]);
                toneAndNeighboringTonesNames.push_back(majorTones[t]);
                toneAndNeighboringTonesNames.push_back(majorTones[t + 1]);
            } else if (t == highestTone)// last tone on keyboard
            {
                toneAndNeighboringTones.push_back(keyboardFrequencyMajor[t]);
                toneAndNeighboringTones.push_back(keyboardFrequencyMajor[t - 1]);
                toneAndNeighboringTonesNames.push_back(majorTones[t]);
                toneAndNeighboringTonesNames.push_back(majorTones[t - 1]);
            } else { // any tone in between
                toneAndNeighboringTones.push_back(keyboardFrequencyMajor[t]);
                toneAndNeighboringTones.push_back(keyboardFrequencyMajor[t + 1]);
                toneAndNeighboringTones.push_back(keyboardFrequencyMajor[t - 1]);
                toneAndNeighboringTonesNames.push_back(majorTones[t]);
                toneAndNeighboringTonesNames.push_back(majorTones[t + 1]);
                toneAndNeighboringTonesNames.push_back(majorTones[t - 1]);
            }
        }

        vector<float> averagePower;
        averagePower.resize(toneAndNeighboringTones.size(),0.0f);

        high_resolution_clock::time_point start = high_resolution_clock::now(), finish, ms;
        uint iter = 0;

        bool hitPin = false;
        ros::Duration d(0.01);

        do{
            {
                // This takes over tone detection
                lock_guard<mutex> lock(mux);
                for (uint i = 0; i < toneAndNeighboringTones.size(); i++) {
                    uint lowbin = (toneAndNeighboringTones[i] - frequencyband) * NnextPower2 / sampleFreq;
                    uint highbin = (toneAndNeighboringTones[i] + frequencyband) * NnextPower2 / sampleFreq;
                    for (uint bin = lowbin; bin < highbin; bin++) {
                        if (powr[bin] > minPower)
                            averagePower[i] += powr[bin];
                    }
                    averagePower[i] /= (float) (highbin - lowbin);
                }

                int16_t tonesDetected = 0;
                if(!req.majorOnly) {
                    for (uint16_t tone = 0; tone < keyboardFrequency.size(); tone++) {
                        if (keyboardFrequency[tone] >= freq_low && keyboardFrequency[tone] <= freq_high) {
                            float averagePower = 0;
                            uint lowbin = (keyboardFrequency[tone] - frequencyband) * NnextPower2 / sampleFreq,
                                    highbin = (keyboardFrequency[tone] + frequencyband) * NnextPower2 / sampleFreq;
                            for (uint bin = lowbin; bin < highbin; bin++) {
                                averagePower += powr[bin];
                            }
                            averagePower /= (float) (highbin - lowbin);
                            if (averagePower > 200000.0f) {
                                tonesDetected |= (int16_t) pow(2, tone);
                            }
                        }
                    }
                }else{
                    for (uint16_t tone = 0; tone < keyboardFrequencyMajor.size(); tone++) {
                        if (keyboardFrequencyMajor[tone] >= freq_low && keyboardFrequencyMajor[tone] <= freq_high) {
                            float averagePower = 0;
                            uint lowbin = (keyboardFrequencyMajor[tone] - frequencyband) * NnextPower2 / sampleFreq,
                                    highbin = (keyboardFrequencyMajor[tone] + frequencyband) * NnextPower2 / sampleFreq;
                            for (uint bin = lowbin; bin < highbin; bin++) {
                                averagePower += powr[bin];
                            }
                            averagePower /= (float) (highbin - lowbin);
                            if (averagePower > 200000.0f) {
                                tonesDetected |= (int16_t) pow(2, tone);
                            }
                        }
                    }
                }

                uint numberOfDetectedTones = NumberOfSetBits(tonesDetected);
                if(numberOfDetectedTones>6) {
                    hitPin = true;
                    break;
                }

            }

            iter++;
            finish = high_resolution_clock::now();
        }while((duration_cast<milliseconds>(finish - start).count()) < req.timeInMilliseconds );

        // calculate quality with respect to maximal power over number of iterations
        print(keyboardFrequency.size()+3,0,cols," ");
        print(keyboardFrequency.size()+4,0,cols," ");
        print(keyboardFrequency.size()+5,0,cols," ");
        for(uint i=0; i<toneAndNeighboringTones.size(); i++){
//            averagePower[i]/=(float)iter;
            averagePower[i]/=(averagePower[i]+minPower);
            mvprintw(keyboardFrequency.size()+3+i,0,"averagePower of %s:\t%f", toneAndNeighboringTonesNames[i].c_str(), averagePower[i]);
        }
        print(keyboardFrequency.size()+9,0,cols," ");

        if(!hitPin) {
            res.hitPin = false;
            if (t == 0)// first or last tone on keyboard
            {
                if ((averagePower[0] + averagePower[1]) > 0.0f) {
                    res.purity = averagePower[0] / (averagePower[0] + averagePower[1]);
                } else {
                    res.purity = 0.0f;
                }
                res.directionY = 1.0f - res.purity;
            } else if (t == highestTone) {
                if ((averagePower[0] + averagePower[1]) > 0.0f) {
                    res.purity = averagePower[0] / (averagePower[0] + averagePower[1]);
                } else {
                    res.purity = 0.0f;
                }
                res.directionY = -1.0f + res.purity;
            } else { // any tone in between
                if ((averagePower[0] + averagePower[1] + averagePower[2]) > 0.0f) {
                    res.purity = averagePower[0] / (averagePower[0] + averagePower[1] + averagePower[2]);
                } else {
                    res.purity = 0.0f;
                }
                if (averagePower[1] > averagePower[2]) { // if I'm too high, go lower
                    if ((averagePower[0] + averagePower[1]) > 0.0f) {
                        res.directionY = 1.0f - averagePower[0] / (averagePower[0] + averagePower[1]);
                    } else {
                        res.directionY = 1.0f;
                    }
                    mvprintw(keyboardFrequency.size() + 9, 0, "too high");
                } else {  // if I'm too low, go higher
                    if ((averagePower[0] + averagePower[2]) > 0.0f) {
                        res.directionY = -1.0f + averagePower[0] / (averagePower[0] + averagePower[2]);
                    } else {
                        res.directionY = -1.0f;
                    }
                    mvprintw(keyboardFrequency.size() + 9, 0, "too low");
                }
            }
            res.quality = averagePower[0];
            if(res.quality==0.0f && res.purity == 0.0f)
                res.directionZ = -1.0f;
            else
                res.directionZ = 0.0f;

            print(keyboardFrequency.size() + 1, 0, cols, " ");
            mvprintw(keyboardFrequency.size() + 1, 0, "quality of %s :\t%f", (*it).c_str(), res.quality);
            print(keyboardFrequency.size() + 2, 0, cols, " ");
            mvprintw(keyboardFrequency.size() + 2, 0, "purity of %s :\t%f", (*it).c_str(), res.purity);
            print(keyboardFrequency.size() + 6, 0, cols, " ");
            mvprintw(keyboardFrequency.size() + 6, 0, "move in Z direction :\t%f", res.directionZ);
            print(keyboardFrequency.size() + 7, 0, cols, " ");
            mvprintw(keyboardFrequency.size() + 7, 0, "move in Y direction :\t%f", res.directionY);

            print(keyboardFrequency.size() + 8, 0, cols, " ");
	    print(keyboardFrequency.size() + 9, 0, cols, " ");
        }else{
            mvprintw(keyboardFrequency.size() + 9, 0, "hit pin");
            res.quality = 0;
            res.purity = 0;
            res.directionZ = 0;
            res.directionY = 0;
            res.hitPin = true;
        }
        refresh();
        return true;
    }else{
        // I do not know this tone
        print(keyboardFrequency.size()+8,0,cols," ");
        mvprintw(keyboardFrequency.size()+8,0,"I don't know %s",  req.tone.c_str());
        return false;
    }
}

bool FreqOmeter::qualityOfTones(awesome_svcs::qualityOfTones::Request  &req,
                   awesome_svcs::qualityOfTones::Response &res){
    
    uint highestTone;
    vector<float> averagePower;
    if(!req.majorOnly){
	highestTone = keyboardFrequency.size()-1;
	averagePower.resize(keyboardFrequency.size(),0.0f);
    }else{
	highestTone = keyboardFrequencyMajor.size()-1;
	averagePower.resize(keyboardFrequencyMajor.size(),0.0f);
    }
    
    high_resolution_clock::time_point start = high_resolution_clock::now(), finish, ms;
    uint iter = 0;
    
    bool hitPin = false;
    
    do{
	{
	    // This takes over tone detection
	    lock_guard<mutex> lock(mux);
	    
	    int16_t tonesDetected = 0;
	    if(!req.majorOnly) {
		for (uint i = 0; i < keyboardFrequency.size(); i++) {
		    uint lowbin = (keyboardFrequency[i] - frequencyband) * NnextPower2 / sampleFreq;
		    uint highbin = (keyboardFrequency[i] + frequencyband) * NnextPower2 / sampleFreq;
		    for (uint bin = lowbin; bin < highbin; bin++) {
			if (powr[bin] > minPower)
			    averagePower[i] += powr[bin];
		    }
		    averagePower[i] /= (float) (highbin - lowbin);
		}
		
		for (uint16_t tone = 0; tone < keyboardFrequency.size(); tone++) {
		    if (keyboardFrequency[tone] >= freq_low && keyboardFrequency[tone] <= freq_high) {
			float averagePower = 0;
			uint lowbin = (keyboardFrequency[tone] - frequencyband) * NnextPower2 / sampleFreq,
				highbin = (keyboardFrequency[tone] + frequencyband) * NnextPower2 / sampleFreq;
			for (uint bin = lowbin; bin < highbin; bin++) {
			    averagePower += powr[bin];
			}
			averagePower /= (float) (highbin - lowbin);
			if (averagePower > 200000.0f) {
			    tonesDetected |= (int16_t) pow(2, tone);
			}
		    }
		}
	    }else{
		for (uint i = 0; i < keyboardFrequencyMajor.size(); i++) {
		    uint lowbin = (keyboardFrequencyMajor[i] - frequencyband) * NnextPower2 / sampleFreq;
		    uint highbin = (keyboardFrequencyMajor[i] + frequencyband) * NnextPower2 / sampleFreq;
		    for (uint bin = lowbin; bin < highbin; bin++) {
			if (powr[bin] > minPower)
			    averagePower[i] += powr[bin];
		    }
		    averagePower[i] /= (float) (highbin - lowbin);
		}
		
		for (uint16_t tone = 0; tone < keyboardFrequencyMajor.size(); tone++) {
		    if (keyboardFrequencyMajor[tone] >= freq_low && keyboardFrequencyMajor[tone] <= freq_high) {
			float averagePower = 0;
			uint lowbin = (keyboardFrequencyMajor[tone] - frequencyband) * NnextPower2 / sampleFreq,
				highbin = (keyboardFrequencyMajor[tone] + frequencyband) * NnextPower2 / sampleFreq;
			for (uint bin = lowbin; bin < highbin; bin++) {
			    averagePower += powr[bin];
			}
			averagePower /= (float) (highbin - lowbin);
			if (averagePower > 200000.0f) {
			    tonesDetected |= (int16_t) pow(2, tone);
			}
		    }
		}
	    }
	    
	    uint numberOfDetectedTones = NumberOfSetBits(tonesDetected);
	    if(numberOfDetectedTones>6) {
		hitPin = true;
		break;
	    }
	    
	}
	
	iter++;
	finish = high_resolution_clock::now();
    }while((duration_cast<milliseconds>(finish - start).count()) < req.timeInMilliseconds );
    
    // calculate quality with respect to maximal power over number of iterations
    for(uint i=0; i<averagePower.size(); i++){
	averagePower[i]/=(averagePower[i]+minPower);
    }
    
    res.quality.resize(averagePower.size(),0.0f);
    res.purity.resize(averagePower.size(),0.0f);
    res.directionZ = 0.0f;
    res.directionY.resize(averagePower.size(),0.0f);
    
    if(!hitPin) {
	res.hitPin = false;
	for(uint i=0; i<averagePower.size(); i++){
	    if (i == 0)// first tone on keyboard
	    {
		if ((averagePower[i] + averagePower[i+1]) > 0.0f) {
		    res.purity[i] = averagePower[i] / (averagePower[i] + averagePower[i+1]);
		} else {
		    res.purity[i] = 0.0f;
		}
		res.directionY[i] = 1.0f - res.purity[i];
	    } else if (i == highestTone) {
		if ((averagePower[i] + averagePower[i-1]) > 0.0f) {
		    res.purity[i] = averagePower[i] / (averagePower[i] + averagePower[i-1]);
		} else {
		    res.purity[i] = 0.0f;
		}
		res.directionY[i] = -1.0f + res.purity[i];
	    } else { // any tone in between
		if ((averagePower[i] + averagePower[i-1] + averagePower[i+1]) > 0.0f) {
		    res.purity[i] = averagePower[i] / (averagePower[i] + averagePower[i-1] + averagePower[i+1]);
		} else {
		    res.purity[i] = 0.0f;
		}
		if (averagePower[i+1] > averagePower[i-1]) { // if I'm too high, go lower
		    if ((averagePower[i] + averagePower[i-1]) > 0.0f) {
			res.directionY[i] = 1.0f - averagePower[i] / (averagePower[i] + averagePower[i-1]);
		    } else {
			res.directionY[i] = 1.0f;
		    }
		} else {  // if I'm too low, go higher
		    if ((averagePower[i] + averagePower[i+1]) > 0.0f) {
			res.directionY[i] = -1.0f + averagePower[i] / (averagePower[i] + averagePower[i+1]);
		    } else {
			res.directionY[i] = -1.0f;
		    }
		}
	    }
	    res.quality[i] = averagePower[i];
	    if(res.quality[i]==0.0f && res.purity[i] == 0.0f)
		res.directionZ = -1.0f;

	    if(req.majorOnly)
		mvprintw(keyboardFrequency.size() + i, 0, "%s quality/purity: %.5f / %.5f", majorTones[i].c_str(), res.quality[i], res.purity[i]);
	    else
		mvprintw(keyboardFrequency.size() + i, 0, "%s quality/purity: %.5f / %.5f", tone[i].c_str(), res.quality[i], res.purity[i]);
	}
    }else{
	mvprintw(keyboardFrequency.size() + averagePower.size()+1, 0, "hit pin");
	res.hitPin = true;
    }
    refresh();
    return true;

}

int FreqOmeter::NumberOfSetBits(int i)
{
    uint c = 0;
    for (c = 0; i; i >>= 1)
    {
        c += i & 1;
    }
    return c;
};

void FreqOmeter::drawTones(){
//    bitset<16> b(toneDetected);
//    cout << b << std::endl;
    clearAll();
    
    // locks toneCirularBuffer
    lock_guard<mutex> lock(mux);
    
    if(toneCircularBuffer.size()>=circularBufferSize)
	toneCircularBuffer.pop_front();
    toneCircularBuffer.push_back(toneDetected);
    
    for(uint i=0;i<toneCircularBuffer.size();i++){
	for(uint tone=0; tone<keyboardFrequency.size();tone++){
	    if(toneCircularBuffer[i]&(int16_t)pow(2,tone))
		printTone(tone,i,tone);
	    else
		printNoTone(tone,i);
	}
    }
    refresh();
}

void FreqOmeter::printTone(uint row, uint col, uint color){
    mvprintw(row,col,"o");
    mvchgat(row, col, 1, A_BOLD, color, NULL);
}

void FreqOmeter::printNoTone(uint row, uint col){
    mvprintw(row,col,"-");
}

void FreqOmeter::print(uint row, uint startcol, uint length, const char* s){
    for (uint i=startcol;i<startcol+length;i++){
	mvprintw(row,i,"%s",s);
    }
}
    
void FreqOmeter::clearAll(){
    for(uint i=0;i<keyboardFrequency.size();i++){
	print(i,0,cols," ");
    }
}

double FreqOmeter::mag(fftwf_complex& c){
    return sqrt(c[0] * c[0] + c[1] * c[1]);
}    

uint FreqOmeter::nextPower2(uint v){
    // find the log base 2 of 32-bit v
    uint r;      // result goes here
    
    v |= v >> 1; // first round down to one less than a power of 2
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    
    r = MultiplyDeBruijnBitPosition[(uint32_t)(v * 0x07C4ACDDU) >> 27];
    v=pow(2,r+1);
    return v;
}
