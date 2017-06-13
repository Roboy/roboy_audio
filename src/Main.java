//project Music Beats Detection
//by Petr Romanov

import java.io.*;
import java.util.concurrent.TimeUnit;

import static java.lang.Math.abs;
/**
 * \brief Class detecting percussion beats in music
 * \author Petr Romanov
 * \version 1.0
 * \date 08.06.2017
 * \details Takes a music file and sends a signal to the control unit
 *          when a beat splash is detected in sound stream.
 **/
public class Main
{
    /**
     * \brief Takes sound file bytes array and finds DATA tag
     * \return end of the tag position
     */
    public static int find_data_tag(Samples cassette)
    {
        int i = 0;
        byte t = 0;
        for (i=30, t=0; i< 500 && t==0; i++)
            if ((char)cassette.buffer[i]=='d' && (char)cassette.buffer[i+1]=='a' &&
                    (char)cassette.buffer[i+2]=='t' && (char)cassette.buffer[i+3]=='a')
            {
                t=1;
                System.out.println("DATA tag found");
                return i;
            }
        return 0;
    }
    /**
     * \brief Takes end of the DATA tag position and sound file bytes array to define the number of samlpes
     * \return doubled number of samples because array is of byte type, but samples are of two bytes
     */
    public static long define_subchunk(int start, Samples cassette)
    {
        int i = 0;
        long subchunk2Size = 0;
        for (i=start+7; i>start+4; i--)
        {
            subchunk2Size += cassette.buffer[i]<0 ? (long)(256+cassette.buffer[i]) : (long)cassette.buffer[i];
            subchunk2Size = subchunk2Size << 8;
        }
        subchunk2Size += cassette.buffer[i]<0 ? (long)(256+cassette.buffer[i]) : (long)cassette.buffer[i];
        return subchunk2Size;
    }
    /**
     * \brief Takes position in array and array itself to join two bytes in one sample
     * \return A sample of word type
     */
    public static short take_sample(int i, Samples cassette)
    {
        short smpl_seq = 0;
        smpl_seq += cassette.buffer[i+1]<0 ? (short)(256+cassette.buffer[i+1]) : (short)cassette.buffer[i+1];
        smpl_seq = (short)(smpl_seq << 8);
        smpl_seq += cassette.buffer[i]<0 ? (short)(256+cassette.buffer[i]) : (short)cassette.buffer[i];
        return smpl_seq;
    }
    /**
     * \brief Takes position in array, width of the desired segment and array itself to calculate their mean value
     * \return Mean value of given number of samples
     */
    public static int calc_mean(int m, int start, Samples cassette)
    {
        int i = 0, mean = 0;
        short smpl_seq = 0;
        for ( i = start; i < start+m; )
        {
            smpl_seq = take_sample(i, cassette);
            mean += (int)smpl_seq;
            i+=2;
        }
        return mean*2/m;
    }
    /**
     * \brief Analyses sound stream on the presence of beats
     * \return A sample of word type
     */
    public static void main(String[] args)
    {
        byte t = 0;
        int i = 0, j = 0, d = 0, g = 0, n = 0, window = 0,
          bitsPerSample = 0,//16 - bits per sample => 32 bits is enough (8bits*4)
          threshold = 0, //max = 65535
          left_mean = 0, right_mean = 0, max_ampl = 0, period = 0, absolute = 0;//5 sec. - 430000; 3 - 258000
        long  subchunk2Size = 0;
        short smpl_seq = 0;
        Samples cassette = new Samples();
        cassette.initialise("src//song000.wav");
        bitsPerSample += cassette.buffer[35]<0 ? (int)(256+cassette.buffer[35]) : (int)cassette.buffer[35];
        bitsPerSample = bitsPerSample << 8;
        bitsPerSample = cassette.buffer[34]<0 ? (int)(256+cassette.buffer[34]) : (int)cassette.buffer[34];
        d = find_data_tag(cassette);
        subchunk2Size = define_subchunk(d, cassette);
        System.out.println("Bits per sample: "+bitsPerSample);
        //System.out.println("Subchunk size defined: "+subchunk2Size/2); //the same as the next line
        System.out.println("Samples number: "+(cassette.buffer.length-d-8)/2 );
        window = 200;//100 samples -
        // heuristics! - average noticed time of signal envelope+local_SD crossing the time axis
        n = (cassette.buffer.length-window);
        for (i = d+8+window; i < n; )
        {
            left_mean = calc_mean(window, i-window, cassette);
            right_mean = calc_mean(window, i, cassette);
            if (left_mean < 0 && right_mean > 0)
            {
                for (j = i+window, t = 0; j < n && t == 0; )
                {
                    smpl_seq = take_sample(j, cassette);
                    absolute = (int)abs(smpl_seq);
                    if ( max_ampl <  absolute)
                    {
                        max_ampl = absolute;
                    }
                    left_mean = calc_mean(window, j-window, cassette);
                    right_mean = calc_mean(window, j, cassette);
                    if (left_mean < 0 && right_mean > 0)
                    {
                        t = 1;
                    }
                    j+=2;
                }
                period = j - i;
                if (period > 1400 && max_ampl > 10000)
                {
                    absolute = (i-d-8)/2;
                    System.out.println("beat: "+absolute/44138+","+(absolute%44138)*100/44138+"sec" );
                    System.out.println("sample №: "+absolute);
                    i += 43000;//~1 sec skipped (for future when legs will be completed)
                    //few further lines are for the realtime dance
                    /*try
                    {
                        TimeUnit.MILLISECONDS.sleep(500);
                    }
                    catch (InterruptedException ex)
                    {
                        ex.printStackTrace();
                    }*/
                }
                else
                {
                    g = i;
                    i = (n - j) % 2 == 0 ? j + 2 : j + 1;//for real-time a pause is required
                }
            }
            else
            {
                i += 2;//for real-time a pause is required
            }
        }

    }
}
