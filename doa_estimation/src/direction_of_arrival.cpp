/*
 * Copyright 2016 <Admobilize>
 * MATRIX Labs  [http://creator.matrix.one]
 * This file is part of MATRIX Creator HAL
 *
 * MATRIX Creator HAL is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <map>
#include <string>
#include <iostream>

#include "cross_correlation.h"
#include "direction_of_arrival.h"
#include "microphone_array_location.h"

namespace matrix_hal {

    DirectionOfArrival::DirectionOfArrival(MicrophoneArray &mics) : mics_(mics) {}

    bool DirectionOfArrival::Init() {
        length_ = mics_.NumberOfSamples();
        corr_ = new CrossCorrelation();
        corr_->Init(mics_.NumberOfSamples());
        current_mag_.resize(4);
        current_index_.resize(4);
        sample_difference_.resize(7);
        time_difference_.resize(7);
        buffer_1D_.resize(mics_.Channels() * mics_.NumberOfSamples());
        buffer_2D_.resize(mics_.Channels());
        mic_direction_ = 0;
        azimutal_angle_ = 0;
        polar_angle_ = 0;
        for (uint16_t c = 0; c < mics_.Channels(); c++) {
            buffer_2D_[c] = &buffer_1D_[c * mics_.NumberOfSamples()];
        }
        return true;
    }

    int DirectionOfArrival::getAbsDiff(int index) {
        if (index < length_ / 2) {
            return index;
        }
        return length_ - 1 - index;
    }

    void DirectionOfArrival::Calculate() {
        // Max delay in samples between microphones of a pair
        int max_tof = 6;

        // Prepare buffer for cross correlation calculation between the microphones of a pair
        for (uint32_t s = 0; s < mics_.NumberOfSamples(); s++) {
            for (uint16_t c = 0; c < mics_.Channels(); c++) { /* mics_.Channels()=8 */
                buffer_2D_[c][s] = mics_.At(s, c);
            }
        }

        // Loop over each microphone pair
        for (int channel = 0; channel < 4; channel++) {
            // Calculate the cross correlation
            corr_->Exec(buffer_2D_[channel + 4], buffer_2D_[channel]);

            float *c = corr_->Result();

            // Find the sample index of the highest peak (beginning of the window)
            int index = 0;
            float m = c[0];
            for (int i = 1; i < max_tof; i++)
                if (c[i] > m) {
                    index = i;
                    m = c[i];
                }

            // Find the sample index of the highest peak (end of the window)
            for (int i = length_ - max_tof; i < length_; i++)
                if (c[i] > m) {
                    index = i;
                    m = c[i];
                }

            // Store the highest value with the index of this microphone pair
            current_mag_[channel] = m;
            current_index_[channel] = index;
        }

        // Loop over all microphone pairs and find the microphone pair perpendicular to the source
        int perp = 0;
        int index = current_index_[0];
        float mag = current_mag_[0];
        for (int channel = 0; channel < 4; channel++) {
            if (getAbsDiff(current_index_[channel]) < getAbsDiff(index)) {
                perp = channel;
                if (current_mag_[channel] > mag)
                    mag = current_mag_[channel];
                index = current_index_[channel];
            }
        }

        // Determine the direction of the source (mic index)
        int dir = (perp + 2) % 4;
        if (current_index_[dir] > length_ / 2) {
            dir = (dir + 4);
        }

        // Calculate the physical angle
        mic_direction_ = dir;
        azimutal_angle_ = atan2(micarray_location[mic_direction_][1],
                                micarray_location[mic_direction_][0]);
        polar_angle_ = fabs(index) * M_PI / 2.0 / float(max_tof - 1);

    }

    void DirectionOfArrival::ImprovedCalculation() {
        // Max delay in samples between microphones of a pair
        int max_tof = 6;

        // Prepare buffer for cross correlation calculation between the microphones of a pair
        for (uint32_t s = 0; s < mics_.NumberOfSamples(); s++) {
            for (uint16_t c = 0; c < mics_.Channels(); c++) { /* mics_.Channels()=8 */
                buffer_2D_[c][s] = mics_.At(s, c);
            }
        }

        // Loop over the certain pairs (1 to j)
        for (int channel = 1; channel < 8; channel++) {
            // Calculate the cross correlation
            corr_->Exec(buffer_2D_[channel], buffer_2D_[0]);

            float *c = corr_->Result();

            // Find the sample index of the highest peak (beginning of the window)
            int index = 0;
            float m = c[0];
            for (int i = 1; i < max_tof; i++)
                if (c[i] > m) {
                    index = i;
                    m = c[i];
                }

            // Find the sample index of the highest peak (end of the window)
            for (int i = length_ - max_tof; i < length_; i++)
                if (c[i] > m) {
                    index = i;
                    m = c[i];
                }

            // Store the highest value with the index of this microphone pair
            sample_difference_[channel] = index;
            time_difference_[channel] = index * TimePerSample;
        }

        sourceX_ = 0;
        sourceY_ = 0;
        for(int i = 0; i < 7; i ++) {
            sourceX_ += X1jInverse[0][i] * SpeedOfSound * time_difference_[i];
            sourceY_ += X1jInverse[1][i] * SpeedOfSound * time_difference_[i];
        }

        /*float source[2] = {sourceX_, sourceY_};

        float min_diff = L2Norm(micarray_location[0], source);
        mic_direction_ = 0;

        for (int i = 1; i < 8; i++){
            float curr_diff = L2Norm(micarray_location[i], source);
            if (curr_diff < min_diff){
                min_diff = curr_diff;
                mic_direction_ = i;
            }
        }*/


    }

    void DirectionOfArrival::ImprovedCalculationMatrix() {
        // Max delay in samples between microphones of a pair
        int max_tof = 6;

        // Prepare buffer for cross correlation calculation between the microphones of a pair
        for (uint32_t s = 0; s < mics_.NumberOfSamples(); s++) {
            for (uint16_t c = 0; c < mics_.Channels(); c++) { /* mics_.Channels()=8 */
                buffer_2D_[c][s] = mics_.At(s, c);
            }
        }

        //float corr_matrix[8][8];
        Eigen::MatrixXf corr_matrix(8, 8);
        // Loop over the certain pairs (1 to j)
        for (int channel = 0; channel < 8; channel++) {
            for(int channel2 = channel; channel2 < 8; channel2++) {
                // Calculate the cross correlation
                // Corr(f(x),g(x)) = sum_i f[i] * g[i]
                // no complex conjugate since everything is real
                float corr = 0;
                for (int i = 0; i < length_; i ++)
                    corr += buffer_2D_[channel][i] * buffer_2D_[channel2][i];

                corr_matrix(channel, channel2) = corr;
                corr_matrix(channel2, channel) = corr;
            }
        }
        std::cout << "The eigenvalues of the Correlation Matrix: "
                  << std::endl
                  << corr_matrix.eigenvalues()
                  << std::endl;

    }

    float DirectionOfArrival::L2Norm(float *x1, float *x2) {
        float sum = 0;
        for(int i = 0; i < 2; i ++)
            sum += x1[i] * x2[i];
        return sqrtf(sum);

    }

};  // namespace matrix_hal
