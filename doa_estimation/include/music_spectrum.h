//
// Created by parallels on 3/1/18.
//

#ifndef DOA_ESTIMATION_MUSIC_SPECTRUM_H
#define DOA_ESTIMATION_MUSIC_SPECTRUM_H
#define _USE_MATH_DEFINES

#include <Eigenvalues>
#include <boost/math/special_functions/math_fwd.hpp>
#include <math.h>
class MusicSpectrum {
public:
    MusicSpectrum(int N, int M);

    Eigen::VectorXcd v(double angle);
    Eigen::MatrixXd J(double angle);
    double alpha(double param);

private:
    Eigen::MatrixXcd _W;

    int _M; // highest order mode
    int _N; // number of mics
};
#endif //DOA_ESTIMATION_MUSIC_SPECTRUM_H
