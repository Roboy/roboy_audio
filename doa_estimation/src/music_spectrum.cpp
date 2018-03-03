#include "music_spectrum.h"


void MusicSpectrum::MusicSpectrum(int N, int M) {
    _N = N;
    _M = M;

    // the W calculation
    // W should look like [v(alpha(-M)): ... : v(alpha(0)) : ... : v(alpha(M))]
    _W.resize(2*M+1, 2*M+1);

    int m = -_M;
    for(int i = 0; i < (2*M+1); i++){
        Eigen::VectorXcd vectorXcd = v(alpha(m));
        _W.col(i) = vectorXcd;
    }
}
/**
 * creates the steering vector responsible for the azimuthal angle.
 * it looks like [ exp(-j *M * angle), ..., exp(-j *1 * angle), exp(-j 0), exp(j *1 * angle), ..., exp(j *M * angle)]^T
 * @param angle the azimuthal angle
 * @return vectorXcd: the steering vector w.r.t. the given angle
 */
Eigen::VectorXcd MusicSpectrum::v(double angle) {
    Eigen::VectorXcd vectorXcd(2*_M + 1);

    int m = -_M;
    for(int i = 0; (i < 2*_M + 1); i ++){
        std::complex<double> z;
        z = std::polar(1.0, m*angle);
        vectorXcd(i) = z;
        m++;
    }

    return vectorXcd;
}

/**
 * creates the Bessel diagonal matrix, that is responsible for the elevation angle.
 * It looks like diag[J_M(angle), .. J_0(angle), ... J_M(angle)]
 * where J is the bessel function
 * @param angle
 * @return
 */
Eigen::MatrixXd MusicSpectrum::J(double angle){
    Eigen::VectorXd vectorXd(2*_M + 1);

    int m = -_M;
    for(int i = 0; i < (2*_M + 1); i++){
        vectorXd(i) = boost::math::cyl_bessel_j(std::abs(m), angle);
    }
    return vectorXd.asDiagonal();
}

/**
 * @param param : the certain parameter for this alpha function
 * @return the certain alpha value, needed for the W matrix
 */
double MusicSpectrum::alpha(double param){
    return 2 * M_PI * param / (2*_M + 1);
}