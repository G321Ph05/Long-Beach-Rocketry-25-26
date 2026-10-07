#include "kalman.h"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

namespace LBR 
{
EKF::EKF(double dt, 
        const Matrix<double, 13, 13>& P,
        const Matrix<double, 13, 13>& F,
        const Matrix<double, 5, 13>& H,
        const Matrix<double, 13, 13>& Q,
        const Matrix<double, 5, 5>& R
    )
{
    this->dt = dt;
    this->P = P;
    this->F = F;
    this->H = H;
    this->Q = Q;
    this->R = R;

    x_hat.setZero();
    x_hat(6) = 1.0; //qw = 1 for a standing still rocket pointing upward at 90degree
    x_hat_pred.setZero();
    x_hat_new(6) = 1.0;
    I.setIdentity();
}

Eigen::Matrix Vector13d& getXhat const(){
    return x_hat;
};


void EKF::init(const Matrix<double, 13, 1>& x0, 
        const Matrix<double, 13, 13>& P0){
    x_hat = x0;
    x_hat_pred = x0;
    P = P0;
    P_new = P0;
    init = true;
}

void EKF::predict(const Vector3d& gyromeasure){
    if (!init) return; //Check whether x and P has been initliazed

    //Angular velocity minus the gyroscope bias 
    Vector3d omega = gyromeasure - x_hat.segment<3>(10);
    double wx, wy, wz; 
    wx = omega(0);
    wy = omega(1);
    wz = omega(2);
    //Define a matrix that transform qk -> qk+1 use omega
    Matrix<double, 4, 4> transform;
    transform << 0, -wx, -wy, -wz,
                wx, 0, wz, -wy,
                wy, -wz, 0, wx,
                wz, wy, -wx, 0;

    const double alpha = dt / 2;
    const Matrix<double, 4, 4> I4 = Matrix4d::Identity();

    //Convert matrix for quaternion
    // q = q * (I + dt/2 * transform)
    // A  = (I + dt/2 * transform)
    Matrix<double, 4, 4> Aq;
    Aq = I4 + alpha * transform;

    Vector4d q = x_hat.segment<4>(6);

    //Construct a matrix4d to use when computing
    //Jacobian term for Jqb, use q at time k
    double qw, qx, qy, qz;
    qw = q(0);
    qx = q(1); 
    qy = q(2); 
    qz = q(3);
    Matrix<double, 4, 3> B;
    B << qx, qy, qz,
        -qw, qz, -qy,
        -qz, -qw, qx,
        qy, -qx, -qw;
    B *= alpha;

    //Compute new q (raw + normalized)
    q = Aq * q;
    //Compute r using the raw quaternion before normalization
    const double r = q.norm();
    q.normalize();

    //F is identity matrix, except 
    // F(0, 3), F(1, 4), F(2, 5) = dt --> Position relate to velocity
    //Will be changed later with Jqq and Jqb
    x_hat_pred = F * x_hat;
    x_hat_pred.segment<4>(6) = q; //Append q_new_value to prediction

    //Construct Jacobian matric for Jqq and Jqb
    //Jqq: use normalized q
    //Jqb: use raw q + normalized q
    const Matrix<double, 4, 4> N = 
        1/r * (I4 - q * q.transpose());
    
    const Matrix<double, 4, 4> Jqq = N * Aq; //Relationship between qK+1 / qk
    const Matrix<double, 4, 3> Jqb = N * B; //Relationship between qK+1 / b (gyro-bias)

    //Jacobian matrix
    Matrix<double, 13, 13> J;
    J.setIdentity();

    J.block<3, 3>(0, 3) = dt * Matrix3d::Identity();
    J.block<4, 4>(6, 6) = Jqq;
    J.block<4, 3>(6, 10) = Jqb;

    //Covariance computation
    P_new = J * P * J.transpose() + Q;
}

void EKF::update(const Eigen::Matrix<double, 5, 1>& z){
    if (!init) return;

    //Kalman gain
    const Matrix<double, 13, 5> K = P_new * H.transpose() 
                    * (H * P_new * H.transpose() + R).inverse();
    
    x_hat = x_hat_pred + K * (z - H * x_hat_pred);
    P = (I - K * H) * P_new;
}
}