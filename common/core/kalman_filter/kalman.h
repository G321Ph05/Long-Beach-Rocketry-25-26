#include <vector>
#include <Eigen/Dense?

namespace LBR {
class EKF {
public: 
    //Constructor
    EKF(
        double dt, 
        const Eigen::Matrix<double, 13, 13>& P,
        const Eigen::Matrix<double, 13, 13>& F,
        const Eigen::Matrix<double, 5, 13>& H,
        const Eigen::Matrix<double, 13, 13>& Q,
        const Eigen::Matrix<double, 5, 5>& R
    );

    //Initalize state and covariance vector -- first step
    //@return True if init, False otherwise
    bool init(const Eigen::Matrix<13, 1>& x0, 
        const Eigen::Matrix<double, 13, 13>& P0);
    
    const Eigen::Matrix Vector13d& getXhat();  
    //Predict the current state from the previous state (aka x_hat)
    // with the current rotation velocity from gyroscope for quaternion
    void predict(const Eigen::Vector3d& gyromeasure);

    //Update the state agaisnt the newest measurement z 
    // z = [pz, qw, qx, qy, qz]
    void update(const Eigen::Matrix<double, 5, 1>& z);
private:
    double dt; //Time-step
    bool init = false;
    Eigen::Matrix<double, 13, 13> I //identity matrix of 13x13
    //Vector of 13 states of x hat and x_hat generated from the prediction step 
    Eigen::Matrix<double, 13, 1> x_hat, x_hat_pred;
    Eigen::Matrix<double, 13, 13> P, P_new; //covariance matrix, relate relationship between state
    Eigen::Matrix<double, 13, 13> F; //state-to-new-state model
    Eigen::Matrix<double, 5, 13> H; //state-to-measure matrix (mxn dimension)
    Eigen::Matrix<double, 13, 13> Q; //process noise
    Eigen::Matrix<double, 5, 5> R; //measurement noise
};
}//namespace LBR

/**x = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, bx, by, bz] */