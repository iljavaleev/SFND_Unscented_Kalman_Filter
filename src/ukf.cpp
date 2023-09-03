#include "ukf.h"
#include "Eigen/Dense"
#include <cmath>
#include "Eigen/Core"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

  // initial state vector
    x_ = VectorXd(5);

  // initial covariance matrix
    P_ = Eigen::MatrixXd(5, 5);
    Q_ = Eigen::MatrixXd(2, 2);
    
    
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
    
    Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
    
  // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 2; // modify for bike
    time_us_ = 0.0;
    Q_ << pow(std_a_, 2), 0,
        0, pow(std_yawdd_, 2);
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
    R_rad_ = MatrixXd(3, 3);
    R_rad_ << pow(std_radr_, 2), 0, 0,
        0, pow(std_radphi_, 2), 0,
        0, 0, pow(std_radr_, 2);
    
    R_lid_ = MatrixXd(2, 2);
    R_lid_ << pow(std_laspx_, 2), 0,
          0, pow(std_laspy_, 2);
    
    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_(0) = lambda_/(lambda_ + n_aug_);
    
    for (int i{1}; i < 2 * n_aug_ + 1; i++){
        weights_(i) = 1/(2 * (lambda_ + n_aug_));
    }
  
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */
    if (!is_initialized_) {

        x_ << meas_package.raw_measurements_[0],
        meas_package.raw_measurements_[1],
        0,
        0,
        0;

        P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
        0, std_laspx_*std_laspx_, 0, 0, 0,
        0, 0, 10, 0, 0,
        0, 0, 0, 10, 0,
        0, 0, 0, 0, 10;

        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
//        return;
       
    }
    
    double delta{(meas_package.timestamp_ - time_us_) / 1000000.0};
        
    Prediction(delta);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        UpdateLidar(meas_package);
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
        UpdateRadar(meas_package);
    else
        return;
    
    time_us_ = meas_package.timestamp_;
}

void UKF::Prediction(double delta_t) {
  
    VectorXd x_aug(n_aug_);
    Eigen::MatrixXd P_aug(n_aug_, n_aug_);
    Eigen::MatrixXd Xsig_aug(n_aug_, 2 * n_aug_ + 1);
    
    x_aug.head(5) = x_;
            
    P_aug(Eigen::seq(0, 4), Eigen::seq(0, 4)) = P_;
    P_aug(Eigen::seq(5, 6), Eigen::seq(5, 6)) = Q_;
    
    Eigen::MatrixXd A = P_aug.llt().matrixL();
    
    Xsig_aug.col(0) = x_aug;
    
    for (int i = 0; i < n_aug_; ++i) {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
    }
    
    //create sigma points
       
    for (int i{}; i < 2 * n_aug_ + 1; i++){
        auto col = Xsig_aug.col(i);
        auto px{col(0,0)}, py{col(1, 0)}, v{col(2, 0)}, phi{col(3, 0)}, phi_hat{col(4, 0)}, nu_a{col(5, 0)}, nu_phi{col(6, 0)};
        
        VectorXd x_k(col(Eigen::seq(0, 4)));
        VectorXd part1{VectorXd(n_x_, 1)}, part2{VectorXd(n_x_, 1)};
        
        phi_hat += phi_hat * delta_t;
        if (phi_hat > 0.0001){
            part1 << px + (v/phi_hat) * (sin(phi + phi_hat * delta_t) - sin(phi)),
            py + (v/phi_hat) * (-cos(phi + phi_hat * delta_t) + cos(phi)),
            v,
            phi_hat,
            nu_phi;
        }else{
            part1 << px + v * delta_t * cos(phi),
            py + v * delta_t * sin(phi),
            v,
            phi_hat,
            nu_phi;
        }
        
        part2 << 0.5 * pow(delta_t, 2) * cos(phi) * nu_a,
                 0.5 * pow(delta_t, 2) * sin(phi) * nu_a,
                 delta_t * nu_a,
                 0.5 * pow(delta_t, 2) * nu_phi,
                 delta_t * nu_phi;
        
        Xsig_pred_.col(i) = x_k + part1 + part2;
        
    }
    
    // predicted mean and variance
    
    
    
    
    for (int i{}; i < 2 * n_aug_ + 1; i++){
        x_ += Xsig_pred_.col(i) * weights_(i);
    }
    
    
    for (int i{}; i < 2 * n_aug_ + 1; i++){
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
//        while (x_diff(3) > M_PI) {
//             x_diff(3) -= 2.0 * M_PI;
//           }
//        while (x_diff(3) < -M_PI) {
//             x_diff(3) += 2.0 * M_PI;
//           }
        P_ += weights_(i) * x_diff * x_diff.transpose();
    }
    
    
}
void UKF::UpdateHelper(MeasurementPackage meas_package, MatrixXd Zsig, bool lidar){
    VectorXd z_pred = VectorXd(n_z_);
    for (int i{}; i < 2 * n_aug_ + 1; i++){
        z_pred += Zsig.col(i) * weights_(i);
    }
    
    Eigen::MatrixXd S = MatrixXd(n_z_, n_z_);
    for (int i{}; i < 2 * n_aug_ + 1; i++){
        VectorXd z_diff{Zsig.col(i) - z_pred};
//
//        while (z_diff(1) > M_PI)
//            z_diff(1) -= 2. * M_PI;
//        while (z_diff(1) < -M_PI)
//            z_diff(1) += 2. * M_PI;
        
        S += weights_(i) * z_diff * z_diff.transpose();
    }
    
    if (lidar)
        S += R_lid_;
    else
        S += R_rad_;
    
    auto z = meas_package.raw_measurements_;
    
    Eigen::MatrixXd Tc{MatrixXd(n_x_, n_z_)};
      for (int i{}; i < 2 * n_aug_ + 1; i++){
          
          VectorXd z_diff{Zsig.col(i) - z_pred};
//          while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//          while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
//
          VectorXd x_diff{Xsig_pred_.col(i) - x_};
//          while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
//          while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

          Tc += weights_(i) * x_diff * z_diff.transpose();
      }
    
    auto K = Tc * S.inverse();
    
    x_ += K * (z - z_pred);
    P_ -= K * S * K.transpose();
    
    double nis = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
    UKF::niss.push_back(nis);
}


void UKF::UpdateLidar(MeasurementPackage meas_package) {
    n_z_ = 2;
    Eigen::MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
    for (int i{}; i < 2 * n_aug_ + 1; i++){
        auto x_sig_pred = Xsig_pred_.col(i);
        auto px{x_sig_pred(0)}, py{x_sig_pred(1)};
        Zsig.col(i) << px,
                    py;
        
    }
    UpdateHelper(meas_package, Zsig, true);
    
    
    MatrixXd H_ = MatrixXd(2, 5);
      H_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0;
      MatrixXd R_ = MatrixXd(2, 2);
      R_ << (std_laspx_*std_laspx_), 0,
            0, (std_laspy_*std_laspy_);

      VectorXd z_pred = H_ * x_;
      VectorXd y = meas_package.raw_measurements_ - z_pred;
      MatrixXd Ht = H_.transpose();
      MatrixXd S = H_ * P_ * Ht + R_;
      MatrixXd Si = S.inverse();
      MatrixXd PHt = P_ * Ht;
      MatrixXd K = PHt * Si;

      //new estimate
      x_ = x_ + (K * y);
      long x_size = x_.size();
      MatrixXd I = MatrixXd::Identity(x_size, x_size);
      P_ = (I - K * H_) * P_;
}


void UKF::UpdateRadar(MeasurementPackage meas_package) {
    n_z_ = 3;
    Eigen::MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
    for (int i{}; i < 2 * n_aug_ + 1; i++){
        auto x_sig_pred = Xsig_pred_.col(i);
        auto px{x_sig_pred(0)}, py{x_sig_pred(1)}, v{x_sig_pred(2)}, phi{x_sig_pred(3)};
        Zsig.col(i) << sqrt(pow(px, 2) + pow(py, 2)),
                    atan(px/py),
                    (px*cos(phi)*v + py*sin(phi)*v)/sqrt(pow(px, 2) + pow(py, 2));
    }

    UpdateHelper(meas_package, Zsig, false);
}


