#include "ukf.h"
#include "Eigen/Dense"
#include<iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.5;
  
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
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  // set state dimension
  n_x_ = 5;

  // set augmented dimension
  n_aug_ = 7;

  // define spreading parameter
  lambda_ = 3 - n_aug_;
  
  x_ = VectorXd(n_x_);
  
  P_ = MatrixXd(n_x_, n_x_);
  
  // create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(this->n_x_, 2 * (this->n_aug_) + 1);
  
  // create vector for weights
  weights_ = VectorXd(2*(this->n_aug_)+1);
  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; ++i) {  // 2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
  {
    if (this->use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) 
    {
      std::cout << "Kalman Filter Initialization... " << std::endl;
      std::cout << "First package is from LASER. " << std::endl;

      // set the state with the initial location and zero velocity
      this->x_ << meas_package.raw_measurements_[0], 
                meas_package.raw_measurements_[1], 
                0, 
                ((meas_package.raw_measurements_[0])==0) ? (M_PI/2) : atan(meas_package.raw_measurements_[1]/meas_package.raw_measurements_[0]), 
                0;
      this->P_ << 0.1000,   -0.0000,    0.0000,   -0.1000,   -0.1000,
                 -0.0000,    0.1000,    0.0000,    0.1000,    0.1000,
                  0.0000,    0.0000,    0.1000,    0.0000,    0.0000,
                 -0.1000,    0.1000,    0.0000,    0.1000,    0.1000,
                 -0.1000,    0.1000,    0.0008,    0.1000,    0.1000;
      
      this->Prediction(0.);

      this->time_us_ = meas_package.timestamp_;
      this->is_initialized_ = true;
      std::cout << "DONE! " << std::endl;

      this->UpdateLidar(meas_package);
    }
    if (this->use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
      std::cout << "Kalman Filter Initialization... " << std::endl;
      std::cout << "First package is from RADAR. " << std::endl;

      // set the state with the initial location and zero velocity
      this->x_ << (meas_package.raw_measurements_[0])*cos(meas_package.raw_measurements_[1]), 
                (meas_package.raw_measurements_[0])*sin(meas_package.raw_measurements_[1]), 
                meas_package.raw_measurements_[2], 
                meas_package.raw_measurements_[1], 
                0;
      this->P_ << 0.1000,   -0.0000,    0.0000,   -0.1000,   -0.1000,
                 -0.0000,    0.1000,    0.0000,    0.1000,    0.1000,
                  0.0000,    0.0000,    0.1000,    0.0000,    0.0000,
                 -0.1000,    0.1000,    0.0000,    0.1000,    0.1000,
                 -0.1000,    0.1000,    0.0008,    0.1000,    0.1000;
      
      this->Prediction(0.);

      this->time_us_ = meas_package.timestamp_;
      this->is_initialized_ = true;
      std::cout << "DONE! " << std::endl;

      this->UpdateRadar(meas_package);
    }
    
    return;
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (meas_package.timestamp_ - this->time_us_)/1e6;
  this->time_us_ = meas_package.timestamp_;

  // predict
  if (dt > 0)
  {
    this->Prediction(dt);
  }

  // measurement update
  if (this->use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) 
  {
    this->UpdateLidar(meas_package);
  }
  if (this->use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) 
  {
    this->UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  {
    // GenerateSigmaPoints
    // set state dimension
      // create augmented mean vector
    VectorXd x_aug = VectorXd(this->n_aug_);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(this->n_aug_, this->n_aug_);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(this->n_aug_, 2 * (this->n_aug_) + 1);

    /**
     * Student part begin
     */
 
    // create augmented mean state
    x_aug.head(this->n_x_) = this->x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(this->n_x_,this->n_x_) = this->P_;
    P_aug(5,5) = (this->std_a_)*(this->std_a_);
    P_aug(6,6) = (this->std_yawdd_)*(this->std_yawdd_);

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< (this->n_aug_); ++i) {
      Xsig_aug.col(i+1)       = x_aug + sqrt((this->lambda_)+(this->n_aug_)) * L.col(i);
      Xsig_aug.col(i+1+(this->n_aug_)) = x_aug - sqrt((this->lambda_)+(this->n_aug_)) * L.col(i);
    }
  
    // SigmaPointPrediction

    /**
     * Student part begin
     */

    // predict sigma points
    for (int i = 0; i< 2*(this->n_aug_)+1; ++i) {
      // extract values for better readability
      double p_x = Xsig_aug(0,i);
      double p_y = Xsig_aug(1,i);
      double v = Xsig_aug(2,i);
      double yaw = Xsig_aug(3,i);
      double yawd = Xsig_aug(4,i);
      double nu_a = Xsig_aug(5,i);
      double nu_yawdd = Xsig_aug(6,i);

      // predicted state values
      double px_p, py_p;

      // avoid division by zero
      if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
      } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
      }

      double v_p = v;
      double yaw_p = yaw + yawd*delta_t;
      double yawd_p = yawd;

      // add noise
      px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
      py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
      v_p = v_p + nu_a*delta_t;

      yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
      yawd_p = yawd_p + nu_yawdd*delta_t;

      // write predicted sigma point into right column
      (this->Xsig_pred_)(0,i) = px_p;
      (this->Xsig_pred_)(1,i) = py_p;
      (this->Xsig_pred_)(2,i) = v_p;
      (this->Xsig_pred_)(3,i) = yaw_p;
      (this->Xsig_pred_)(4,i) = yawd_p;
    }
  
  
  
    // PredictMeanAndCovariance

    /**
     * Student part begin
     */

    // predicted state mean
    (this->x_).fill(0.0);
    for (int i = 0; i < 2 * (this->n_aug_) + 1; ++i) {  // iterate over sigma points
      this->x_ = (this->x_) + (this->weights_)(i) * (this->Xsig_pred_).col(i);
    }

    // predicted state covariance matrix
    (this->P_).fill(0.0);
    for (int i = 0; i < 2 * (this->n_aug_) + 1; ++i) {  // iterate over sigma points
      // state difference
      VectorXd x_diff = (this->Xsig_pred_).col(i) - (this->x_);
      // angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      (this->P_) = (this->P_) + (this->weights_)(i) * x_diff * x_diff.transpose() ;
    }
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // TODO: YOUR CODE HERE
  MatrixXd H = MatrixXd(2, this->n_x_);
  H << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;
  
  MatrixXd R = MatrixXd(2, 2);
  R << (this->std_laspx_)*(this->std_laspx_),                                     0,
                                           0, (this->std_laspy_)*(this->std_laspy_);
  
  VectorXd z_pred = H * (this->x_);
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * (this->P_) * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = (this->P_) * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  this->x_ = (this->x_) + (K * y);
  long x_size = (this->x_).size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  this->P_ = (I - K * H) * (this->P_);
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  int n_z = 3;
  
  // PredictRadarMeasurement
    // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * (this->n_aug_) + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * (this->n_aug_) + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = (this->Xsig_pred_)(0,i);
    double p_y = (this->Xsig_pred_)(1,i);
    double v  = (this->Xsig_pred_)(2,i);
    double yaw = (this->Xsig_pred_)(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*(this->n_aug_)+1; ++i) {
    z_pred = z_pred + (this->weights_)(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * (this->n_aug_) + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + (this->weights_)(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  (this->std_radr_)*(this->std_radr_),                                       0,                                    0,
                                          0, (this->std_radphi_)*(this->std_radphi_),                                    0,
                                          0,                                       0,(this->std_radrd_)*(this->std_radrd_);
  S = S + R;
  
  
  
  // UpdateState
    // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd((this->n_x_), n_z);


  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * (this->n_aug_) + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = (this->Xsig_pred_).col(i) - (this->x_);
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + (this->weights_)(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  this->x_ = (this->x_) + K * z_diff;
  this->P_ = (this->P_) - K*S*K.transpose();
}