#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  
  // from Q+A video
  x_ = F_ * x_ ; //section 8 lesson 5
  MatrixXd Ft_ = F_.transpose(); //section 9 lesson 5 I note in the Q+A he had Ft no underscore I added one for consistency
  P_ = F_ * P_ * Ft_ + Q_ ;   // from Q+A video
  
  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  //// from Q+A video // In section 7 of lesson 5
  VectorXd y_ = z - H_ * x_ ; // I had z_ but changed to z as per paramete (const VectorXd &z)
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht_ + R_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ = P_ * Ht_ * Si_ ;
  
  MatrixXd I_ ; // Identity matrix from quix section 7 of lesson 2
  
  //New State // from Q+A video // In section 7 of lesson 5
  I_ = MatrixXd::Identity(2, 2); //from quiz section 7 of lesson 5 2,2 might be wrong check...
  x_ = x_ + (K_ * y_);
  P_ = ( I_ - K_ * H_ ) * P_;
  //below was added in error from video then deleted....I might use it ...
 // //KF prediction step // from Q+A video // In section 7 of lesson 5
  //x_ = F_ * x_ + u_ ;
  //MatrixXd  Ft_ = F_.transpose() ;
  //P_ = F_ * P_ * Ft_ + Q_ ;
  
  
  
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
