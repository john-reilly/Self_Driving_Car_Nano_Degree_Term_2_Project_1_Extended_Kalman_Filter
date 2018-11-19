#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  
  MatrixXd I_ ; // Identity matrix from quiz section 7 of lesson 2 not mentioned in video
  
  //New State // from Q+A video // In section 7 of lesson 5
  I_ = MatrixXd::Identity(4,4); //from quiz section 7 of lesson 5 2,2 might be wrong check... yes caused gried 4,4 correct 
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
  //cout <<" Debug print out: UpdateEKF line 70 "<< endl;
  //section 14 of lesson 5
  float px_ = x_(0);// changed this from Q+A x already used and px actual name// float x = ekf_.x_(0);
  float py_ = x_(1);// changed this from Q+A // y already used and py actual name//float y = ekf_.x_(1);
  float vx_ = x_(2);// changed this from Q+A // //float vx = ekf_.x_(2);
  float vy_ = x_(3);// changed this from Q+A // //float vy = ekf_.x_(3);
  
  //cout <<" Debug print out: UpdateEKF line 77: after px py "<< endl;
  float rho =  sqrt( px_ * px_ + py_ * py_ ) ;
  float theta = atan2(py_,px_);
  float rho_dot =  (px_ * vx_ + py_ * vy_  ) / rho ;
  VectorXd z_predict = VectorXd(3);
  z_predict << rho,theta,rho_dot ;
  //cout <<" Debug print out: UpdateEKF line 83: after z_predict rho,theta,rho_dot "<< endl;
  VectorXd y_ = z - z_predict ;
    // from Slack channel about pi   ..implenmts normailisting angles from Tips & Tricks section 7
    //while(y(1) > M_PI) { y(1) -= M_PI; }
    //while(y(1) < -M_PI) { y(1) += M_PI; }
  //cout <<" Debug print out: UpdateEKF line 90: before M_PI NORMALISATION "<< endl;
  while(y_(1) > M_PI) { y_(1) -= M_PI; }
  while(y_(1) < -M_PI) { y_(1) += M_PI; }
  //cout <<" Debug print out: UpdateEKF line 93: after M_PI NORMALISATION "<< endl;
  
  //cout <<" Debug print out: UpdateEKF line 85: y_ "<< endl;
    //section 8 of lesson 5 Q+A say section 7 not 8 but that is Quiz and 8 is answer to quiz
 // VectorXd y = z - H * x; // this line in quiz answer but not Q+A video
  // underscores added for consitency
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht_ + R_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ =  P_ * Ht_ * Si_;
  //cout <<" Debug print out: UpdateEKF line 93: before I_ matrix "<< endl;//crash just after this line
  MatrixXd I_ ; // similar to line 48 // Identity matrix from quiz section 7 of lesson 2 not mentioned in video
  I_ = MatrixXd::Identity(4,4); //from quiz ssection 8 lesson 5.  (2,2) might need to changed to 4,4

  //new state //section 8 of lesson 5 Q+A say section 7 not 8 but that is Quiz and 8 is answer to quiz
  x_ = x_ + (K_ * y_);
  P_ = (I_ - K_ * H_) * P_;
  //cout <<" Debug print out: UpdateEKF line 100: last line in method "<< endl;
  
}
