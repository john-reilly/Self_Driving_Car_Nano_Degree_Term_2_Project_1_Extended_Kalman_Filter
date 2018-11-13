#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
//from section 10 lesson 5 here
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // from online Q+A video https://www.youtube.com/watch?list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2&v=J7WK9gEUltM
  H_laser_ << 1,0,0,0, 
    		0,1,0,0;
  
  ekf_.F_ = MatrixXd(4,4);// 4 * 4 Matrix (state transition)
  ekf_.F_ << 1,0,1,0,
  			0,1,0,1,
  			0,0,1,0,
            0,0,0,1;
  //from Lesson 5 Video 13 Quiz file: tracking.cpp
  //the initial transition matrix F_
//kf_.F_ = MatrixXd(4, 4);
//kf_.F_ << 1, 0, 1, 0,
//		  0, 1, 0, 1,
//		  0, 0, 1, 0,
//		  0, 0, 0, 1;   
    
  ekf_.P_ = MatrixXd(4,4); // 4 * 4 Matrix
  ekf_.P_ << 1,0,0,0,
            0,1,0,1,
            0,0,1000,0,
            0,0,0,1000;
//quiz 9 section 13 lesson 5 here
    	//state covariance matrix P
//	kf_.P_ = MatrixXd(4, 4);
//	kf_.P_ << 1, 0, 0, 0,
//			  0, 1, 0, 0,
//			  0, 0, 1000, 0,
//			  0, 0, 0, 1000;
  
  //Set the accleration noise compnents
  double noise_ax = 5 ; //9 in video 5 in quiz as per quiz 9 video 13 lesson 5
  double noise_ay = 5 ; //9 in video 5 in quiz as per quiz 9 video 13 lesson 5

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1; //as per video this is important for RMSE

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      
      //as per Q+A video just set ekf_.x_(0) to ro*cos(theta)
      //as per Q+A video just set ekf_.x_(1) to ro*sin(theta)
      // from Data file description Project Lesson 6
      // For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, 
      //rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
      float rho = measurement_pack.raw_measurements_(0) ;
      float theta = measurement_pack.raw_measurements_(1) ;//theta is called phi_measured in lesson 6
      //float rhodot = measurement_pack.raw_measurements_(2) ;
      
      ekf_.x_(0) = rho*cos(theta);
      ekf_.x_(1) = rho*sin(theta);
      
      
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0) ; //I thought this was 1 and 2 for below but first word in package "sensor type" is parsed off and first data is now position 0
      ekf_.x_(1) = measurement_pack.raw_measurements_(1) ;
      //from lesson 6 For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
    }
    //ekf_.F is set to << 1 diagonal matirx // as per Q+A
    //previous timestamp is measurement_package.timestamp // as per Q+A
    //from Lesson 5 video 13 quiz
    	//the initial transition matrix F_
	//kf_.F_ = MatrixXd(4, 4);
	//kf_.F_ << 1, 0, 1, 0,
	//		  0, 1, 0, 1,
	//		  0, 0, 1, 0,
	//		  0, 0, 0, 1;
    ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
