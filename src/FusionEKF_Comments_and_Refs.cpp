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
 // double noise_ax = 9 ; // was 5 set to nine as per video  9 in video 5 in quiz as per quiz 9 video 13 lesson 5
 // double noise_ay = 9 ; //was 5 set to nine as per video 9 in video 5 in quiz as per quiz 9 video 13 lesson 5

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
   // cout <<"Debug print out: line 93 start of initalisation" << endl;
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 0,0,0,0 ;//changing to 0 to see effect //ekf_.x_ << 1, 1, 1, 1; //as per video this is important for RMSE
	//ekf_.x_ << 5,5,5,5 ;
   // ekf_.x_ << 1,1,1,1 ;
     //ekf_.x_ << 1,1,5,0 ; //best so far
    //ekf_.x_ << 0,0,5,0 ; // same as 1 1 5 0 as 1 1 gets written over below but wanted to check anyway
    //ekf_.x_ << 0,0,5,1 ; //slightly worse than 1 1 5 0
    //ekf_.x_ << 0,0,6,0 ; // tiny bit better in one worse in another
    ekf_.x_ << 0,0,5.5,0 ;//tiny bit better than 0 0 6 0
    
    
    
    
    
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
     // cout <<"Debug print out: line 100 start of radar section" << endl;
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
      //from tips and tricks section Although radar gives velocity data in the form of the range rate ρ˙\dot{\rho}ρ˙​, a radar measurement does not contain enough information to determine the state variable velocities vxv_xvx​ and vyv_yvy​. You can, however, use the radar measurements ρ\rhoρ and ϕ\phiϕ to initialize the state variable locations pxp_xpx​ and pyp_ypy​.
      ekf_.x_(0) = rho*cos(theta);
      ekf_.x_(1) = rho*sin(theta);
      //ekf_.x_(2) = rhodot*cos(theta);//experiment
      //ekf_.x_(3) = rhodot*sin(theta);// this and above line made no difference as per hint suggestion leaving both out
      
    //  cout <<"Debug print out: line 117 end of radar section" << endl;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
          //  cout <<"Debug print out: line 124 start of lidar section" << endl;
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
   // cout <<"Debug print out: line 146 end of lidar section" << endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
// cout <<"Debug print out: line 153 start of predict" << endl;
  // as per Q+A video
  float dt = (measurement_pack.timestamp_ - previous_timestamp_ ) / 1000000.0; // dt expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_ ;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  // as per Q+A video
  //Modify the F matrix so that the time is integrated section 8 of lesson 5
  ekf_.F_(0 , 2) = dt ;
  ekf_.F_(1 , 3) = dt ;
  
  //set the process covariance matrix Q Section 9 of lesson 5
  // I got error about noise_ax and noise_ay not been delared in this scope so added below delarations
  double noise_ax = 9 ; //changef to 9 was 5 ....9 in video 5 in quiz as per quiz 9 video 13 lesson 5
  double noise_ay = 9;//changef to 9 was 5  ....9 in video 5 in quiz as per quiz 9 video 13 lesson 5
  
  ekf_.Q_ = MatrixXd(4,4) ;
  ekf_.Q_ << dt_4 / 4 * noise_ax , 0 , dt_3/2 * noise_ax ,0,
  			0, dt_4/4 * noise_ay , 0 , dt_3/2 * noise_ay,
  			dt_3/2 * noise_ax, 0 , dt_2 * noise_ax, 0 ,
  			0 , dt_3/2 * noise_ay, 0 , dt_2 *  noise_ay;
  
//cout <<"Debug print out: line 177  end of predict" << endl;
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
//cout <<"Debug print out: line 200 start of update before if statement" << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
   // cout <<"Debug print out: line 202  if statement== RADAR" << endl;
    // Radar updates
    // as per Q+A video
    //set ekf_.H_ by setting to Hj which is the calculated Jackobian
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_); // finish calcute jackobian ..done
    //set ekf_.R_  by just using R_radar
    ekf_.R_ = R_radar_ ;
    
    
    // from Slack channel about pi   somewhere around here //I am puttin gthis idea into updateEKF()
    //while(y(1) > M_PI) { y(1) -= M_PI; }
    //while(y(1) < -M_PI) { y(1) += M_PI; }
     // cout <<"Debug print out: line 214  end if statement== RADAR" << endl; //got this far in test
    //ekf_.UpdateEKF( measurement_pack.raw_measurements_); // as per Q+A video
     ekf_.UpdateEKF( measurement_pack.raw_measurements_); //bug fixed had updateEKF here
    
    
  } else {
    //cout <<"Debug print out: line 220 else if statement== laser" << endl;
    // Laser updates // as per Q+A video
    //set ekf_.H_  by just using H_laser
     ekf_.H_ = H_laser_;
    //set ekf_.R_  by just using R_laser // as per Q+A video
     ekf_.R_ = R_laser_ ;
   // cout <<"Debug print out: line 226 end  if statement== laser" << endl;
    //ekf_.UpdateEKF( measurement_pack.raw measurements_); // as per Q+A video
    ekf_.Update( measurement_pack.raw_measurements_); //change line 211 by mistake...
    
    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  
  
}
