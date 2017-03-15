#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

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
    0, 0.09, 0,
    0, 0, 0.09;
  /**
  TODO:
    * Finish initializing the FusionEKF.
  */
  ekf_.x_ = VectorXd(4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1000, 0,
			0, 0, 0, 1000;

  ekf_.R_ = MatrixXd(2, 2);
  ekf_.R_ << 0.0225, 0,
	  0, 0.0225;


  ekf_.R2_ = MatrixXd(3, 3);
  ekf_.R2_ << 0.09, 0, 0,
    0, 0.09, 0,
    0, 0, 0.09;

  //measurement matrix
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0,
	  0, 1, 0, 0;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	  0, 1, 0, 1,
	  0, 0, 1, 0,
	  0, 0, 0, 1;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

  noise_r=5;
  noise_theta=5;
  noise_rdot=5;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
	//std::cout << "process measurement function" << endl;
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //std::cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

		/**
		Convert radar from polar to cartesian coordinates and initialize state.
		*/
		//std::cout << "measurent radar " << measurement_pack.raw_measurements_[0] << "  " << measurement_pack.raw_measurements_[1] << " " << measurement_pack.raw_measurements_[2] << endl;
		double x_radar = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
		double y_radar= measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
		double v_x= measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
		double v_y = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
		//std::cout << "radar x= " << x_radar << " y= " << y_radar << " vx= " << v_x << " vy=" << v_y << endl;
		ekf_.x_ << x_radar, y_radar, v_x, v_y;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		std::cout << "Kalman filter laser" << endl;
		ekf_.x_<< measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
	previous_timestamp_ = measurement_pack.timestamp_;
	//std::cout << "timestamp= =" << previous_timestamp_ << endl;
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
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  //std::cout <<"time difference= "<< dt << endl;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
    0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
    dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
    0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;
  //cout << "before predict" << endl;

  
  
    ekf_.Predict();
  
  
  //cout << "after prediction" << endl;
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
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //std::cout << "x_ = " << ekf_.x_ << endl;
  //std::cout << "P_ = " << ekf_.P_ << endl;
}
