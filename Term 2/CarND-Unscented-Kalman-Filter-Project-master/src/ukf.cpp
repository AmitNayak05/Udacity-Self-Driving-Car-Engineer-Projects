#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  P_ = MatrixXd(5, 5);
  P_ << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
	  -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
	  0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
	  -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
	  -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

  x_pred = VectorXd(5);
  x_pred.fill(0.0);
  //create covariance matrix for prediction
  P_pred = MatrixXd(5, 5);
  P_pred.fill(0.0);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

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

  //setting state Dimensions
  n_x_ = 5;

  //setting augmentation dimensions
  n_aug_ = 7;

  //lambda
  lambda_ = 3 - n_x_;

  //lambda augmented
  lambda_aug_ = 3 - n_aug_;

  //Augmented state
  x_aug_ = VectorXd(n_aug_);


  //Augmented covariance
  P_aug_= MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);



  //x sigma prediction
  Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  

  n_z_radar = 3;
  n_z_laser = 2;


  is_initialized_ = false;

  previous_timestamp_ = 0;
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

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
		x_ << 1, 1, 1, 1,1;

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			//std::cout << "measurent radar " << measurement_pack.raw_measurements_[0] << "  " << measurement_pack.raw_measurements_[1] << " " << measurement_pack.raw_measurements_[2] << endl;
			double x_radar = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
			double y_radar = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
			double v_x = meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]);
			double v_y = meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]);
			//std::cout << "radar x= " << x_radar << " y= " << y_radar << " vx= " << v_x << " vy=" << v_y << endl;
			x_ << x_radar, y_radar, sqrt((v_x*v_x)+ (v_y*v_y)),0,0;

		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/
			std::cout << "Unscented Kalman filter laser" << endl;
			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0,0;

		}

		// done initializing, no need to predict or update
		is_initialized_ = true;
		previous_timestamp_ = meas_package.timestamp_;
		//std::cout << "timestamp= =" << previous_timestamp_ << endl;
		cout << "Initialized first time" << endl;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/
	float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds

	previous_timestamp_ = meas_package.timestamp_;


	Prediction(dt);

	/*****************************************************************************
	*  Update
	****************************************************************************/

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		UpdateRadar(meas_package);
	}
	else {
		// Laser updates
		UpdateLidar(meas_package);
	}


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	/**
	TODO:

	Complete this function! Estimate the object's location. Modify the state
	vector, x_. Predict sigma points, the state, and the state covariance matrix.
	*/
	x_aug_.head(5) = x_;
	x_aug_(5) = 0;
	x_aug_(6) = 0;

	P_aug_.fill(0.0);
	P_aug_.topLeftCorner(5, 5) = P_;
	P_aug_(5, 5) = std_a_*std_a_;
	P_aug_(6, 6) = std_yawdd_*std_yawdd_;

	MatrixXd Xsig_aug;

	AugmentedSigmaPoints(&Xsig_aug);

	for (int i = 0; i< 2 * n_aug_ + 1; i++)
	{
		//extract values for better readability
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
		}
		else {
			px_p = p_x + v*delta_t*cos(yaw);
			py_p = p_y + v*delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd*delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
		v_p = v_p + nu_a*delta_t;

		yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
		yawd_p = yawd_p + nu_yawdd*delta_t;

		//write predicted sigma point into right column
		Xsig_pred(0, i) = px_p;
		Xsig_pred(1, i) = py_p;
		Xsig_pred(2, i) = v_p;
		Xsig_pred(3, i) = yaw_p;
		Xsig_pred(4, i) = yawd_p;
	}

	//create vector for weights
	VectorXd weights = VectorXd(2 * n_aug_ + 1);


	// set weights
	double weight_0 = lambda_aug_ / (lambda_aug_ + n_aug_);
	weights(0) = weight_0;
	for (int i = 1; i<2 * n_aug_ + 1; i++) {  //2n+1 weights
		double weight = 0.5 / (n_aug_ + lambda_aug_);
		weights(i) = weight;
	}

	//predicted state mean
	
	x_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
		x_ = x_ + weights(i) * Xsig_pred.col(i);
	}
	
		//predicted state covariance matrix
		P_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

											   // state difference
		VectorXd x_diff = Xsig_pred.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		P_ = P_ + weights(i) * x_diff * x_diff.transpose();
	}
	
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
	
	VectorXd weights = VectorXd(2 * n_aug_ + 1);
	double weight_0 = lambda_aug_ / (lambda_aug_ + n_aug_);
	weights(0) = weight_0;
	for (int i = 1; i<2 * n_aug_ + 1; i++) {
		double weight = 0.5 / (n_aug_ + lambda_aug_);
		weights(i) = weight;
	}


	MatrixXd Zsig = MatrixXd(n_z_laser, 2 * n_aug_ + 1);
	Zsig.fill(0.0);
	//transform sigma points into measurement space
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

											   // extract values for better readibility
		double p_x = Xsig_pred(0, i);
		double p_y = Xsig_pred(1, i);
		double v = Xsig_pred(2, i);
		double yaw = Xsig_pred(3, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		Zsig(0, i) = p_x;                        //r
		Zsig(1, i) = p_y;                                 //phi
		//Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
	}

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z_laser);
	z_pred.fill(0.0);
	z_pred = Zsig * weights;
	
	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z_laser, n_z_laser);
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
											   //residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		S = S + weights(i) * z_diff * z_diff.transpose();
	}

	
	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z_laser, n_z_laser);
	R << std_laspx_*std_laspx_, 0,
		0, std_laspy_*std_laspy_;

	S = S + R;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z_laser);
	Tc.fill(0.0);
	
	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

											   //residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		// state difference
		VectorXd x_diff = Xsig_pred.col(i) - x_;
		
		Tc = Tc + weights(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	VectorXd z = VectorXd(n_z_laser);
	//values from measurement pack
	z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

	//residual
	VectorXd z_diff = z - z_pred;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S*K.transpose();
	NIS_laser_ = z_diff.transpose()*S.inverse()*z_diff;
	//cout << "NIS_laser_ = " << NIS_laser_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

	//cout << "in update radar" << endl;


	VectorXd weights = VectorXd(2 * n_aug_ + 1);
	double weight_0 = lambda_aug_ / (lambda_aug_ + n_aug_);
	weights(0) = weight_0;
	for (int i = 1; i<2 * n_aug_ + 1; i++) {
		double weight = 0.5 / (n_aug_ + lambda_aug_);
		weights(i) = weight;
	}


	MatrixXd Zsig = MatrixXd(n_z_radar, 2 * n_aug_ + 1); 
	Zsig.fill(0.0);

	//transform sigma points into measurement space
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

												// extract values for better readibility
		double p_x = Xsig_pred(0, i);
		double p_y = Xsig_pred(1, i);
		double v = Xsig_pred(2, i);
		double yaw = Xsig_pred(3, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		if (fabs(p_x) < 0.001) {
			p_x = 0.001;
		}
		if (fabs(p_y) < 0.001) {
			p_y = 0.001;
		}

		// measurement model
		Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                        //r
		Zsig(1, i) = atan2(p_y, p_x);                                 //phi
		Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
	}

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z_radar);
	z_pred.fill(0.0);
	z_pred = Zsig * weights;

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z_radar, n_z_radar);
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
												//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

		S = S + weights(i) * z_diff * z_diff.transpose();
	}
	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z_radar, n_z_radar);
	R << std_radr_*std_radr_, 0, 0,
		0, std_radphi_*std_radphi_, 0,
		0, 0, std_radrd_*std_radrd_;

	S = S + R;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z_radar);

	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

												//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		Tc = Tc + weights(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	VectorXd z = VectorXd(n_z_radar);

	//values from measurement pack
	z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

	//residual
	VectorXd z_diff = z - z_pred;

	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S*K.transpose();

	NIS_radar_ = z_diff.transpose()*S.inverse()*z_diff;
	//cout << "NIS_radar_ = " << NIS_radar_ << endl;
}
/**
* GenerateSigmaPoints
* Find sigma Points
*/
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {
	MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

	//calculate square root of P
	MatrixXd A = P_.llt().matrixL();

	//set sigma points as columns of matrix Xsig
	Xsig.col(0) = x_;
	for (int i = 0; i < n_x_; i++)
	{
		Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
		Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
	}

	*Xsig_out = Xsig;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	MatrixXd L = P_aug_.llt().matrixL();
	//create augmented sigma points
	Xsig_aug.col(0) = x_aug_;
	for (int i = 0; i< n_aug_; i++)
	{
		Xsig_aug.col(i + 1) = x_aug_ + sqrt(lambda_aug_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_aug_ + n_aug_) * L.col(i);
	}
	*Xsig_out = Xsig_aug;
}