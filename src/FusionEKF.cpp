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
FusionEKF::FusionEKF() 
{
  is_initialized_ = false;
  Tools tools;
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

  // initializing the process and measurement noises
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
			 0, 1, 0, 0,
			 0, 0, 1000, 0,
			 0, 0, 0, 1000;

  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

	VectorXd test = VectorXd(4);
	test << -5.29137, 0.220292, -2.01086, -4.86364;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) 
  {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

	//Create state covariance matrix
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	//initial transition matrix
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			   0, 1, 0, 1,
			   0, 0, 1, 0,
			   0, 0, 0, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
	{
		cout << "EKF : First measurement- RADAR" << endl;
		double rho = measurement_pack.raw_measurements_(0);
		double phi = measurement_pack.raw_measurements_(1);
		double rhodot = measurement_pack.raw_measurements_(2);

		double x = rho * cos(phi);
		double y = rho * sin(phi);
		double vx = rhodot * cos(phi);
		double vy = rhodot * sin(phi);

		// polar to cartesian - r * cos(angle) for x and r * sin(angle) for y
		ekf_.x_ << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
	{
		cout << "EKF : First measurement- LASER" << endl;
		ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0.0, 0.0;
    }

	// Saving first timestamp in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   * Prediction:
   * Update the state transition matrix F according to the new elapsed time.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   ****************************************************************************/

   //compute the time elapsed between the current and previous measurements
   //dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the State transition matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //process noise
  float noise_ax = 9;
  float noise_ay = 9;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
	         0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
	         dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
	         0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   * Update:
   * Update the state and covariance matrices by the sensor type
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
	  // Radar updates
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else 
  {
	  // Laser updates
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
