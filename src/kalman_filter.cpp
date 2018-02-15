#include "kalman_filter.h"
#define _USE_MATH_DEFINES

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
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_; // 2x1
	VectorXd y = z - z_pred; // 2x1
	MatrixXd Ht = H_.transpose(); // 4x2
	MatrixXd S = H_ * P_ * Ht + R_; // 2x2
	MatrixXd Si = S.inverse(); // 2x2
	MatrixXd PHt = P_ * Ht; // 4x2
	MatrixXd K = PHt * Si; // 4x2

	//new estimate
	x_ = x_ + (K * y); // 4x1
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
	  * update the state by using Extended Kalman Filter equations
	*/
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	float rho = sqrt(px*px + py*py);
	float phi = atan2(py, px);
	float rho_dot;
	VectorXd z_pred(3);

	if (fabs(rho) < 0.0001){
		rho_dot = 0;
	}
	else{
		rho_dot = (px*vx + py*vy) / rho;
	}

	z_pred << rho, phi, rho_dot;
	VectorXd y = z - z_pred; // 3x1

	//to keep phi in between -pi and pi
	while (y(1) > M_PI)
		y(1) -= 2 * M_PI;
	while (y(1) < -M_PI)
		y(1) += 2 * M_PI;

	MatrixXd Ht = H_.transpose(); // 4x3
	MatrixXd S = H_ * P_ * Ht + R_; // 3x3
	MatrixXd Si = S.inverse(); // 3x3
	MatrixXd PHt = P_ * Ht; //4x3
	MatrixXd K = PHt * Si; // 4x3

	//new estimate
	x_ = x_ + (K * y); // 4x1
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
