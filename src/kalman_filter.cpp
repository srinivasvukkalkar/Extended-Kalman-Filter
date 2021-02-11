#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using namespace std;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
//   cout << "BEGIN Predict" << endl;
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
//   cout << "END Predict" << endl;
}

void KalmanFilter::pos_cal(const VectorXd &y){  
//   cout << "BEGIN pos_cal" << endl;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
//   x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
//   cout << "END pos_cal" << endl;
}

void KalmanFilter::Update(const VectorXd &z) {  
//   cout << "BEGIN Update" << endl;
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  pos_cal(y);
//   cout << "END Update" << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {  
//   cout << "BEGIN UpdateEKF" << endl;
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float pos_x = x_(0);
  float pos_y = x_(1);
  float vel_x = x_(2);
  float vel_y = x_(3);
  
  float rho = sqrt(pos_x * pos_x + pos_y * pos_y);
  float phi = atan2(pos_y, pos_x);
  float rho_dot;

  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (pos_x * vel_x + pos_y * vel_y)/rho;
  }
    
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  
  // Angle Normalization [-pi,pi]
  y[1] = atan2(sin(y[1]), cos(y[1]));
  
  pos_cal(y);
//   cout << "END UpdateEKF" << endl;
}
