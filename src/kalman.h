#ifndef KALMAN_H
#define KALMAN_H

#include <cmath>
#include <Eigen/Dense>

class Kalman {
public:
    Kalman() : Kalman(Eigen::MatrixXd::Identity(2,1)) {};
    Kalman(Eigen::MatrixXd initialState,
           Eigen::Matrix2d covInitial = Eigen::Matrix2d::Identity(), Eigen::Matrix2d modelError = Eigen::Matrix2d::Zero(), 
           Eigen::Matrix2d measurementError = Eigen::Matrix2d::Zero(), Eigen::Matrix2d observationTransform = Eigen::Matrix2d::Identity());
    Eigen::MatrixXd filter(double deltaX, double deltaY, double deltaTheta, Eigen::MatrixXd detectedState);
    Eigen::MatrixXd timeUpdate(double deltaX, double deltaY, double deltaTheta);
    Eigen::MatrixXd predictionUpdate(double deltaX, double deltaY, double deltaTheta);
    Eigen::MatrixXd measurementUpdate(double deltaX, double deltaY, Eigen::MatrixXd detectedState);
    void setModelError(Eigen::Matrix2d modelError) {this->modelError = modelError;};
    void setMeasurementError(Eigen::Matrix2d measurementError) {this->measurementError = measurementError;};
    
private:
    Eigen::Matrix2d covPrediction, covUpdated, kGain;
    Eigen::MatrixXd statePrediction, stateUpdated;
    Eigen::Matrix2d initialCovariance, modelError, measurementError, covInitial, observationTransform;
};

#endif