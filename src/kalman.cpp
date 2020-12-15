#include "kalman.h"

#include <Eigen/Dense>
#include <cmath>

Kalman::Kalman(Eigen::MatrixXd initialState,
               Eigen::Matrix2d covInitial, Eigen::Matrix2d modelError, 
               Eigen::Matrix2d measurementError, Eigen::Matrix2d observationTransform) 
    :initialCovariance{initialCovariance}, observationTransform{observationTransform} {
    setModelError(modelError);
    setMeasurementError(measurementError);

    statePrediction = Eigen::MatrixXd(2,1);
    stateUpdated = Eigen::MatrixXd(2,1);

    stateUpdated = initialState;
}

/**
 * @brief Perform a time update and measurement update on this filter
 * 
 * @param deltaX change in X since last update (in the last update's reference frame)
 * @param deltaY change in Y since last update (in the last update's reference frame)
 * @param deltaTheta change in yaw of the robot
 * @param detectedState a 2x1 matrix containing distance and theta
 * 
 * @return a 2x1 matrix containing the filtered distance and theta
 */
Eigen::MatrixXd Kalman::filter(double deltaX, double deltaY, double deltaTheta, Eigen::MatrixXd detectedState) {
    // State Extrapolation
    statePrediction(0,0) = stateUpdated(0,0) - ((deltaX * std::cos(stateUpdated(1,0))) + (deltaY * std::sin(stateUpdated(1,0))));
    statePrediction(1,0) = stateUpdated(1,0) - deltaTheta;

    // Covariance Extrapolation
    Eigen::Matrix2d jacobian;
    jacobian << 1, (deltaX * std::sin(stateUpdated(1,0))) - (deltaY * std::cos(stateUpdated(1,0))),
                0, 1;

    covPrediction = (jacobian * covUpdated * jacobian.transpose()) + modelError;

    // Kalman Gain
    kGain = (covPrediction * observationTransform.transpose()) * 
            ((observationTransform * covPrediction * observationTransform.transpose() + measurementError).inverse());

    // State Update
    stateUpdated = statePrediction + (kGain * (detectedState - statePrediction));

    // Covariance Update
    covUpdated = (Eigen::Matrix2d::Identity() - (kGain * observationTransform)) * covPrediction;

    return stateUpdated;
}

/**
 * @brief Perform a time update on this filter without a valid measurement
 * 
 * @param deltaX change in X since last update (in the last update's reference frame)
 * @param deltaY change in Y since last update (in the last update's reference frame)
 * @param deltaTheta change in yaw of the robot
 * 
 * @return a 2x1 matrix containing the predicted distance and theta for this time step
 */
Eigen::MatrixXd Kalman::filter(double deltaX, double deltaY, double deltaTheta) {
    // State Extrapolation
    statePrediction(0,0) = stateUpdated(0,0) - ((deltaX * std::cos(stateUpdated(1,0))) + (deltaY * std::sin(stateUpdated(1,0))));
    statePrediction(1,0) = stateUpdated(1,0) - deltaTheta;

    // Odometry is our only input information, so we will rely on our state transition
    stateUpdated = statePrediction;

    // Keep covariance the same. If we re-calculate it, zero error will be detected
    // since we manually set the prior and posterior estimates to be equal. The EKF will
    // then decrease covariance sharply and begin trusting its prediction too much.

    return stateUpdated;
}
