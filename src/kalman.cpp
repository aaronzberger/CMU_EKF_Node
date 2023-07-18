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

    covUpdated = initialCovariance;

    // If the covarianceUpdated is nan then set to 0
    if (covUpdated(0,0) != covUpdated(0,0) || covUpdated(1,1) != covUpdated(1,1) || covUpdated(0,1) != covUpdated(0,1) || covUpdated(1,0) != covUpdated(1,0)) {
        std::cout << "Covariance is nan, setting to 0" << std::endl;
        covUpdated = Eigen::Matrix2d::Zero();
    }

    stateUpdated = initialState;
}

/**
 * @brief Make and return a state prediction without updating the actual state. Should normally be followed by a filter(detectedState) call.
 * 
 * @param deltaX change in X since last update (in the last update's reference frame)
 * @param deltaY change in Y since last update (in the last update's reference frame)
 * @param deltaYaw change in yaw since last update
 * 
 * @return a 2x1 matrix containing the predicted distance and theta for this time step
*/
Eigen::MatrixXd Kalman::predictionUpdate(double deltaX, double deltaY, double deltaTheta) {
    // State Extrapolation
    statePrediction(0,0) = stateUpdated(0,0) - ((deltaX * std::cos(stateUpdated(1,0))) + (deltaY * std::sin(stateUpdated(1,0))));
    statePrediction(1,0) = stateUpdated(1,0) - deltaTheta;

    return statePrediction;
}

/**
 * @brief Perform a measurement update on this filter. Should normally be preceded by a makePrediction() call.
 * 
 * @param detectedState a 2x1 matrix containing distance and theta
 * 
 * @return a 2x1 matrix containing the filtered distance and theta
*/
Eigen::MatrixXd Kalman::measurementUpdate(double deltaX, double deltaY, Eigen::MatrixXd detectedState) {
    Eigen::Matrix2d jacobian;
    jacobian << 1, (deltaX * std::sin(stateUpdated(1,0))) - (deltaY * std::cos(stateUpdated(1,0))),
                0, 1;

    covPrediction = (jacobian * covUpdated * jacobian.transpose()) + modelError;

    // std::cout << "jacobian: " << jacobian << " " << "covUpdated: " << covUpdated << " " << "modelError: " << modelError << std::endl;

    // Kalman Gain
    kGain = (covPrediction * observationTransform.transpose()) * 
            ((observationTransform * covPrediction * observationTransform.transpose() + measurementError).inverse());

    // std::cout << "kGain: " << kGain << std::endl;
    // std::cout << "covPrediction: " << covPrediction << " " << "observationTransform: " << observationTransform << " " << "measurementError: " << measurementError << std::endl;

    // State Update
    stateUpdated = statePrediction + (kGain * (detectedState - statePrediction));

    // Covariance Update
    covUpdated = (Eigen::Matrix2d::Identity() - (kGain * observationTransform)) * covPrediction;

    // std::cout << "stateUpdated: " << stateUpdated << std::endl;

    return stateUpdated;
}

/**
 * @brief Perform a time update and measurement update on this filter
 * 
 * @param deltaX change in X since last update (in the last update's reference frame)
 * @param deltaY change in Y since last update (in the last update's reference frame)
 * @param deltaYaw change in yaw since last update
 * @param detectedState a 2x1 matrix containing distance and theta
 * 
 * @return a 2x1 matrix containing the filtered distance and theta
 */
Eigen::MatrixXd Kalman::filter(double deltaX, double deltaY, double deltaYaw, Eigen::MatrixXd detectedState) {
    predictionUpdate(deltaX, deltaY, deltaYaw);
    measurementUpdate(deltaX, deltaY, detectedState);
}

/**
 * @brief Perform a time-only update on this filter (update the filter state)
 * 
 * @param deltaX change in X since last update (in the last update's reference frame)
 * @param deltaY change in Y since last update (in the last update's reference frame)
 * @param deltaTheta change in yaw since last update
 * 
 * @return a 2x1 matrix containing the predicted distance and theta for this time step
 */
Eigen::MatrixXd Kalman::timeUpdate(double deltaX, double deltaY, double deltaTheta) {
    // State Extrapolation
    predictionUpdate(deltaX, deltaY, deltaTheta);

    // Odometry is our only input information, so we will rely on our state transition
    stateUpdated = statePrediction;

    // Keep covariance the same. If we re-calculate it, zero error will be detected
    // since we manually set the prior and posterior estimates to be equal. The EKF will
    // then decrease covariance sharply and begin trusting its prediction too much.

    return stateUpdated;
}
