# ROS EKF Node for Autonomous Lidar Navigation

Receive row data from the U-Net node and differentiate and track each line.

![Node Pipeline](https://user-images.githubusercontent.com/35245591/101996100-547de180-3c9d-11eb-8e79-2426dafaca41.png)

## Table of Contents
- [Details](#Details)
- [Extended Kalman Filter](#Extended-Kalman-Filter)
  - [Row Representation](#Row-Representation)
  - [State Transition Matrix](#State-Transition-Matrix)
  - [Parameters](#Parameters)
- [Pipeline](#Pipeline)
- [Usage](#Usage)
- [Acknowledgements](#Acknowledgements)

## Details
Every timestep, a `lineGroup` message is received from the U-Net node, and undergoes the following:

- The lines are [converted](https://github.com/aaronzberger/CMU_EKF_Node/blob/603475039d350126492272b0f50923d9ddbec81c/src/main.cpp#L207) to polar form
- The polar-form lines are [sorted](https://github.com/aaronzberger/CMU_EKF_Node/blob/603475039d350126492272b0f50923d9ddbec81c/src/main.cpp#L210) from left to right
- We [differentiate](https://github.com/aaronzberger/CMU_EKF_Node/blob/603475039d350126492272b0f50923d9ddbec81c/src/main.cpp#L213) these lines into 'slots' like *left*, *right*, and possibly *far left* and *far right*, that represent their location relative to the robot
- The lines are [passed through](https://github.com/aaronzberger/CMU_EKF_Node/blob/603475039d350126492272b0f50923d9ddbec81c/src/main.cpp#L232-L248) the [Extended Kalman Filter](#Extended-Kalman-Filter) for tracking and smoothing

The resulting lines from the Extended Kalman Filter are published to the `ekf_lines` ROS topic in a `lines_org` message (which means 'lines organized').

## Extended Kalman Filter
For tracking each row and smoothing the inputs from the U-Net model, we use an [Extended Kalman Filter](https://www.kalmanfilter.net/default.aspx). A brief explanation of the algorithm is below:

- Each time step, an Extended Kalman Filter predicts what the current state should be, through the State Transition Matrix. This is called the 'prior' estimate

- Then, it receives the measurement of the current state by the robot and updates its prediction. This is called the 'posterior' estimate, and is essentially the output of the filter

- During this process, the filter also updates the covariance matrix, which encodes the filter's confidence in the model
  - If the major axis of the covariance matrix is small, the filter trusts its model more, and cares less about the measurements coming in from the robot
  - If the major axis is large, however, the filter does not trust its model and relies more on the measurements to make its 'posterior' predictions

#### Row Representation
Because this pipeline is intended for use in a vineyard, the rows should never change direction.

We can represent each row as its closest point to the robot. This point's polar form will be the state for our Extended Kalman Filter. With this representation, we can easily obtain the line that represents the row, as it is normal to the vector to this point, and contains this point.

The theta in our row model represents the angle between the robot and the vector to this point:

If the robot is parallel to the row, theta will be 90 degrees. As the robot turns to face the row, theta will decrease, and as the robot turns away from the row, theta will increase (for rows on both the left and right).
 
![State](https://user-images.githubusercontent.com/35245591/102545079-c1212380-4083-11eb-8e33-7a14571bc360.png)

#### State Transition Matrix
The State Transition Matrix/Model is how the Extended Kalman Filter extrapolates the estimate for the current state (this is the 'prior' estimate).

![State Transition Matrix](https://user-images.githubusercontent.com/35245591/103568173-1be3b900-4e93-11eb-81a5-bbc53120d044.png)

In this case, we are tracking static objects, so the model should be perfect except for inaccuracies in odometry caused by drifting.

#### Parameters
There are a few parameters for an Extended Kalman Filter that may need to be adjusted. They are defined at the top of `main.cpp`. The values you decide should be input into the major axis of these matrices.

- Intitial Covariance
  - Confidence the model should have in the initial state. For this case, the intial state is simply one reading from the U-Net model
- Model Error
  - Essentially the minimum value for the covariance matrix, to avoid overconfidence in the model if the filter predicts accurately many times
- Measurement Error
  - Essentially the minimum amount the measurement must be used to calculate the 'posterior' prediction

## Pipeline
This node is part of a larger autonomous navigation pipeline that is currently being developed. 

This node is the __ROW TRACKING__ node represented in the full pipeline below:

![Full Pipeline](https://user-images.githubusercontent.com/35245591/101267056-ed06e580-3722-11eb-9215-42603bd370c5.png)

Each node is a separate ROS node, each receiving and publishing relevant data.

Code locations for the other nodes are listed below:
- [__VISION__](https://github.com/aaronzberger/CMU_UNet_Node)
- [__PATH PLANNING__](https://github.com/aaronzberger/CMU_Path_Planning_Node)
- __DRIVING__ (Not yet on Github)

## Usage
  
  `rosrun CMU_EKF_Node ekf`

## Acknowledgements
- Francisco Yandun, for state transition matrix and assistance
