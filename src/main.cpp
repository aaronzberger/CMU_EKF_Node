#include "ros/ros.h"
#include "kalman.h"
#include "kalman.cpp"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include <Eigen/Dense>
#include <algorithm>
#include <utility>
#include <tuple>
#include "CMU_UNet_Node/line_2pts.h"
#include "CMU_UNet_Node/line_list.h"
#include "CMU_EKF_Node/lines_org.h"
#include "CMU_EKF_Node/line_polar.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

// USER PARAMETERS
constexpr double initialCovarianceInput[2][2] = {{0.3, 0  },
                                                 {0  , 0.3}};
                                                 
// 12/16/20 : Temporarily setting model error to 0 while UNet is undertrained,
// so I can work on path planning with consistent measurements.
constexpr double initialModelErrorInput[2][2] = {{0, 0},
                                                 {0, 0}};
constexpr double initialMeasurementErrorInput[2][2] = {{0.3, 0   },
                                                       {0  , 0.3}};

constexpr unsigned maxInitialStateDetections = 5;

constexpr double ROW_SPACING = 1;
constexpr bool MIDDLE_ROW = true;

// Helper Constants
constexpr uint8_t LEFT = CMU_EKF_Node::line_polar::LEFT;
constexpr uint8_t CENTER = CMU_EKF_Node::line_polar::CENTER;
constexpr uint8_t RIGHT = CMU_EKF_Node::line_polar::RIGHT;

constexpr char lineTopic[] = "/lines";
constexpr char odomTopic[] = "/odometry/filtered";

/**
 * @brief A line_polar with all values set to 0
*/
CMU_EKF_Node::line_polar NO_LINE() {
    CMU_EKF_Node::line_polar line;
    line.distance = 0.0;
    line.theta = 0.0;
    // Direction range is [0,2], so 3 is impossible
    line.direction = 3;
    return line;
};


// Function Prototyes
bool isLeftOf(const CMU_EKF_Node::line_polar s1, const CMU_EKF_Node::line_polar s2);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
std::vector<CMU_EKF_Node::line_polar> pointsToPolar(const CMU_UNet_Node::line_list::ConstPtr &msg);
double getYaw(const double w, const double x, const double y, const double z);
void lineCallback(const CMU_UNet_Node::line_list::ConstPtr &msg);
bool startKalmanFilters();

// Robot odometry
double x, y, yaw;

// TODO: Replace this with a list of filters, to allow for variable number of lines
Kalman filterLeft;
Kalman filterCenter;
Kalman filterRight;

ros::Publisher pubLines;

// Last reference frame for obtaining deltaX and deltaY each time step
Eigen::Matrix3d Tglobal_lastFrame;
double lastYaw;

// For graphing EKF results
std::ofstream ekfGraphing;
int graphCounter;

/**
 * @brief Determines whether a line is to the left of another line
 * 
 * @param s1 the first line 
 * @param s2 the second line
 * 
 * @return whether s1 is to the left of s2
 */
bool isLeftOf(const CMU_EKF_Node::line_polar s1, const CMU_EKF_Node::line_polar s2) {
    if ((s1.direction == LEFT && s2.direction == RIGHT) ||
        (s1.direction == CENTER && s2.direction == RIGHT) ||
        (s1.direction == LEFT && s2.direction == CENTER)) return true;
    if ((s1.direction == RIGHT && s2.direction == LEFT) ||
        (s1.direction == CENTER && s2.direction == LEFT) ||
        (s1.direction == RIGHT && s2.direction == CENTER)) return false;
    if (s1.direction == LEFT && s2.direction == LEFT) return s1.distance > s2.distance;
    else return s1.distance < s2.distance;
}

/**
 * @brief Set the global robot odometry variables
 * 
 * @param msg the Odometry message
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    geometry_msgs::Quaternion orientation = {msg->pose.pose.orientation};
    yaw = getYaw(orientation.w, orientation.x, orientation.y, orientation.z);
}

/**
 * @brief Convert a list of lines from two-point format to polar
 * 
 * @param msg the line_list.msg received
 * 
 * @return a vector of states, each containing the polar form of the lines
 */
std::vector<CMU_EKF_Node::line_polar> pointsToPolar(const CMU_UNet_Node::line_list::ConstPtr &msg) {
    std::vector<CMU_EKF_Node::line_polar> lines;
    for (int i{0}; i < msg->num_lines; i++) {
        double x1 { msg->lines[i].x1 };
        double y1 { msg->lines[i].y1 };
        double x2 { msg->lines[i].x2 };
        double y2 { msg->lines[i].y2 };

        double dx {x2 - x1};
        double dy {y2 - y1};
        double det {(dx*dx) + (dy*dy)};
        double num {((dy * -y1) + (dx * -x1)) / det};

        double closestX {x1 + (num * dx)};
        double closestY {y1 + (num * dy)};

        double a {dy};
        double b {-dx};
        double c {-((a * x1) + (b * y1))};

        CMU_EKF_Node::line_polar polar;

        // Use the y intercept of the line. We don't use the distance formula
        // because it would give us the distance to a point on the line that may be ahead or behind the robot
        polar.distance = std::abs(c / b);

        // Ignore the direction of the robot. PI/2 is parallel to the line,
        // and theta decreases as the robot turns towards the line (in either direction)
        polar.theta = std::abs(std::atan2(closestY, closestX));
        
        if (c / b > 0) polar.direction = RIGHT;
        else if (c / b < 0) polar.direction = LEFT;
        else polar.direction = CENTER;

        lines.push_back(polar);
    }

    // If there are two lines at similar but opposite distance, 
    return lines;
}

/**
 * @brief Find the yaw from a quaternion
 * 
 * @param w the w of the quaternion
 * @param x the x of the quaternion
 * @param y the y of the quaternion
 * @param z the z of the quaternion
 * 
 * @return the yaw calculated
 */
double getYaw(const double w, const double x, const double y, const double z) {
    tf::Quaternion quat;
    quat.setW(w);
    quat.setX(x);
    quat.setY(y);
    quat.setZ(z);

    return tf::getYaw(quat);
}

CMU_EKF_Node::lines_org organizeLines(const std::vector<CMU_EKF_Node::line_polar> lines) {
    CMU_EKF_Node::lines_org organized;
    
    // Twice through all spots: take the closest line, unless it's closer to another
    // unassigned spot, or its further than another assigned line

    double spots[5] = {-ROW_SPACING * 2, -ROW_SPACING, 0, ROW_SPACING, ROW_SPACING * 2};
    int assigned[5] = {-1, -1, -1, -1, -1};

    // Do this process twice
    for (int k{0}; k < 2; k++) {
        for (int i{0}; i < 5; i++) {
            // Don't assign the middle row if it doesn't exist
            if (!MIDDLE_ROW && i == 2) continue;

            // Don't assign a spot that's already assigned
            if (assigned[i] != -1) continue;

            // Find the closest line to the current spot
            double closestLineDist{1000};
            int closestLineIndex{-1};
            for (int j{0}; j < lines.size(); j++) {
                double position = lines.at(j).distance * (lines.at(j).direction == RIGHT ? 1 : -1);
                double dist = std::abs(position - spots[i]);
                if (dist < closestLineDist) {
                    closestLineDist = dist;
                    closestLineIndex = j;
                }
            }

            // For that closest line, find the closest spot
            double linePosition {lines.at(closestLineIndex).distance * (lines.at(closestLineIndex).direction == RIGHT ? 1 : -1)};
            double closestSpotDist{1000};
            int closestSpotIndex{-1};
            for (int j{0}; j < 5; j++) {
                double dist = std::abs(spots[j] - linePosition);
                if (dist < closestSpotDist) {
                    closestSpotDist = dist;
                    closestSpotIndex = j;
                }
            }

            // If they match, or if the closest spot is already assigned, take this line
            if (closestSpotIndex == i || (assigned[closestSpotIndex] != -1 && assigned[closestSpotIndex] != closestLineIndex)) {
                assigned[i] = closestLineIndex;
            }
        }
    }

    // Assign the lines to the output
    if (assigned[0] != -1) organized.far_left = lines.at(assigned[0]);
    if (assigned[1] != -1) organized.left = lines.at(assigned[1]);
    if (assigned[2] != -1) organized.center = lines.at(assigned[2]);
    if (assigned[3] != -1) organized.right = lines.at(assigned[3]);
    if (assigned[4] != -1) organized.far_right = lines.at(assigned[4]);

    return organized;
}

/**
 * @brief Update the Kalman filters based on the lines received from U-Net
 * 
 * @param msg the line_list.msg message from U-Net
 */
void lineCallback(const CMU_UNet_Node::line_list::ConstPtr &msg) {
    // Convert the received lines to polar form
    std::vector<CMU_EKF_Node::line_polar> polarLines {pointsToPolar(msg)};

    // TODO: Optimally, in the first frame the positions of all lines are decided and from then on,
    // lines are matched with the filter they are closest to (by prediction, perhaps with a small weighting
    // towards what makes sense for the current frame). For now, they're decided each frame, then inserted.

    // Determine the transition model parameters for the EKFs by transforming the frame of reference
    Eigen::Matrix3d Tglobal_thisFrame;
    Tglobal_thisFrame << std::cos(-yaw), std::sin(-yaw), x,
                        -std::sin(-yaw), std::cos(-yaw), y,
                        0              , 0             , 1;
    
    Eigen::Matrix3d TlastFrame_thisFrame;
    TlastFrame_thisFrame = Tglobal_lastFrame.inverse() * Tglobal_thisFrame;

    double deltaX {TlastFrame_thisFrame(0,2)};
    double deltaY {TlastFrame_thisFrame(1,2)};

    Tglobal_lastFrame = Tglobal_thisFrame;

    // Get predictions for all the filters
    Eigen::MatrixXd predictionLeft = filterLeft.predictionUpdate(deltaX, deltaY, yaw - lastYaw);
    Eigen::MatrixXd predictionCenter = filterCenter.predictionUpdate(deltaX, deltaY, yaw - lastYaw);
    Eigen::MatrixXd predictionRight = filterRight.predictionUpdate(deltaX, deltaY, yaw - lastYaw);

    // Set the output state for the two rows by updating the EKF for each
    Eigen::MatrixXd outputStateLeft(2, 1), outputStateRight(2, 1);

    // Match each line to the closest filter prediction (by distance)


    if(closest.first != NO_LINE()) {
        Eigen::MatrixXd state(2, 1);
        state << closest.first.distance, closest.first.theta;
        // std::cout << "Start left update" << std::endl;
        outputStateLeft = filterLeft.filter(deltaX, deltaY, yaw - lastYaw, state);
        // std::cout << "End left update" << outputStateLeft << std::endl;
    } else {
        ROS_INFO("No line on the left found, time update only");
        outputStateLeft = filterLeft.filter(deltaX, deltaY, yaw - lastYaw);
    }
    if(closest.second != NO_LINE()) {
        Eigen::MatrixXd state(2, 1);
        state << closest.second.distance, closest.second.theta;
        // Reverse deltaYaw so the direction is mirrored for rows on both sides
        outputStateRight = filterRight.filter(deltaX, deltaY, lastYaw - yaw, state);
    } else {
        ROS_INFO("No line on the right found, time update only");
        outputStateRight = filterRight.filter(deltaX, deltaY, lastYaw - yaw);
    }

    // std::cout << "Left: " << outputStateLeft << std::endl;
    // std::cout << "Right: " << outputStateRight << std::endl;

    lastYaw = yaw;

    // Publish the message containing these two rows
    CMU_EKF_Node::lines_org linesMsg;

    CMU_EKF_Node::line_polar left;
    left.distance = outputStateLeft(0,0);
    left.theta = outputStateLeft(1,0);
    left.direction = LEFT;

    CMU_EKF_Node::line_polar right;
    right.distance = outputStateRight(0,0);
    right.theta = outputStateRight(1,0);
    right.direction = RIGHT;

    linesMsg.left = left;
    linesMsg.right = right;
    linesMsg.far_left = NO_LINE();
    linesMsg.far_right = NO_LINE();

    pubLines.publish(linesMsg);

    // Save the original lines and smoothed lines to a csv file for graphing
    ekfGraphing << graphCounter << "," << closest.first.distance << "," << closest.first.theta << 
    "," << outputStateLeft(0,0) << "," << outputStateLeft(1,0) << "\n";
    graphCounter++;
}

/**
 * @brief Start or restart the EKFs for the left and right rows
 * 
 * @return whether both filters were successfully setup. If they weren't it is due to insufficient state detection.
 */
bool startKalmanFilters() {
    // Gather the filter parameters
    Eigen::Matrix2d initialCovariance, modelError, measurementError;
    initialCovariance << initialCovarianceInput[0][0], initialCovarianceInput[0][1],
                            initialCovarianceInput[1][0], initialCovarianceInput[1][1];

    modelError << initialModelErrorInput[0][0], initialModelErrorInput[0][1],
                    initialModelErrorInput[1][0], initialModelErrorInput[1][1];

    measurementError << initialMeasurementErrorInput[0][0], initialMeasurementErrorInput[0][1],
                        initialMeasurementErrorInput[1][0], initialMeasurementErrorInput[1][1];

    // Gather the initial state
    CMU_UNet_Node::line_list::ConstPtr list {ros::topic::waitForMessage<CMU_UNet_Node::line_list>(lineTopic)};
    
    // Convert the received lines to polar form
    std::vector<CMU_EKF_Node::line_polar> polarLines {pointsToPolar(list)};

    // Organize the lines into the rows
    CMU_EKF_Node::lines_org organized {organizeLines(polarLines)};


    // For the initial state detection, all lines must be present. If they are not, wait for another reading:
    for (int i{0}; i < maxInitialStateDetections && 
        ((MIDDLE_ROW && organized.center == NO_LINE()) || (organized.left == NO_LINE() || organized.right == NO_LINE())); i++) {
        list = ros::topic::waitForMessage<CMU_UNet_Node::line_list>(lineTopic);
        polarLines = pointsToPolar(list);
        organized = organizeLines(polarLines);
    }

    if ((MIDDLE_ROW && organized.center == NO_LINE()) || (organized.left == NO_LINE() || organized.right == NO_LINE())) {
        std::stringstream ss;
        ss << "Tried " << maxInitialStateDetections << " times, but either the middle row is not present (if navigating with middle row)" <<
        " or the left and right rows are not present (if there's no middle row). Aborting";
        ROS_INFO(ss.str().c_str());
        return false;
    }
    
    ROS_INFO("Collected initial state detection");

    // Initialize the Kalman Filters
    Eigen::MatrixXd left(2, 1), center(2,1), right(2, 1);
    left << organized.left.distance, organized.left.theta;
    center << organized.center.distance, organized.center.theta;
    right << organized.right.distance, organized.right.theta;

    std::cout << "Initial state left: " << left << std::endl;
    std::cout << "Initial state center: " << center << std::endl;
    std::cout << "Initial state right: " << right << std::endl;

    // Gather the initial odometry
    // This must be after waiting for the lines to publish, 
    // in case the robot moves during that time (which wouldn't be tracked)
    nav_msgs::Odometry::ConstPtr initialOdom {ros::topic::waitForMessage<nav_msgs::Odometry>(odomTopic)};
    odomCallback(initialOdom);
    
    ROS_INFO("Gathered initial robot odometry");

    filterLeft = Kalman(left, initialCovariance, modelError, measurementError);
    filterCenter = Kalman(center, initialCovariance, modelError, measurementError);
    filterRight = Kalman(right, initialCovariance, modelError, measurementError);

    lastYaw = yaw;

    // Set an initial frame of reference: must be done after gathering initial odometry,
    // or else x, y, and yaw will be default 0 and the delta values will be wrong on first loop.
    Tglobal_lastFrame << std::cos(-yaw), std::sin(-yaw), x,
                        -std::sin(-yaw), std::cos(-yaw), y,
                        0              , 0             , 1;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf");

    ros::NodeHandle n;

    ros::Subscriber subLines = n.subscribe<CMU_UNet_Node::line_list>(lineTopic, 1, lineCallback);
    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>(odomTopic, 1, odomCallback);
    pubLines = n.advertise<CMU_EKF_Node::lines_org>("/ekf_lines", 1);

    // For graphing EKF results
    ekfGraphing = std::ofstream("ekf_graphing.csv");
    graphCounter = 0;

    if(!startKalmanFilters()) return -1;

    ros::spin();

    ekfGraphing.close();
    return 0;
}