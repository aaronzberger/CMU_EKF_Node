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
#include "cmu_unet/line_2pts.h"
#include "cmu_unet/line_list.h"
#include "cmu_ekf/lines_org.h"
#include "cmu_ekf/line_polar.h"
#include <sstream>

// USER PARAMETERS
constexpr double initialCovarianceInput[2][2] = {{0.3, 0  },
                                                 {0  , 0.3}};
constexpr double initialModelErrorInput[2][2] = {{0.02, 0},
                                                 {0, 0.02}};
constexpr double initialMeasurementErrorInput[2][2] = {{0.25, 0   },
                                                       {0   , 0.25}};

constexpr unsigned maxInitialStateDetections = 5;

// Helper Constants
constexpr uint8_t LEFT = cmu_ekf::line_polar::LEFT;
constexpr uint8_t CENTER = cmu_ekf::line_polar::CENTER;
constexpr uint8_t RIGHT = cmu_ekf::line_polar::RIGHT;

constexpr auto NO_LINE = [&]() {
    cmu_ekf::line_polar line;
    line.distance = 0.0;
    line.theta = 0.0;
    // Direction range is [0,2], so 3 is impossible
    line.direction = 3;
    return line;
};

typedef std::pair<cmu_ekf::line_polar, cmu_ekf::line_polar> line_pair;

// Function Prototyes
bool isLeftOf(const cmu_ekf::line_polar s1, const cmu_ekf::line_polar s2);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
std::vector<cmu_ekf::line_polar> pointsToPolar(const cmu_unet::line_list::ConstPtr &msg);
line_pair getClosestLines(const std::vector<cmu_ekf::line_polar> lines);
double getYaw(const double w, const double x, const double y, const double z);
void lineCallback(const cmu_unet::line_list::ConstPtr &msg);
bool startKalmanFilters();

double x, y, yaw;
Kalman filterLeft;
Kalman filterRight;
ros::Subscriber subLines;
ros::Subscriber subOdom;
ros::Publisher pubLines;
Eigen::Matrix3d Tglobal_lastFrame;

/**
 * @brief Determines whether a line is to the left of another line
 * 
 * @param s1 the first line 
 * @param s2 the second line
 * 
 * @return whether s1 is to the left of s2
 */
bool isLeftOf(const cmu_ekf::line_polar s1, const cmu_ekf::line_polar s2) {
    if ((s1.direction == LEFT && s2.direction == RIGHT) ||
        (s1.direction == CENTER && s2.direction == RIGHT) ||
        (s1.direction == LEFT && s2.direction == CENTER)) return true;
    if ((s1.direction == RIGHT && s2.direction == LEFT) ||
        (s1.direction == CENTER && s2.direction == LEFT) ||
        (s1.direction == RIGHT && s2.direction == CENTER)) return false;
    if (s1.direction == s2.direction == LEFT) return s1.distance > s2.distance;
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
std::vector<cmu_ekf::line_polar> pointsToPolar(const cmu_unet::line_list::ConstPtr &msg) {
    std::vector<cmu_ekf::line_polar> lines;
    for (int i{0}; i < msg->num_lines; i++) {
        double a {msg->lines[i].y2 - msg->lines[i].y1};
        double b {msg->lines[i].x1 - msg->lines[i].x2};
        double c {-((a * msg->lines[i].x1) + (b * msg->lines[i].y1))};

        cmu_ekf::line_polar polar;
        polar.distance = std::abs(c) / (std::sqrt(a*a + b*b));
        polar.theta = M_PI_2 + std::atan(-(a / b));
        if (a * c < 0) polar.direction = RIGHT;
        else if (a * c > 0) polar.direction = LEFT;
        else polar.direction = CENTER;

        lines.push_back(polar);
    }
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

/**
 * @brief Retrieve the closest rows to the robot from all rows
 * 
 * @param lines an array of lines (in order)
 * 
 * @return a pair containing a left and right line (either one may be NO_LINE() if it doesn't exist)
 */
line_pair getClosestLines(const std::vector<cmu_ekf::line_polar> lines) {
    if(lines.size() == 0) {
        return std::make_pair(NO_LINE(), NO_LINE());
    }
    if(lines.at(0).direction == RIGHT) {
        return std::make_pair(NO_LINE(), lines.at(0));
    }
    if(lines.at(lines.size() - 1).direction == LEFT) {
        return std::make_pair(lines.at(lines.size() - 1), NO_LINE());
    }
    for(int i{1}; i < lines.size(); i++) {
        if(lines.at(i).direction == RIGHT) {
            return std::make_pair(lines.at(i-1), lines.at(i));
        }
    }
}

/**
 * @brief Update the Kalman filters based on the lines received from U-Net
 * 
 * @param msg the line_list.msg message from U-Net
 */
void lineCallback(const cmu_unet::line_list::ConstPtr &msg) {
    // Convert the received lines to polar form
    std::vector<cmu_ekf::line_polar> polarLines {pointsToPolar(msg)};

    // Sort the lines from left to right
    std::sort(polarLines.begin(), polarLines.end(), isLeftOf);
    
    // Retrieve the nearest line in either direction for filtering (ignore the rest)
    line_pair closest {getClosestLines(polarLines)};

    // Determine the transition model parameters for the EKFs by transforming the frame of reference
    Eigen::Matrix3d Tglobal_thisFrame;
    Tglobal_thisFrame << std::cos(-yaw), std::sin(-yaw), x,
                        -std::sin(-yaw), std::cos(-yaw), y,
                        0              , 0             , 1;
    
    Eigen::Matrix3d TlastFrame_thisFrame;
    TlastFrame_thisFrame = Tglobal_lastFrame.inverse() * Tglobal_thisFrame;

    double deltaX {TlastFrame_thisFrame(0,2)};
    double deltaY {TlastFrame_thisFrame(1,2)};

    Eigen::MatrixXd outputStateLeft(2, 1), outputStateRight(2, 1);

    if(closest.first != NO_LINE()) {
        Eigen::MatrixXd state(2, 1);
        state << closest.first.distance, closest.first.theta;
        outputStateLeft = filterLeft.filter(deltaX, deltaY, yaw, state);
    } else {
        outputStateLeft = filterLeft.filter(deltaX, deltaY, yaw);
    }
    if(closest.second != NO_LINE()) {
        Eigen::MatrixXd state(2, 1);
        state << closest.second.distance, closest.second.theta;
        outputStateRight = filterRight.filter(deltaX, deltaY, yaw, state);
    } else {
        outputStateRight = filterRight.filter(deltaX, deltaY, yaw);
    }

    cmu_ekf::lines_org linesMsg;

    cmu_ekf::line_polar left;
    left.distance = outputStateLeft(0,0);
    left.theta = outputStateLeft(1,0);
    left.direction = LEFT;

    cmu_ekf::line_polar right;
    right.distance = outputStateRight(0,0);
    right.theta = outputStateRight(1,0);
    right.direction = RIGHT;

    linesMsg.left = left;
    linesMsg.right = right;
    linesMsg.far_left = NO_LINE();
    linesMsg.far_right = NO_LINE();

    pubLines.publish(linesMsg);
}

bool startKalmanFilters() {
    // Gather the filter parameters
    Eigen::Matrix2d initialCovariance, modelError, measurementError;
    initialCovariance << initialCovarianceInput[0][0], initialCovarianceInput[0][1],
                            initialCovarianceInput[1][0], initialCovarianceInput[1][1];

    modelError << initialModelErrorInput[0][0], initialModelErrorInput[0][1],
                    initialModelErrorInput[1][0], initialModelErrorInput[1][1];

    measurementError << initialMeasurementErrorInput[0][0], initialMeasurementErrorInput[0][1],
                        initialMeasurementErrorInput[1][0], initialMeasurementErrorInput[1][1];

    // Gather the initial state found by U-Net
    cmu_unet::line_list::ConstPtr list {ros::topic::waitForMessage<cmu_unet::line_list>("/unet_lines")};
    
    // Convert the received lines to polar form
    std::vector<cmu_ekf::line_polar> polarLines {pointsToPolar(list)};

    // Sort the lines from left to right
    std::sort(polarLines.begin(), polarLines.end(), isLeftOf);

    // Retrieve the nearest line in either direction
    line_pair closest {getClosestLines(polarLines)};

    // For the initial state detection, all lines must be present. If they are not, wait for another reading:
    for (int i{0}; i < maxInitialStateDetections && 
        (closest.first == NO_LINE() || closest.second == NO_LINE()); i++) {
        list = ros::topic::waitForMessage<cmu_unet::line_list>("/unet_lines");
        polarLines = pointsToPolar(list);
        std::sort(polarLines.begin(), polarLines.end(), isLeftOf);
        closest = getClosestLines(polarLines);
    }

    if (closest.first == NO_LINE() || closest.second == NO_LINE()) {
        std::stringstream ss;
        ss << "Tried " << maxInitialStateDetections << " times, but only " << ((closest.first == NO_LINE() && closest.second == NO_LINE()) ? 0 : 1) << 
              " out of 2 required rows were detected. Save an image of the results in the UNet Node to diagnose the problem.";
        ROS_INFO(ss.str().c_str());
        return false;
    }

    // Initialize the Kalman Filters
    Eigen::MatrixXd left(2, 1), right(2, 1);
    left << closest.first.distance, closest.first.theta;
    right << closest.second.distance, closest.second.theta;

    filterLeft = Kalman(x, y, yaw, left, initialCovariance, modelError, measurementError);
    filterRight = Kalman(x, y, yaw, right, initialCovariance, modelError, measurementError);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf");

    ros::NodeHandle n;

    subLines = n.subscribe<cmu_unet::line_list>("/unet_lines", 1, lineCallback);
    subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, odomCallback);
    pubLines = n.advertise<cmu_ekf::lines_org>("/ekf_lines", 1);

    // Gather the initial odometry
    nav_msgs::Odometry::ConstPtr initialOdom {ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered")};
    odomCallback(initialOdom);

    if(!startKalmanFilters()) return -1;

    ros::spin();

    return 0;
}