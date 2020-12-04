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
#include "state.h"
#include "cmu_unet/line.h"
#include "cmu_unet/line_list.h"
#include "cmu_ekf/lines_org.h"
#include <sstream>

// USER PARAMETERS
constexpr double initialCovarianceInput[2][2] = {{0.3, 0  },
                                                 {0  , 0.3}};
constexpr double initialModelErrorInput[2][2] = {{0.02, 0},
                                                 {0, 0.02}};
constexpr double initialMeasurementErrorInput[2][2] = {{0.25, 0   },
                                                       {0   , 0.25}};

constexpr unsigned maxInitialStateDetections = 5;

constexpr bool count4Lines = false;

// Helper Constants
constexpr state::Direction LEFT = state::Direction::LEFT;
constexpr state::Direction CENTER = state::Direction::CENTER;
constexpr state::Direction RIGHT = state::Direction::RIGHT;


// Function Prototyes
bool isLeftOf(state s1, state s2);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
std::vector<state> pointsToPolar(cmu_unet::line_list::ConstPtr &msg);
double getYaw(double w, double x, double y, double z);
void lineCallback(const cmu_unet::line_list::ConstPtr &msg);


double x, y, yaw;
Kalman filterLeft;
Kalman filterRight;
Kalman filterFarLeft;
Kalman filterFarRight;
ros::Subscriber subLines;
ros::Subscriber subOdom;
ros::Publisher pubLines;

/**
 * @brief Determines whether a line is to the left of another line
 * 
 * @param s1 the first line 
 * @param s2 the second line
 * 
 * @return whether s1 is to the left of s2
 */
bool isLeftOf(state s1, state s2) {
    if ((s1.direction == LEFT && s2.direction == RIGHT) ||
        (s1.direction == CENTER && s2.direction == RIGHT)) return true;
    if ((s1.direction == RIGHT && s2.direction == LEFT) ||
        (s1.direction == CENTER && s2.direction == LEFT)) return false;
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
std::vector<state> pointsToPolar(cmu_unet::line_list::ConstPtr &msg) {
    std::vector<state> lines;
    for (int i{0}; i < msg->num_lines; i++) {
        double a {msg->lines[i].y2 - msg->lines[i].y1};
        double b {msg->lines[i].x1 - msg->lines[i].x2};
        double c {-((a * msg->lines[i].x1) + (b * msg->lines[i].y1))};

        state polar;
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
double getYaw(double w, double x, double y, double z) {
    tf::Quaternion quat;
    quat.setW(w);
    quat.setX(x);
    quat.setY(y);
    quat.setZ(z);

    return tf::getYaw(quat);
}

/**
 * @brief Receive a line list, differentiate which is which, filter them, and publish
 * 
 * @TODO Implement this method
 */
void lineCallback(const cmu_unet::line_list::ConstPtr &msg) {
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

    // Gather the filter parameters
    Eigen::Matrix2d initialCovariance, modelError, measurementError;
    initialCovariance << initialCovarianceInput[0][0], initialCovarianceInput[0][1],
                            initialCovarianceInput[1][0], initialCovarianceInput[1][1];

    modelError << initialModelErrorInput[0][0], initialModelErrorInput[0][1],
                    initialModelErrorInput[1][0], initialModelErrorInput[1][1];

    measurementError << initialMeasurementErrorInput[0][0], initialMeasurementErrorInput[0][1],
                        initialMeasurementErrorInput[1][0], initialMeasurementErrorInput[1][1];


    // Gather the initial state found by UNet
    cmu_unet::line_list::ConstPtr list {ros::topic::waitForMessage<cmu_unet::line_list>("/unet_lines")};

    // For the initial state detection, all lines must be present. If they are not, wait for another reading:
    for (int i{0}; i < maxInitialStateDetections && list->lines.size() < (count4Lines ? 4 : 2); i++) {
        list = ros::topic::waitForMessage<cmu_unet::line_list>("/unet_lines");
    }

    if (list->lines.size() < (count4Lines ? 4 : 2)) {
        std::stringstream ss;
        ss << "Tried " << maxInitialStateDetections << " times, but only " << list->lines.size() << " out of " << (count4Lines ? 4 : 2) <<
              " required rows were detected. Try saving an image of the results in the UNet Node (code is already there).";
        ROS_INFO(ss.str().c_str());
        return 0;
    }

    std::vector<state> polar {pointsToPolar(list)};
    std::sort(polar.begin(), polar.end(), isLeftOf);

    // Middle represents in-between which lines the robot is
    std::size_t middle = 0;
    while(middle < polar.size() - 1 && !(polar.at(middle).theta < 0 && polar.at(middle + 1).theta > 0)) {
        middle++;
    }
    middle += 0.5;

    // Initialize the Kalman Filters
    if(!count4Lines) {
        Eigen::MatrixXd left(2, 1), right(2, 1);
        left << polar.at(middle - 0.5).distance, polar.at(middle - 0.5).theta;
        right << polar.at(middle + 0.5).distance, polar.at(middle + 0.5).theta;

        filterLeft = Kalman(x, y, yaw, left, initialCovariance, modelError, measurementError);
        filterRight = Kalman(x, y, yaw, right, initialCovariance, modelError, measurementError);
    } else {
        Eigen::MatrixXd farLeft(2, 1), left(2, 1), right(2, 1), farRight(2, 1);
        farLeft << polar.at(middle - 1.5).distance, polar.at(middle - 1.5).theta;
        left << polar.at(middle - 0.5).distance, polar.at(middle - 0.5).theta;
        right << polar.at(middle + 0.5).distance, polar.at(middle + 0.5).theta;
        farRight << polar.at(middle + 1.5).distance, polar.at(middle + 1.5).theta;

        filterFarLeft = Kalman(x, y, yaw, farLeft, initialCovariance, modelError, measurementError);
        filterLeft = Kalman(x, y, yaw, left, initialCovariance, modelError, measurementError);
        filterRight = Kalman(x, y, yaw, right, initialCovariance, modelError, measurementError);
        filterFarRight = Kalman(x, y, yaw, farRight, initialCovariance, modelError, measurementError);
    }

    ros::spin();

    return 0;
}