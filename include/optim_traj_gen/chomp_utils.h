
/**
 * @file chomp_utils.h
 * @author JBS (junbs95@gmail.com)
 * @brief This scirpts define necessary operation for CHOMP (root header)
 * @version 0.1
 * @date 2019-10-04
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <voxblox_ros/esdf_server.h>
#include <cmath>
#include <vector>
#include <functional>
#include <string>
#include <chrono>
#include <eigen3/Eigen/LU>
#include <qpOASES.hpp>
#include <numeric>
#include <tf/transform_broadcaster.h>

// linear regression 
struct LinearModel{
    // x=beta0+beta1*t
    double beta0;
    double beta1;
};
LinearModel linear_regression(const Eigen::VectorXd& ts,const Eigen::VectorXd& xs);
double model_eval(const LinearModel& model,double t);

// conversion between nav_msgs::Path to vector set x/y/z
void path2vec(const nav_msgs::Path& path,std::vector<double> &xs,std::vector<double> &ys,std::vector<double> &zs);
void vec2path(std::vector<double> &xs,std::vector<double> &ys,std::vector<double> &zs,nav_msgs::Path& path);
Eigen::VectorXd get_time_stamps_from_nav_path(const nav_msgs::Path& path);
double interpolate( Eigen::VectorXd &xData, Eigen::VectorXd &yData, double x, bool extrapolate );

