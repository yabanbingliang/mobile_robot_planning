#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Useful customized headers
#include "Astar_searcher.h"
#include "backward.hpp"
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint* _trajGene = new TrajectoryGeneratorWaypoint();
AstarPathFinder* _astar_path_finder = new AstarPathFinder();

// Set the obstacle map
double _resolution, _inv_resolution, _path_resolution;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub;
ros::Publisher _traj_vis_pub, _traj_pub, _path_vis_pub, _path_a_star_vis_pub, _replan_start_vis_pub;

// for planning
Vector3d odom_pt, odom_vel, start_pt, target_pt, start_vel;
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
double time_duration;
ros::Time time_traj_start;
bool has_odom = false;
bool has_target = false;

// for replanning
enum STATE {
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  EXEC_TRAJ,
  REPLAN_TRAJ
} exec_state = STATE::INIT;
double no_replan_thresh, replan_thresh;
ros::Timer _exec_timer;
void execCallback(const ros::TimerEvent& e);

// declare
void changeState(STATE new_state, string pos_call);
void printState();
void visTrajectory(MatrixXd polyCoeff, VectorXd time);
void visPath(MatrixXd nodes);
void visAStarPath(MatrixXd nodes);

void visEmptyTrajectory();
void visEmptyPath();
void visEmptyAStarPath();

void trajOptimization(Eigen::MatrixXd path);
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
void rcvWaypointsCallback(const nav_msgs::Path& wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map);
void trajPublish(MatrixXd polyCoeff, VectorXd time);
void trajClear();
bool trajGeneration();

void visualizeStartPoint(const geometry_msgs::Point& start_pt, int color = 0);

VectorXd timeAllocation(MatrixXd Path);
Vector3d getPos(double t_cur);
Vector3d getVel(double t_cur);

void visualizeStartPoint(const Vector3d& start_pt, int color = 1) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "replan_start_point";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = start_pt.x();
  marker.pose.position.y = start_pt.y();
  marker.pose.position.z = start_pt.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  if (color == 0) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  else if (color == 1) {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }


  _replan_start_vis_pub.publish(marker);
}

// change the state to the new state
void changeState(STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ" };
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
    state_str[int(new_state)]
    << endl;
}

void printState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ" };
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}

void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  odom_pt(0) = odom->pose.pose.position.x;
  odom_pt(1) = odom->pose.pose.position.y;
  odom_pt(2) = odom->pose.pose.position.z;

  odom_vel(0) = odom->twist.twist.linear.x;
  odom_vel(1) = odom->twist.twist.linear.y;
  odom_vel(2) = odom->twist.twist.linear.z;

  has_odom = true;
}

// Control the State changes
void execCallback(const ros::TimerEvent& e) {
  static int num = 0;
  num++;
  if (num == 100) {
    printState();
    if (!has_odom)
      cout << "no odom." << endl;
    if (!has_target)
      cout << "wait for goal." << endl;
    num = 0;
  }

  switch (exec_state) {
  case INIT: {
    if (!has_odom)
      return;
    if (!has_target)
      return;
    changeState(WAIT_TARGET, "STATE");
    break;
  }

  case WAIT_TARGET: {
    if (!has_target)
      return;
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case GEN_NEW_TRAJ: {
    bool success = trajGeneration();
    if (success)
    {
      changeState(EXEC_TRAJ, "STATE");

      // for debug
      // changeState(WAIT_TARGET, "STATE");
      // has_target = false;
    }

    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case EXEC_TRAJ: {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_replan = ros::Duration(1, 0).toSec();
    t_cur = min(time_duration, t_cur);

    // cout << "t_cur = " << t_cur << endl;
    // cout << "time_duration = " << time_duration << endl;

    if (t_cur > time_duration - 1e-2) {
      has_target = false;
      changeState(WAIT_TARGET, "STATE");
      return;
    }
    else if ((target_pt - odom_pt).norm() < no_replan_thresh) {
      return;
    }
    else if ((start_pt - odom_pt).norm() < replan_thresh) {
      return;
    }
    else if (t_cur < t_replan) {
      return;
    }
    else {
      cout << "############################################" << endl;
      cout << "time_duration = " << time_duration << endl;
      cout << "no_replan_thresh = " << no_replan_thresh << endl;
      cout << "replan_thresh = " << replan_thresh << endl;
      cout << "t_cur = " << t_cur << endl;
      cout << "t_replan = " << t_replan << endl;
      cout << "(target_pt - odom_pt).norm() = " << (target_pt - odom_pt).norm() << endl;
      cout << "(start_pt - odom_pt).norm() = " << (start_pt - odom_pt).norm() << endl;
      cout << "############################################" << endl;
      changeState(REPLAN_TRAJ, "STATE");
    }
    break;
  }
  case REPLAN_TRAJ: {
    // MatrixXd _polyCoeff_empty;
    // VectorXd _polyTime_empty;
    // trajPublish(_polyCoeff_empty, _polyTime_empty);

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_delta = ros::Duration(0, 50).toSec();
    t_cur = t_delta + t_cur;
    start_pt = getPos(t_cur);
    start_vel = getVel(t_cur);


    visualizeStartPoint(start_pt);

    ROS_WARN("[replan] start_pt = %f, %f, %f", start_pt(0), start_pt(1), start_pt(2));
    ROS_WARN("[replan] start_vel = %f, %f, %f", start_vel(0), start_vel(1), start_vel(2));
    ROS_WARN("[replan] target_pt = %f, %f, %f", target_pt(0), target_pt(1), target_pt(2));

    // trajClear();
    // ROS_WARN("clear trajPublish");
    // visEmptyTrajectory();
    // visEmptyPath();
    // visEmptyAStarPath();
    // ROS_WARN("all vis empty");

    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }
  }
}

void rcvWaypointsCallBack(const nav_msgs::Path& wp) {
  if (wp.poses[0].pose.position.z < 0.0)
    return;
  target_pt << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y,
    wp.poses[0].pose.position.z;
  ROS_INFO("[node] receive the planning target");
  start_pt = odom_pt;
  start_vel = odom_vel;
  has_target = true;

  if (exec_state == WAIT_TARGET)
    changeState(GEN_NEW_TRAJ, "STATE");
  else if (exec_state == EXEC_TRAJ)
    changeState(REPLAN_TRAJ, "STATE");
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map) {

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;

  pcl::fromROSMsg(pointcloud_map, cloud);

  if ((int)cloud.points.size() == 0)
    return;

  pcl::PointXYZ pt;
  for (int idx = 0; idx < (int)cloud.points.size(); idx++) {
    pt = cloud.points[idx];
    // set obstalces into grid map for path planning
    _astar_path_finder->setObs(pt.x, pt.y, pt.z);
  }
}

// trajectory generation: front-end + back-end
// front-end : A* search method
// back-end  : Minimum snap trajectory generation
bool trajGeneration() {
  /**
   *
   * STEP 1:  search the path and get the path
   *
   * **/
  _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
  auto grid_path = _astar_path_finder->getPath();

  // try to compare with RDP algorithm
  vector<Vector3d> a_star_grid_path = grid_path;
  MatrixXd a_star_path(int(a_star_grid_path.size()), 3);
  for (int k = 0; k < int(a_star_grid_path.size()); k++) {
    a_star_path.row(k) = a_star_grid_path[k];
  }
  visAStarPath(a_star_path);

  // Print grid_path
  // for (int i = 0; i < grid_path.size(); i++) {
  //   std::cout << "grid_path[" << i << "]: " << grid_path[i] << std::endl;
  // }

  // Print grid_path

  // for (int i = 0; i < grid_path.size(); i++) {
  //   std::cout << "grid_path[" << i << "]: ";
  //   for (int j = 0; j < grid_path[i].size(); j++) {
  //     std::cout << grid_path[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }



  // Reset map for next call

  /**
   *
   * STEP 2:  Simplify the path: use the RDP algorithm
   *
   * **/
  grid_path = _astar_path_finder->pathSimplify(grid_path, _path_resolution);

  // std::cout << "_path_resolution = " << _path_resolution << std::endl;

  // for (int i = 0; i < grid_path.size(); i++) {
  //   std::cout << "grid_path_simplify[" << i << "]: ";
  //   for (int j = 0; j < grid_path[i].size(); j++) {
  //     std::cout << grid_path[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }


  MatrixXd path(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    path.row(k) = grid_path[k];
  }

  // cout << "_dev_order = " << _dev_order << std::endl;

  /**
   *
   * STEP 3:  Trajectory optimization
   *
   * **/
  trajOptimization(path);
  time_duration = _polyTime.sum();

  // Publish the trajectory
  trajPublish(_polyCoeff, _polyTime);

  // record the trajectory start time
  time_traj_start = ros::Time::now();

  // cout << "time_traj_start = " << time_traj_start << endl;

  // return if the trajectory generation successes
  if (_polyCoeff.rows() > 0)
    return true;
  else
    return false;
}

void trajOptimization(Eigen::MatrixXd path) {
  // if( !has_odom ) return;
  MatrixXd vel = MatrixXd::Zero(2, 3);
  MatrixXd acc = MatrixXd::Zero(2, 3);

  vel.row(0) = start_vel;

  /**
   *
   * STEP 3.1:  finish the timeAllocation() using resonable allocation
   *
   * **/
  _polyTime = timeAllocation(path);

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
   // minimum-snap
  _polyCoeff =
    _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

  // check if the trajectory is safe, if not, do reoptimize
  int unsafe_segment;

  /**
   *
   * STEP 3.3:  finish the safeCheck()
   *
   * **/
  unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);

  MatrixXd repath = path;
  int count = 0;
  while (unsafe_segment != -1) {
    /**
     *
     * STEP 3.4:  reoptimize
     * the method comes from: "Polynomial Trajectory
     * Planning for Aggressive Quadrotor Flight in Dense Indoor Environment"
     * part 3.5
     *
     * **/
  }
  // visulize path and trajectory
  visPath(repath);
  visTrajectory(_polyCoeff, _polyTime);
}

void trajClear() {
  unsigned int count = 1; // 第一个轨迹的 ID 必须大于 0

  quadrotor_msgs::PolynomialTrajectory traj_msg;

  traj_msg.header.seq = count;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = "world";
  traj_msg.trajectory_id = count;
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

  traj_msg.num_order = 0; // 设置为 0 表示空的轨迹
  traj_msg.num_segment = 0;

  traj_msg.start_yaw = 0.0; // 空的轨迹，起始和终止角度都设为 0
  traj_msg.final_yaw = 0.0;

  traj_msg.mag_coeff = 1; // 这个字段的含义不明，暂时保持不变

  ROS_WARN("[trajClear] Publishing empty trajectory.");
  _traj_pub.publish(traj_msg);
}


void trajPublish(MatrixXd polyCoeff, VectorXd time) {
  if (polyCoeff.size() == 0 || time.size() == 0) {
    ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to "
      "publish.");
    return;
  }

  unsigned int poly_number;

  static int count =
    1; // The first trajectory_id must be greater than 0. zxzxzxzx

  quadrotor_msgs::PolynomialTrajectory traj_msg;

  traj_msg.header.seq = count;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = std::string("world"); // delete " / "
  traj_msg.trajectory_id = count;
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

  traj_msg.num_order = 2 * _dev_order - 1; // the order of polynomial
  traj_msg.num_segment = time.size();

  Vector3d initialVel, finalVel;
  initialVel = _trajGene->getVelPoly(_polyCoeff, 0, 0);
  finalVel = _trajGene->getVelPoly(_polyCoeff, traj_msg.num_segment - 1,
    _polyTime(traj_msg.num_segment - 1));
  traj_msg.start_yaw = atan2(initialVel(1), initialVel(0));
  traj_msg.final_yaw = atan2(finalVel(1), finalVel(0));

  poly_number = traj_msg.num_order + 1;
  // cout << "p_order:" << poly_number << endl;
  // cout << "traj_msg.num_order:" << traj_msg.num_order << endl;
  // cout << "traj_msg.num_segment:" << traj_msg.num_segment << endl;
  for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
    for (unsigned int j = 0; j < poly_number; j++) {
      traj_msg.coef_x.push_back(polyCoeff(i, j) * pow(time(i), j));
      traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j) *
        pow(time(i), j));
      traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j) *
        pow(time(i), j));
    }
    traj_msg.time.push_back(time(i));
    traj_msg.order.push_back(traj_msg.num_order);
  }
  traj_msg.mag_coeff = 1;

  count++;
  ROS_WARN("[traj..gen...node] traj_msg publish");
  _traj_pub.publish(traj_msg);
}


double timeTrapzVel(const double dist,
  const double vel,
  const double acc)
{
  const double t = vel / acc;
  const double d = 0.5 * acc * t * t;

  if (dist < d + d)
  {
    return 2.0 * sqrt(dist / acc);
  }
  else
  {
    return 2.0 * t + (dist - 2.0 * d) / vel;
  }
}

VectorXd timeAllocation(MatrixXd Path) {
  VectorXd time(Path.rows() - 1);

  // cout << "_Vel = " << _Vel << endl; // 3
  // cout << "_Acc = " << _Acc << endl; // 2

  for (int i = 0; i < time.size(); i++) {
    double dist = 0.0;
    for (int j = 0; j < Path.cols(); ++j) {
      double diff = Path(i + 1, j) - Path(i, j);
      dist += diff * diff;
    }
    dist = sqrt(dist);
    time(i) = timeTrapzVel(dist, _Vel, _Acc);
  }

  cout << "time: " << endl;
  for (int i = 0; i < time.size(); i++) {
    std::cout << "time[" << i << "]: ";
    cout << time(i) << endl;
  }

  return time;
}

// for debug visEmpty
void visEmptyTrajectory() {
  visualization_msgs::Marker traj_vis_msg;

  traj_vis_msg.header.stamp = ros::Time::now();
  traj_vis_msg.header.frame_id = "world";

  traj_vis_msg.ns = "traj_node/trajectory";
  traj_vis_msg.id = 0;
  traj_vis_msg.type = visualization_msgs::Marker::DELETE;
  traj_vis_msg.action = visualization_msgs::Marker::DELETE;

  _traj_vis_pub.publish(traj_vis_msg);
}

void visEmptyPath() {
  visualization_msgs::Marker path_msg;

  int id = 0;
  path_msg.id = id;
  path_msg.type = visualization_msgs::Marker::DELETE;
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = ros::Time::now();
  path_msg.ns = "traj_node/path";
  path_msg.action = visualization_msgs::Marker::DELETE;

  _path_vis_pub.publish(path_msg);
}

void visEmptyAStarPath() {
  visualization_msgs::Marker path_msg;

  int id = 0;
  path_msg.id = id;
  path_msg.type = visualization_msgs::Marker::DELETE;
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = ros::Time::now();
  path_msg.ns = "traj_node/a_start_path";
  path_msg.action = visualization_msgs::Marker::DELETE;

  _path_a_star_vis_pub.publish(path_msg);
}


void visTrajectory(MatrixXd polyCoeff, VectorXd time) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp = ros::Time::now();
  _traj_vis.header.frame_id = "world";

  _traj_vis.ns = "traj_node/trajectory";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 0.0;
  _traj_vis.color.g = 0.5;
  _traj_vis.color.b = 1.0;

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < time(i); t += 0.01) {
      pos = _trajGene->getPosPoly(polyCoeff, i, t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      _traj_vis.points.push_back(pt);
    }
  }
  _traj_vis_pub.publish(_traj_vis);
}

void visPath(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 1.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _path_vis_pub.publish(points);
}

// vis A* path for comparison with RDP
void visAStarPath(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/a_start_path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 1.0; // change color
  points.color.b = 1.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _path_a_star_vis_pub.publish(points);
}

Vector3d getPos(double t_cur) {
  double time = 0;
  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        pos = _trajGene->getPosPoly(_polyCoeff, i, t); // change t 2 time
        return pos;
      }
    }
  }
  return pos;
}

Vector3d getVel(double t_cur) {
  double time = 0;
  Vector3d Vel = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        Vel = _trajGene->getVelPoly(_polyCoeff, i, t); // change t 2 time
        return Vel;
      }
    }
  }
  return Vel;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_node");
  ros::NodeHandle nh("~");

  nh.param("planning/vel", _Vel, 1.0);
  nh.param("planning/acc", _Acc, 1.0);
  nh.param("planning/dev_order", _dev_order, 3);
  nh.param("planning/min_order", _min_order, 3);
  nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
  nh.param("map/resolution", _resolution, 0.2);
  nh.param("map/x_size", _x_size, 50.0);
  nh.param("map/y_size", _y_size, 50.0);
  nh.param("map/z_size", _z_size, 5.0);
  nh.param("path/resolution", _path_resolution, 0.05);
  nh.param("replanning/thresh_replan", replan_thresh, -1.0);
  nh.param("replanning/thresh_no_replan", no_replan_thresh, -1.0);

  _poly_num1D = 2 * _dev_order;

  _exec_timer = nh.createTimer(ros::Duration(0.01), execCallback);

  _odom_sub = nh.subscribe("odom", 10, rcvOdomCallback);
  _map_sub = nh.subscribe("local_pointcloud", 1, rcvPointCloudCallBack);
  _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);

  _traj_pub =
    nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
  _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
  _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);
  _path_a_star_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_a_star_path", 1);
  _replan_start_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_replan_start", 1);

  // set the obstacle map
  _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
  _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;
  _inv_resolution = 1.0 / _resolution;
  _max_x_id = (int)(_x_size * _inv_resolution);
  _max_y_id = (int)(_y_size * _inv_resolution);
  _max_z_id = (int)(_z_size * _inv_resolution);

  _astar_path_finder = new AstarPathFinder();
  _astar_path_finder->initGridMap(_resolution, _map_lower, _map_upper,
    _max_x_id, _max_y_id, _max_z_id);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}