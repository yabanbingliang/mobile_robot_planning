#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
  Vector3d global_xyz_u, int max_x_id,
  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr * *[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

// void AstarPathFinder::setObs(const double coord_x, const double coord_y,
//   const double coord_z) {
//   if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
//     coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
//     return;

//   int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
//   int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
//   int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

//   if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
//     idx_y == GLY_SIZE)
//     data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
//   else {
//     data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
//     data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
//     data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
//     data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
//     data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
//     data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
//     data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
//     data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
//     data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
//   }
// }

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
    coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}


vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
          -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i& index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d& pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
    min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
    min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d& coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i& index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i& index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int& idx_x, const int& idx_y,
  const int& idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
    idx_z >= 0 && idx_z < GLZ_SIZE &&
    (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int& idx_x, const int& idx_y,
  const int& idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
    idx_z >= 0 && idx_z < GLZ_SIZE &&
    (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
  vector<GridNodePtr>& neighborPtrSets,
  vector<double>& edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;

  for (size_t i = 0; i < expand_dirs.size(); i++)
  {
    Vector3i node_index = currentPtr->index;
    node_index += expand_dirs.at(i);

    if (!isFree(node_index))
      continue;
    auto nodeptr = GridNodeMap[node_index(0)][node_index(1)][node_index(2)];
    if (-1 == nodeptr->id)
      continue;

    neighborPtrSets.push_back(nodeptr);
    edgeCostSets.push_back(expand_cost.at(i) * resolution);
  }

  // old
  // for (int dx = -1; dx < 2; dx++) {
  //   for (int dy = -1; dy < 2; dy++) {
  //     for (int dz = -1; dz < 2; dz++) {

  //       if (dx == 0 && dy == 0 && dz == 0)
  //         continue;

  //       neighborIdx(0) = (currentPtr->index)(0) + dx;
  //       neighborIdx(1) = (currentPtr->index)(1) + dy;
  //       neighborIdx(2) = (currentPtr->index)(2) + dz;

  //       if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
  //         neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
  //         neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
  //         continue;
  //       }

  //       neighborPtrSets.push_back(
  //         GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
  //       edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
  //     }
  //   }
  // }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.

  double dx, dy, dz, hScore;
  dx = std::abs(node1->coord.x() - node2->coord.x());
  dy = std::abs(node1->coord.y() - node2->coord.y());
  dz = std::abs(node1->coord.z() - node2->coord.z());

  // Diagonal Heuristic
  double min_xyz = std::min({ dx, dy, dz });
  hScore = dx + dy + dz + (std::sqrt(3.0) - 3) * min_xyz;

  // Euclidean 
  // hScore = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));

  // no tie_breaker
  // hScore = hScore * (1 + 1.0 / 10);

  // cout << "hScore = " << hScore << endl;

  return hScore;
}


void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
  ros::Time time_1 = ros::Time::now();

  //index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);

  goalIdx = end_idx;

  //position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  //Initialize the pointers of struct GridNode which represent start node and goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  //openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  //put start node in open set
  startPtr->gScore = 0;
  startPtr->fScore = getHeu(startPtr, endPtr);
  //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));
  /*
  *
  STEP 2 :  some else preparatory works which should be done before while loop
  please write your code below
  *
  *
  */


  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;


  // this is the main loop
  while (!openSet.empty()) {
    /*
    *
    *
    step 3: Remove the node with lowest cost function from open set to closed set
    please write your code below

    IMPORTANT NOTE!!!
    This part you should use the C++ STL: multimap, more details can be find in Homework description
    *
    *
    */

    auto iter = openSet.begin();
    currentPtr = (*iter).second;
    currentPtr->id = -1;
    openSet.erase((*iter).first);


    // if the current node is the goal 
    if (currentPtr->index == goalIdx) {
      ros::Time time_2 = ros::Time::now();
      terminatePtr = currentPtr;
      ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution);
      return;
    }
    //get the succetion
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

    /*
    *
    *
    STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
    please write your code below
    *
    */

    for (int i = 0; i < (int)neighborPtrSets.size(); i++) {
      /*
      *
      *
      Judge if the neigbors have been expanded
      please write your code below

      IMPORTANT NOTE!!!
      neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
      neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
      *
      */
      neighborPtr = neighborPtrSets.at(i);
      double gScore = currentPtr->gScore + edgeCostSets.at(i);
      double hScore = getHeu(neighborPtr, endPtr);
      double fScore = gScore + hScore;

      if (neighborPtr->id == 0) { //discover a new node, which is not in the closed set and open set
        /*
        *
        *
        STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
        please write your code below
        *
        */

        neighborPtr->gScore = gScore;
        neighborPtr->fScore = fScore;
        neighborPtr->id = 1; // OPEN set
        neighborPtr->cameFrom = currentPtr;
        openSet.insert(std::make_pair(neighborPtr->fScore, neighborPtr));

        continue;
      }
      else if (neighborPtr->id == 1) { //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
        /*
        *
        *
        STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
        please write your code below
        *
        */
        if (neighborPtr->gScore > gScore) {

          neighborPtr->gScore = gScore;
          neighborPtr->fScore = fScore;
          neighborPtr->cameFrom = currentPtr;
          openSet.insert(std::make_pair(neighborPtr->fScore, neighborPtr));
        }

        continue;
      }
      else {//this node is in closed set
        /*
        *
        please write your code below
        *
        */
        continue;
      }
    }
  }

  // if (currentPtr->index != goalIdx) {
  //   ROS_WARN("[A*]{fail}");
  //   if (currentPtr != nullptr) {
  //     terminatePtr = currentPtr;
  //   }
  // }

  //if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec());
}


vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/

  GridNodePtr tmpPtr = terminatePtr;
  while (tmpPtr->cameFrom != nullptr) {
    gridPath.push_back(tmpPtr);
    tmpPtr = tmpPtr->cameFrom;
  }

  for (auto ptr : gridPath)
    path.push_back(ptr->coord);

  reverse(path.begin(), path.end());

  return path;
}

vector<Vector3d> AstarPathFinder::RDP(const vector<Vector3d>& points, double epsilon) {
  if (points.size() < 3) {
    return points;
  }

  double dmax = 0.0;
  int index = 0;
  int end = points.size() - 1;

  for (int i = 1; i < end; i++) {
    double d = perpendicularDistance(points[i], points[0], points[end]);
    if (d > dmax) {
      index = i;
      dmax = d;
    }
  }

  vector<Vector3d> result;

  if (dmax > epsilon) {
    vector<Vector3d> recResults1 = RDP(vector<Vector3d>(points.begin(), points.begin() + index + 1), epsilon);
    vector<Vector3d> recResults2 = RDP(vector<Vector3d>(points.begin() + index, points.end()), epsilon);

    result.insert(result.end(), recResults1.begin(), recResults1.end() - 1);
    result.insert(result.end(), recResults2.begin(), recResults2.end());
  }
  else {
    result.push_back(points[0]);
    result.push_back(points[end]);
  }

  return result;
}


//wrong
// double AstarPathFinder::perpendicularDistance(const Vector3d& p, const Vector3d& p1, const Vector3d& p2) { 
//   double x = p1.x() - p2.x();
//   double y = p1.y() - p2.y();
//   double z = p1.z() - p2.z();

//   double num = abs(x * (p1.y() - p.y()) - (p1.x() - p.x()) * y + p1.z() * (p1.x() - p.x()) - p1.x() * (p1.z() - p.z()));
//   double den = sqrt(x * x + y * y + z * z);

//   return num / den;
// }

double AstarPathFinder::perpendicularDistance(const Vector3d& p, const Vector3d& p1, const Vector3d& p2) {
  // 计算直线方向向量
  Vector3d lineVec = p2 - p1;
  // 计算p到p1的向量和p到p2的向量的叉乘的模长
  double num = ((p - p1).cross(p - p2)).norm();
  // 计算直线方向向量的模长
  double den = lineVec.norm();

  return num / den;
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d>& path,
  double path_resolution) {
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/

  subPath = RDP(path, path_resolution);

  return subPath;
}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/

  return unsafe_segment;
}