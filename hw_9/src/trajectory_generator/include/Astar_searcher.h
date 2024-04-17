#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"

class AstarPathFinder
{
	private:
	const std::vector<Eigen::Vector3i> expand_dirs = {
	 {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1}, {-1, 1, 0}, {-1, 1, 1},
	 {0, -1, -1}, {0, -1, 0}, {0, -1, 1}, {0, 0, -1}, {0, 0, 1}, {0, 1, -1}, {0, 1, 0}, {0, 1, 1},
	 {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1}, {1, 0, 0}, {1, 0, 1}, {1, 1, -1}, {1, 1, 0}, {1, 1, 1}
	};

	const std::vector<double> expand_cost = { 1.732, 1.414, 1.732, 1.414, 1, 1.414, 1.732, 1.414, 1.732,
									   1.414, 1, 1.414, 1, 1, 1.414, 1, 1.414,
									   1.732, 1.414, 1.732, 1.414, 1, 1.414, 1.732, 1.414, 1.732 };


	protected:
	uint8_t* data;
	GridNodePtr*** GridNodeMap;
	Eigen::Vector3i goalIdx;
	int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
	int GLXYZ_SIZE, GLYZ_SIZE;

	double resolution, inv_resolution;
	double gl_xl, gl_yl, gl_zl;
	double gl_xu, gl_yu, gl_zu;

	GridNodePtr terminatePtr;
	std::multimap<double, GridNodePtr> openSet;

	double getHeu(GridNodePtr node1, GridNodePtr node2);
	void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr>& neighborPtrSets, std::vector<double>& edgeCostSets);

	bool isOccupied(const int& idx_x, const int& idx_y, const int& idx_z) const;
	bool isOccupied(const Eigen::Vector3i& index) const;
	bool isFree(const int& idx_x, const int& idx_y, const int& idx_z) const;
	bool isFree(const Eigen::Vector3i& index) const;

	Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index);
	Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt);

	// qsl add
	std::vector<Eigen::Vector3d> RDP(const std::vector<Eigen::Vector3d>& points, double epsilon);
	double perpendicularDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);


	public:
	AstarPathFinder() {};
	~AstarPathFinder() {};
	void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
	void resetGrid(GridNodePtr ptr);
	void resetUsedGrids();

	void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
	void setObs(const double coord_x, const double coord_y, const double coord_z);

	Eigen::Vector3d coordRounding(const Eigen::Vector3d& coord);
	std::vector<Eigen::Vector3d> getPath();
	std::vector<Eigen::Vector3d> getVisitedNodes();
	std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d>& path, const double path_resolution);
	Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t);
	int safeCheck(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);
};

#endif