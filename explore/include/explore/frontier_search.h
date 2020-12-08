#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double cost;
  geometry_msgs::Point initial;
  geometry_msgs::Point centroid;
  geometry_msgs::Point middle;
  std::vector<geometry_msgs::Point> points;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch()
  {
  }

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale,
				 double gain_scale, double orientation_scale, double min_frontier_size, int max_cell_cost);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param robot_pose Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(const geometry_msgs::Pose& robot_pose);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @param robot_pose the current pose of the robot.
   * @param robot_orientation the current robot orientation in the 2D map, used for adding the orientation cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier, const geometry_msgs::Pose& robot_pose, const Eigen::Vector2d& robot_orientation);

private:
  costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_, orientation_scale_;
  double min_frontier_size_;
  int max_cell_cost_; // maximal cost a costmap cell is allowed to have in order to consider it for the breadth-first search
};
}
#endif
