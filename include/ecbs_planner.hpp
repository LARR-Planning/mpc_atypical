#pragma once

#include "init_traj_planner.hpp"
#include <environment.hpp>

using namespace libMultiRobotPlanning;

namespace libCorridorGen {
    class ECBSPlanner : public InitTrajPlanner {
    public:
        ECBSPlanner(std::shared_ptr<octomap::OcTree> _octree_obj,
                    Param _param)
                : InitTrajPlanner(std::move(_octree_obj),
                                  std::move(_param)) {
            setObstacles();
            setWaypoints();
        }

        bool update(bool log) override {
            Environment mapf(dimx, dimy, dimz, ecbs_obstacles, ecbs_goalLocations,
                             std::vector<double>{param.agent_xy_size}, param.grid_xy_res);
            ECBS<State, Action, int, Conflict, Constraints, Environment> ecbs(mapf, 1);
            std::vector<PlanResult<State, Action, int>> solution;

            // Execute ECBS algorithm
            bool success = ecbs.search(ecbs_startStates, solution);
            if (!success) {
                ROS_ERROR("libCorridorGen: ECBS Failed!");
                return false;
            }

            // Update segment time
            int cost = 0;
            int makespan = 0;
            for (const auto &s : solution) {
                cost += s.cost;
                makespan = std::max<int>(makespan, s.cost);
            }
            for (int i = 0; i <= makespan + 2; i++) {
                T.emplace_back(i * param.time_step);
            }

            // Append start, goal points to both ends respectively
            for (size_t a = 0; a < solution.size(); ++a) {
//            initTraj.emplace_back(octomap::point3d(mission.startState[0],
//                                                   mission.startState[1],
//                                                   mission.startState[2]));
                initTraj.emplace_back(octomap::point3d(param.mission_start_x,
                                                       param.mission_start_y,
                                                       0.2)); //TODO: floor offset

                for (const auto &state : solution[a].states) {
                    initTraj.emplace_back(octomap::point3d(state.first.x * param.grid_xy_res + grid_x_min,
                                                           state.first.y * param.grid_xy_res + grid_y_min,
                                                           0.2));
                }
                while (initTraj.size() <= makespan + 2) {
                    initTraj.emplace_back(octomap::point3d(param.mission_goal_x,
                                                           param.mission_goal_y,
                                                           0.2));
//                initTraj.emplace_back(octomap::point3d(mission.goalState[0],
//                                                       mission.goalState[1],
//                                                       mission.goalState[2]));
                }
            }
            return true;
        }

    private:
        std::unordered_set<Location> ecbs_obstacles;
        std::vector<State> ecbs_startStates;
        std::vector<Location> ecbs_goalLocations;

        bool isObstacleInBox(const std::vector<double> &box, double margin) {
            for (octomap::OcTree::leaf_bbx_iterator it = octree_obj->begin_leafs_bbx(
                    octomap::point3d(box[0], box[1], box[2]),
                    octomap::point3d(box[3], box[4], box[5])),
                         end = octree_obj->end_leafs_bbx(); it != end; ++it) {
                if (octree_obj->isNodeOccupied(*it)) {
                    return true;
                }
            }

            return false;
        }

        // Find the location of obstacles in grid-space
        bool setObstacles() {
            int x, y, z;
            std::vector<double> box;
            box.resize(6);
            for (double i = grid_x_min; i <= grid_x_max; i += param.grid_xy_res) {
                for (double j = grid_y_min; j <= grid_y_max; j += param.grid_xy_res) {
                    if (i - param.agent_xy_size < param.world_x_min) {
                        box[0] = param.world_x_min;
                    } else {
                        box[0] = i - param.agent_xy_size;
                    }
                    if (j - param.agent_xy_size < param.world_y_min) {
                        box[1] = param.world_y_min;
                    } else {
                        box[1] = j - param.agent_xy_size;
                    }
                    box[2] = param.world_z_min;

                    if (i + param.agent_xy_size > param.world_x_max) {
                        box[3] = param.world_x_max;
                    } else {
                        box[3] = i + param.agent_xy_size;
                    }
                    if (j + param.agent_xy_size > param.world_y_max) {
                        box[4] = param.world_y_max;
                    } else {
                        box[4] = j + param.agent_xy_size;
                    }
                    box[5] = param.world_z_max;

                    if (isObstacleInBox(box, param.agent_xy_size)) {
                        x = round((i - grid_x_min) / param.grid_xy_res);
                        y = round((j - grid_y_min) / param.grid_xy_res);
                        ecbs_obstacles.insert(Location(x, y, 0));
                    } else {
                        int debug = 1;
                    }
                }
            }

            return true;
        }

        // Set start, goal points of ECBS
        bool setWaypoints() {
            int xig, yig, zig, xfg, yfg, zfg;

            // For start, goal point of ECBS, we use the nearest grid point.
            xig = (int) round((param.mission_start_x - grid_x_min) / param.grid_xy_res);
            yig = (int) round((param.mission_start_y - grid_y_min) / param.grid_xy_res);
//        zig = (int)round((mission.startState[2] - grid_z_min) / param.grid_z_res);
            zig = 0;
            xfg = (int) round((param.mission_goal_x - grid_x_min) / param.grid_xy_res);
            yfg = (int) round((param.mission_goal_y - grid_y_min) / param.grid_xy_res);
//        zfg = (int)round((mission.goalState[2] - grid_z_min) / param.grid_z_res);
            zfg = 0;

            if (ecbs_obstacles.find(Location(xig, yig, zig)) != ecbs_obstacles.end()) {
                ROS_ERROR_STREAM("ECBSPlanner: start of agent is occluded by obstacle");
                return false;
            }
            if (ecbs_obstacles.find(Location(xfg, yfg, zfg)) != ecbs_obstacles.end()) {
                ROS_ERROR_STREAM("ECBSLauncher: goal of agent is occluded by obstacle");
                return false;
            }

            ecbs_startStates.emplace_back(State(0, xig, yig, zig));
            ecbs_goalLocations.emplace_back(Location(xfg, yfg, zfg));
            return true;
        }
    };
}