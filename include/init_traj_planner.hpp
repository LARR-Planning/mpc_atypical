#pragma once

#include <corridor_common.hpp>
#include <param.hpp>

namespace libCorridorGen {
    class InitTrajPlanner {
    public:
        initTraj_t initTraj; // discrete initial trajectory: pi_0,...,pi_M
        std::vector<double> T; // segment time: T_0,...,T_M

        virtual bool update(bool log) = 0;

        InitTrajPlanner(std::shared_ptr<octomap::OcTree> _octree_obj,
                        Param _param)
                : octree_obj(std::move(_octree_obj)),
                  param(std::move(_param)) {
            grid_x_min = ceil((param.world_x_min + SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
            grid_y_min = ceil((param.world_y_min + SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;

            grid_x_max = floor((param.world_x_max - SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
            grid_y_max = floor((param.world_y_max - SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;

            dimx = (int) round((grid_x_max - grid_x_min) / param.grid_xy_res) + 1;
            dimy = (int) round((grid_y_max - grid_y_min) / param.grid_xy_res) + 1;
            dimz = 1;
        }

    protected:
        std::shared_ptr<octomap::OcTree> octree_obj;
        Param param;

        double grid_x_min, grid_y_min, grid_x_max, grid_y_max;
        int dimx, dimy, dimz;
    };
}