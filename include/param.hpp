#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <string>

namespace libCorridorGen {
    class Param {
    public:
        bool log;
        std::string package_path;

        double agent_xy_size;
        double agent_z_size;
        double agent_speed;

        double world_x_min;
        double world_y_min;
        double world_z_min;
        double world_x_max;
        double world_y_max;
        double world_z_max;

        double grid_xy_res;
        double grid_z_res;
        double grid_margin;

        double box_xy_res;
        double box_z_res;

        double mission_start_x;
        double mission_start_y;
        double mission_goal_x;
        double mission_goal_y;

        double time_step;
        std::vector<double> color;

        bool init(const ros::NodeHandle &nh);
    };

    bool Param::init(const ros::NodeHandle &nh) {
        nh.param<bool>("log", log, false);

        nh.param<double>("agent/xy_size", agent_xy_size, 0.25); // agent xy-plane radius
        nh.param<double>("agent/z_size", agent_z_size, 0.1); // height
        nh.param<double>("agent/speed", agent_speed, 1); // agent speed

        nh.param<double>("world/x_min", world_x_min, -10);
        nh.param<double>("world/y_min", world_y_min, -10);
        nh.param<double>("world/z_min", world_z_min, 0.2);
        nh.param<double>("world/x_max", world_x_max, 10);
        nh.param<double>("world/y_max", world_y_max, 10);
        nh.param<double>("world/z_max", world_z_max, 2.5);

        // A* grid configuration
        nh.param<double>("grid/xy_res", grid_xy_res, 0.5); // xy-plane resolution
        nh.param<double>("grid/z_res", grid_z_res, 1.0);
        nh.param<double>("grid/margin", grid_margin, 0.2); // fitting parameter

        nh.param<double>("box/xy_res", box_xy_res, 0.1); // Corridor expansion resolution
        nh.param<double>("box/z_res", box_z_res, 0.1);

        nh.param<double>("mission/start_x", mission_start_x, -7); // mission
        nh.param<double>("mission/start_y", mission_start_y, 1.5);
        nh.param<double>("mission/goal_x", mission_goal_x, 7);
        nh.param<double>("mission/goal_y", mission_goal_y, 1.5);

        package_path = ros::package::getPath("mpc_atypical"); //TODO: fix it properly
        time_step = grid_xy_res / agent_speed;

        color.emplace_back(0);
        color.emplace_back(0);
        color.emplace_back(1);

        return true;
    }
}