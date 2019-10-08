#pragma once

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Submodules
#include <corridor.hpp>
#include <init_traj_planner.hpp>
#include <param.hpp>

namespace libCorridorGen {
    class ResultPublisher {
    public:
        ResultPublisher(const ros::NodeHandle& _nh,
                        std::shared_ptr<Corridor> _corridor_obj,
                        std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
                        Param _param)
                : nh(_nh),
                  corridor_obj(std::move(_corridor_obj)),
                  initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
                  param(std::move(_param)) {

            pub_initTraj = nh.advertise<visualization_msgs::MarkerArray>("/initTraj", 1);
            pub_obsBox = nh.advertise<visualization_msgs::MarkerArray>("/SFC", 1);
        }


        void update(double current_time) {
            update_initTraj();
            if (corridor_obj.use_count() != 0)
                update_obsBox(current_time);
        }

        void publish() {
            pub_initTraj.publish(msgs_initTraj);
            if (corridor_obj.use_count() != 0)
                pub_obsBox.publish(msgs_obsBox);
        }

    private:
        ros::NodeHandle nh;
        std::shared_ptr<Corridor> corridor_obj;
        std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
        Param param;

        // ROS publisher
        ros::Publisher pub_initTraj;
        ros::Publisher pub_obsBox;

        // ROS messages
        visualization_msgs::MarkerArray msgs_initTraj;
        visualization_msgs::MarkerArray msgs_obsBox;


        void update_initTraj() {
            visualization_msgs::MarkerArray mk_array;
            for (int i = 0; i < initTrajPlanner_obj->initTraj.size(); i++) {
                visualization_msgs::Marker mk;
                mk.header.frame_id = "world";
                mk.header.stamp = ros::Time::now();
                mk.ns = "initTraj";
                mk.type = visualization_msgs::Marker::CUBE;
                mk.action = visualization_msgs::Marker::ADD;

                mk.pose.orientation.x = 0.0;
                mk.pose.orientation.y = 0.0;
                mk.pose.orientation.z = 0.0;
                mk.pose.orientation.w = 1.0;

                mk.color.a = 1.0;
                mk.color.r = param.color[0];
                mk.color.g = param.color[1];
                mk.color.b = param.color[2];

                mk.id = i;
                octomap::point3d p_init = initTrajPlanner_obj->initTraj[i];
                mk.pose.position.x = p_init.x();
                mk.pose.position.y = p_init.y();
                mk.pose.position.z = p_init.z();

                mk.scale.x = 0.1;
                mk.scale.y = 0.1;
                mk.scale.z = 0.1;

                mk_array.markers.emplace_back(mk);
            }

            msgs_initTraj = mk_array;
        }

        void update_obsBox(double current_time) {
            visualization_msgs::MarkerArray mk_array;
            visualization_msgs::Marker mk;
            mk.header.frame_id = "world";
            mk.ns = "SFC";
            mk.type = visualization_msgs::Marker::CUBE;
            mk.action = visualization_msgs::Marker::ADD;

            mk.pose.orientation.x = 0.0;
            mk.pose.orientation.y = 0.0;
            mk.pose.orientation.z = 0.0;
            mk.pose.orientation.w = 1.0;

            for (int bi = 0; bi < corridor_obj->SFC.size(); bi++) {
                mk.id = bi;
                std::vector<double> obstacle_box = corridor_obj->SFC[bi].first;

                mk.pose.position.x = (obstacle_box[0] + obstacle_box[3]) / 2.0;
                mk.pose.position.y = (obstacle_box[1] + obstacle_box[4]) / 2.0;
                mk.pose.position.z = (obstacle_box[2] + obstacle_box[5]) / 2.0;

                mk.scale.x = obstacle_box[3] - obstacle_box[0];
                mk.scale.y = obstacle_box[4] - obstacle_box[1];
                mk.scale.z = obstacle_box[5] - obstacle_box[2];

                mk.color.a = 0.2;
                mk.color.r = param.color[0];
                mk.color.g = param.color[1];
                mk.color.b = param.color[2];

                mk_array.markers.emplace_back(mk);
            }

            msgs_obsBox = mk_array;
        }
    };
}