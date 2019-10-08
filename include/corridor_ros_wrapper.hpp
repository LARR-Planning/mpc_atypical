#pragma once

// Configuration
#include <param.hpp>
#include <timer.hpp>

// Octomap
#include <octomap/OcTree.h>

// Submodule
#include <ecbs_planner.hpp>
#include <corridor.hpp>
#include <result_publisher.hpp>

namespace libCorridorGen{
    class Wrapper{
    public:
        Corridor2D corridor;

        Wrapper(const ros::NodeHandle& _nh){
            nh = _nh;
            param.init(nh);
        };

        // map
        void load_map(const std::string& file_name){
            octree_obj.reset(new octomap::OcTree(param.package_path + file_name));
        }

        // Generate corridors
        bool update(){
            // Step 1: Plan Initial Trajectory
            initTraj_obj.reset(new libCorridorGen::ECBSPlanner(octree_obj, param));
            if (!initTraj_obj.get()->update(param.log)) {
                return -1;
            }

            // Step 2: Generate Safe Corridor
            corridor_obj.reset(new libCorridorGen::Corridor(initTraj_obj, octree_obj, param));
            if (corridor_obj.get()->update(param.log)) {
                corridor = corridor_obj.get()->corridor;
            }

            // Ready for Publish Corridor Result
            resultPub_obj.reset(new libCorridorGen::ResultPublisher(nh, corridor_obj, initTraj_obj, param));
        }

        //ros
        void publish(double current_time){
            resultPub_obj.get()->update(current_time);
            resultPub_obj.get()->publish();
        }

    private:
        ros::NodeHandle nh;
        libCorridorGen::Param param;

        // octree
        std::shared_ptr<octomap::OcTree> octree_obj;

        //submodules
        std::shared_ptr<libCorridorGen::ECBSPlanner> initTraj_obj;
        std::shared_ptr<libCorridorGen::Corridor> corridor_obj;
        std::shared_ptr<libCorridorGen::ResultPublisher> resultPub_obj;
    };
}