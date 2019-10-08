#pragma once

#include <octomap/OcTree.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#define SP_EPSILON          1e-9
#define SP_EPSILON_FLOAT    1e-4
#define SP_INFINITY         1e+9

typedef std::vector<octomap::point3d> initTraj_t;
typedef std::vector<std::pair<std::vector<double>, double>> SFC_t;

//common header please! start
struct BoxConstraint2D{
    double xl;
    double yl;
    double xu;
    double yu;
    // How will these be used? (TODO)
    double t_start;
    double t_end;
};

typedef std::vector<int> BoxAllocSeq;  // {N1,N2,N3,,, Nm}
typedef std::vector<BoxConstraint2D> BoxConstraintSeq; // {B1,B2,...,Bm}

// corridor
struct Corridor2D{
    BoxConstraintSeq box_seq;
    BoxAllocSeq box_alloc_seq;
    double height;
    Corridor2D(){}; // default constructor
    Corridor2D(BoxConstraintSeq box_seq,BoxAllocSeq box_alloc_seq,double height):
            box_seq(box_seq),box_alloc_seq(box_alloc_seq),height(height) {};

    std::vector<geometry_msgs::Point> get_corridor_intersect_points();
    visualization_msgs::MarkerArray get_corridor_markers(std::string world_frame_id);
};
//common header please! end