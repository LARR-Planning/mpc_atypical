#include <corridor_ros_wrapper.hpp>
#include <optim_traj_gen/chomp_ros_wrapper.h>


int main(int argc, char ** argv)
{



    ros::init(argc, argv, "mpc_atypical");
    ros::NodeHandle nh ( "~" );
    libCorridorGen::Wrapper corridor_wrapper(nh);

    //load map
    corridor_wrapper.load_map("/worlds/map_reduced_tmp3.bt");
    ROS_INFO("Map Loaded, Oh Yeah");
    //generate corridor
    corridor_wrapper.update();

    ROS_INFO("Hey main, %d",corridor_wrapper.corridor.box_seq.size());



    using namespace CHOMP;

    CHOMP::Wrapper chomp_wrapper(nh);
    ros::Time t_sim_start = ros::Time::now();

    ROS_INFO("[CHOMP] optim_traj_gen initialized. Referance time is : %d",t_sim_start.toSec());
    chomp_wrapper.t_ref = t_sim_start;

    // load octomap first
    string file_name;
    nh.param<string>("map_file_name",file_name,"/home/lyw/catkin_ws/src/mpc_atypical/worlds/map_reduced_tmp3.bt");

    // 0. octomap read
    octomap::OcTree* tree = new octomap::OcTree(file_name);
    std::cerr<<"octree of size " << tree->size() <<" is opened!"<<std::endl;
    std::cerr<<"Now, create edf from the octree.."<<std::endl;

    chomp_wrapper.load_map(tree);
    chomp_wrapper.map_type = 0; // set octomap representation

    // 1. define corridors
    CHOMP::BoxAllocSeq2 box_alloc_seq2; // how many points inside per corridor
    CHOMP::BoxConstraintSeq2 box_seq2; // sequence of box dimensions
    double corridor_height = 0.5;

    /*
    BoxConstraint2D box1,box2,box3;
    box1.xl = 3.0; box1.yl = 0; box1.xu = 5.0; box1.yu = 2.0; box_seq.push_back(box1);
    box2.xl = 4.0; box2.yl = 1.5; box2.xu = 5.0; box2.yu = 3.5; box_seq.push_back(box2);
    box3.xl = 4.0; box3.yl = 3.0; box3.xu = 5.5; box3.yu = 5.5; box_seq.push_back(box3);
    */

   for (int i =0; i<corridor_wrapper.corridor.box_seq.size();i++)
   {
       CHOMP::BoxConstraint2D box_tmp;
       box_tmp.xl = corridor_wrapper.corridor.box_seq[i].xl;
       box_tmp.yl = corridor_wrapper.corridor.box_seq[i].yl;
       box_tmp.xu = corridor_wrapper.corridor.box_seq[i].xu;
       box_tmp.yu = corridor_wrapper.corridor.box_seq[i].yu;
       box_seq2.emplace_back(box_tmp);
   }

   /*

    for (auto & elements : corridor_wrapper.corridor.box_seq){
        box_seq2.push_back(elements);
    }
    */

    for (int index_box=0 ;index_box < corridor_wrapper.corridor.box_alloc_seq.size();index_box++){
        box_alloc_seq2.emplace_back(corridor_wrapper.corridor.box_alloc_seq[index_box]);
    }



    /*
    int N1,N2,N3; N1 = 10, N2 = 9, N3 = 10;
    box_alloc_seq2.push_back(N1);
    box_alloc_seq2.push_back(N2);
    box_alloc_seq2.push_back(N3);
    */

    // 2.  define chomp problem at this time
    ros::Time t0 = ros::Time::now();  ros::Duration H(100);
    ros::Time tf = t0 + H;

    CHOMP::Corridor2D sample_corridor(box_seq2, box_alloc_seq2, corridor_height); // corridor
    geometry_msgs::Point start;  start.x = -7; start.y = 1.5; // start
    geometry_msgs::Point goal; goal.x = 7; goal.y = 1.5; // goal
    geometry_msgs::Point start_velocity; start_velocity.x = 1.0; start_velocity.y = -0.01;
    // problem construction
    CHOMP::OptimProblem chomp_problem;
    chomp_problem.corridor = sample_corridor;
    chomp_problem.start = start;
    chomp_problem.start_velocity = start_velocity;
    chomp_problem.goal= goal;
    chomp_problem.t0 = t0;
    chomp_problem.tf = tf;

    // trajectory generation
    chomp_wrapper.optim_traj_gen(chomp_problem);


    ros::Rate rate(10);

    while(ros::ok()){
        chomp_wrapper.publish_routine();
        corridor_wrapper.publish(0);
        ros::spinOnce();
        rate.sleep();
    }



    return 0;


    
}
