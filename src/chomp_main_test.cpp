#include <optim_traj_gen/chomp_ros_wrapper.h>
/**
 * @brief test the wrapper routine with sample map  
 */
using namespace CHOMP;
int main(int argc, char** argv){

    ros::init(argc,argv,"chomp_test"); 
    ros::NodeHandle nh("~");
    CHOMP::Wrapper chomp_wrapper(nh); 
    ros::Time t_sim_start = ros::Time::now();

    ROS_INFO("[CHOMP] optim_traj_gen initialized. Referance time is : %d",t_sim_start.toSec());
    chomp_wrapper.t_ref = t_sim_start;     
    
    // load octomap first 
    string file_name;
    nh.param<string>("map_file_name",file_name,"/home/lyw/catkin_ws/src/mpc_atypical/worlds/map3.bt");

    // 0. octomap read
    octomap::OcTree* tree = new octomap::OcTree(file_name);    
    std::cerr<<"octree of size " << tree->size() <<" is opened!"<<std::endl;
    std::cerr<<"Now, create edf from the octree.."<<std::endl;
    
    chomp_wrapper.load_map(tree);
    chomp_wrapper.map_type = 0; // set octomap representation 
     
    // 1. define corridors 
    BoxAllocSeq box_alloc_seq; // how many points inside per corridor 
    BoxConstraintSeq box_seq; // sequence of box dimensions  
    double corridor_height = 0.5; 

    BoxConstraint2D box1,box2,box3;   
    box1.xl = 3.0; box1.yl = 0; box1.xu = 5.0; box1.yu = 2.0; box_seq.push_back(box1);
    box2.xl = 4.0; box2.yl = 1.5; box2.xu = 5.0; box2.yu = 3.5; box_seq.push_back(box2);
    box3.xl = 4.0; box3.yl = 3.0; box3.xu = 5.5; box3.yu = 5.5; box_seq.push_back(box3);

    int N1,N2,N3; N1 = 10, N2 = 9, N3 = 10;    
    box_alloc_seq.push_back(N1);
    box_alloc_seq.push_back(N2);
    box_alloc_seq.push_back(N3);


    // 2.  define chomp problem at this time  
    ros::Time t0 = ros::Time::now();  ros::Duration H(10);
    ros::Time tf = t0 + H; 

    Corridor2D sample_corridor(box_seq,box_alloc_seq,corridor_height); // corridor 
    geometry_msgs::Point start;  start.x = 3.5; start.y = 0.5; // start 
    geometry_msgs::Point goal; goal.x = 5; goal.y = 5; // goal 
    geometry_msgs::Point start_velocity; start_velocity.x = 0.3; start_velocity.y = 0.2; 
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


    ros::Rate rate(30);

    while(ros::ok()){
        chomp_wrapper.publish_routine();
        ros::spinOnce();
        rate.sleep();
    }


    return 0; 
}
