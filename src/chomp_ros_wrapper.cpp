#include "optim_traj_gen/chomp_ros_wrapper.h"

using namespace CHOMP;

Wrapper::Wrapper(const ros::NodeHandle& nh_global):nh("~"){    
    // parameter parsing 
    nh.param("cost_param/r_safe",r_safe,3.4);
    nh.param("cost_param/ground_reject_height",ground_rejection_height,0.5);

    nh.param("optim_param/descending_rate",optim_param_default.descending_rate,0.3);
    nh.param("optim_param/descending_rate_min",optim_param_default.descending_rate_min,0.1);
    nh.param("optim_param/descending_rate_max",optim_param_default.descending_rate_max,0.1);

    nh.param("optim_param/max_iter",optim_param_default.max_iter,300);
    nh.param("optim_param/weight_prior",optim_param_default.weight_prior,1e-2);
    nh.param("optim_param/term_cond",optim_param_default.termination_cond,1e-3);
    nh.param("optim_param/gamma",optim_param_default.gamma,0.4);
    nh.param("optim_param/n_step",optim_param_default.n_step,10);


    nh.param<string>("world_frame_id",world_frame_id,"/world");    
    nh.param<string>("desired_frame_id",desired_frame_id,"/vehicle_desired");    

    pub_path_cur_solution = nh.advertise<nav_msgs::Path>("predictor/chomp_solution_path",1);
    pub_vis_problem = nh.advertise<visualization_msgs::MarkerArray>("optim_traj_gen/cur_problem",1);
    pub_marker_pnts_path = nh.advertise<visualization_msgs::Marker>("optim_traj_gen/chomp_sol_pnts",1);
    pub_cur_control_pose = nh.advertise<geometry_msgs::PoseStamped>("optim_traj_gen/chomp_cur_eval_pnt",1);
    pub_cur_control_input = nh.advertise<geometry_msgs::TwistStamped>("optim_traj_gen/chomp_cur_input",1);

    nh.param("is_voxblox_pub",is_voxblox_pub,false);


    pnts_on_path_marker.action = 0;
    pnts_on_path_marker.header.frame_id = world_frame_id;
    pnts_on_path_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    double scale = 0.16;
    pnts_on_path_marker.scale.x = scale;
    pnts_on_path_marker.scale.y = scale;
    pnts_on_path_marker.scale.z = scale;
    pnts_on_path_marker.color.b = 0.8;
    pnts_on_path_marker.color.g = 1.;    
    pnts_on_path_marker.color.a = 0.8;
};



// load EDF map from octree. The octree is assumed to be provided from outside 
void Wrapper::load_map(octomap::OcTree* octree_ptr){    
    // EDT map scale = octomap  
    double x,y,z;
    octree_ptr->getMetricMin(x,y,z);
    octomap::point3d boundary_min(x,y,z); 
    boundary_min.z() =ground_rejection_height;
    octree_ptr->getMetricMax(x,y,z);
    octomap::point3d boundary_max(x,y,z); 
    dx = octree_ptr->getResolution();
    double edf_max_dist = r_safe;
    bool unknownAsOccupied = false;

    // EDF completed
    edf_ptr = new DynamicEDTOctomap(edf_max_dist,octree_ptr,
        boundary_min,
        boundary_max,unknownAsOccupied);
    edf_ptr->update();

    
    // flag 
    is_map_load = true;
};

void Wrapper::load_map(string file_name){    
    // we will create voxblox server only if it is voxblox mode. It talks too much.
    ros::NodeHandle nh_global;
    voxblox_server = new voxblox::EsdfServer (nh_global,nh);    
    // EDT map scale = octomap  
    voxblox_server->loadMap(file_name);
    dx = voxblox_server->getEsdfMapPtr()->voxel_size(); // voxel_size 
    cout<<"resolution of ESDF : "<<dx<<"/ block size: "<<voxblox_server->getEsdfMapPtr()->block_size()<<endl;
    is_map_load = true;
};

void Wrapper::set_problem(OptimProblem problem){
    cur_problem = problem;
    is_problem_exist = true;
    ROS_INFO("[CHOMP] problem loaded! ");
}

/**
 * @brief build prior term and linear inquality to make 1/2x'Mx+hx. / Cx <= d 
 * @param M insert matrix to M (1/2 x' M x + hx )  
 * @param h insert matrix to h (currently vector ... to be revised ) 
 * @param C insert matrix to C  (Cx <= d , linear inequality constraints ) 
 * @param d insert matrix to d 
 * @param prior_points points of length No<=N
 * @param gamma weight factor for prior points 
 * @param goal goal point 
 * @param horizon : tf - t0  
 */
void Wrapper::build_matrix(MatrixXd &M,VectorXd &h, MatrixXd & C,MatrixXd & d, Corridor2D corridor,geometry_msgs::Point start,geometry_msgs::Point start_velocity,geometry_msgs::Point goal,double horizon,OptimParam* param){

    int N = accumulate(corridor.box_alloc_seq.begin(),corridor.box_alloc_seq.end(),0)-(corridor.box_seq.size()-1); // number of points to be optimized (dim * N = tot number of variable in optimization)
    int N_box_constraint_blck = accumulate(corridor.box_alloc_seq.begin(),corridor.box_alloc_seq.end(),0); // number of points to be optimized (dim * N = tot number of variable in optimization)
    double dt = horizon/(N-1); 
    double vx0 = start_velocity.x;  
    double vy0 = start_velocity.y;  
    
    // init     
    // if external parameter is given 
    if(param == NULL)
        optim_param = optim_param_default;
    else
        optim_param = *param;


    // weight for start and goal  
    double gamma = optim_param.gamma;


    /**
     *  1. matrix for cost (M,h)
     **/

    // here, A,b is |Ax-b|^2
    int m_row = ((N-1) + (N-2) + 3) * dim;  // velocity + acceleration + goal

    MatrixXd A = MatrixXd(m_row,dim*N);
    VectorXd b = VectorXd(m_row);
    
   // 1. start (point and velocity) and goal fitting (1+1+1 = 3)  
    MatrixXd A0 = MatrixXd::Zero(dim*(3),dim*N);
    VectorXd b0 = VectorXd::Zero(dim*(3));
    // end point 
    A0.block(0,dim*(N-1),dim,dim) =  gamma*(VectorXd::Ones(dim)).asDiagonal();     
    b0.block(0,0,dim,1) << gamma*goal.x, 
                                gamma*goal.y;       

    // start point   
    A0.block(dim,0,dim,dim) = gamma*(VectorXd::Ones(dim)).asDiagonal();     
    b0.block(dim,0,dim,1) << gamma*start.x, 
                                gamma*start.y;       
    // start velocity     
    A0.block(2*dim,0,1,3) << -gamma*1,0,gamma*1;    
    A0.block(2*dim+1,1,1,3) << -gamma*1,0,gamma*1;    

    b0.block(2*dim,0,dim,1) << gamma*vx0*dt,
                                gamma*vy0*dt;

    // 2. minimization of 1st order derivatives  
    MatrixXd A1 = MatrixXd::Zero(dim*(N-1),dim*N);
    VectorXd b1= VectorXd::Zero(dim*(N-1));
    
    for(int n = 0;n<N-1;n++){
        A1.block((dim*n),(dim*n),dim,dim) = MatrixXd::Identity(dim,dim);        
        A1.block((dim*n),(dim*(n+1)),dim,dim) = -MatrixXd::Identity(dim,dim);            
    }

    // 3. 2nd order derivatives  
    MatrixXd A2= MatrixXd::Zero(dim*(N-2),dim*N);
    VectorXd b2= VectorXd::Zero(dim*(N-2));

    for(int n = 0;n<N-2;n++){
        A2.block((dim*n),(dim*n),dim,dim) = MatrixXd::Identity(dim,dim);        
        A2.block((dim*n),(dim*(n+1)),dim,dim) = -2*MatrixXd::Identity(dim,dim);            
        A2.block((dim*n),(dim*(n+2)),dim,dim) = MatrixXd::Identity(dim,dim);            
    }

    // cost matrix construct 
    A << A0,
         A1,
         A2;

    b << b0,
         b1,
         b2;


    /**
    cout<<"cost prior : |Ax-b|^2"<<endl;
    
    cout<<"A:"<<endl;
    cout<<A<<endl;

    cout<<"b:"<<endl;
    cout<<b<<endl;
    **/

    M = MatrixXd(dim*N,dim*N);
    h = VectorXd(dim*N);
     
    M = 2*A.transpose()*A;
    h = -2*A.transpose()*b;


    /**
     *  2. matrix for linear inequality (C,d)
     **/

    MatrixXd C_new(4*N_box_constraint_blck,dim*N);
    MatrixXd d_new(4*N_box_constraint_blck,1);
    C_new.setZero(); d_new.setZero();
  
    C = C_new;
    d = d_new;

    MatrixXd C_sub(4,2); C_sub << -1,0,
                                   1,0,
                                   0,-1,
                                   0,1;                                    
    int blck_until_now = 0;
    int box_seq_idx = 0; 
    for(auto it = corridor.box_alloc_seq.begin(); it != corridor.box_alloc_seq.end() ; it++ , box_seq_idx ++){        
        int N_pnts_on_this_box = *it;
        for(int blck_idx = blck_until_now; blck_idx < blck_until_now + N_pnts_on_this_box ; blck_idx++){            
            MatrixXd d_sub(4,1);
            d_sub << -corridor.box_seq[box_seq_idx].xl,
                      corridor.box_seq[box_seq_idx].xu,
                      -corridor.box_seq[box_seq_idx].yl,
                      corridor.box_seq[box_seq_idx].yu;

            C.block(4*blck_idx,2*(blck_idx-box_seq_idx),4,2) = C_sub;
            d.block(4*blck_idx,0,4,1) = d_sub;                         
        }
        blck_until_now += N_pnts_on_this_box;
    }  

    /**    
    cout<<"linear inequality : Cx<=d "<<endl;
    
    cout<<"C:"<<endl;
    cout<<C<<endl;

    cout<<"d:"<<endl;
    cout<<d<<endl;
    **/
}


// Prepare chomp by setting cost matrix and constraint matrix to solver. Also, it outputs initial guess    
VectorXd Wrapper::prepare_chomp(MatrixXd M,VectorXd h,MatrixXd C,MatrixXd d,Corridor2D corridor,geometry_msgs::Point start,geometry_msgs::Point goal,OptimParam* param){
    
    // if external parameter is given 
    if(param == NULL)
        optim_param = optim_param_default;
    else
        optim_param = *param;
        
    
    if (M.rows() == h.size()){
        
        CostParam cost_param;
        cost_param.dx = 1.2*dx;
        cost_param.ground_height = ground_rejection_height;
        cost_param.r_safe = r_safe;

        // complete problem with obstacle functions (currently only octomap)
        solver.set_problem(M,h,C,d,this->edf_ptr,cost_param);

        return cur_problem.get_initial_guess();
    }
    else{
        cerr<<"[CHOMP] dimension error for optimizatoin returning zero-filled initial guess."<<endl;
        return VectorXd::Zero(M.rows());
    }
}

/**
 * @brief we already prepared solver. we just trigger the solver to solve the problem  
 * 
 * @param x0 (if size of x0 == 0, we reuse the previous solution as initial guess)  
 * @return true 
 * @return false 
 */
void Wrapper::solve_chomp(VectorXd x0){


    // TODO (should I reuse the previous solution ?)
    if (x0.size())
        if (map_type == 0)
            recent_optim_result = solver.solve(x0,optim_param);    
        else 
            recent_optim_result = solver.solve2(x0,optim_param);
    else
        if (map_type == 0)
            recent_optim_result = solver.solve(recent_optim_result.solution_raw,optim_param);    
        else
            recent_optim_result = solver.solve2(recent_optim_result.solution_raw,optim_param);
    
    
    // to make the path as trajectory 
    double t0,tf; 
    t0 = (cur_problem.t0-t_ref).toSec(); tf = (cur_problem.tf-t_ref).toSec(); // 
    recent_optim_result.complete_solution(t0,tf,cur_problem.corridor.height); // 

    // if solved, 
    VectorXd x_sol = recent_optim_result.solution_raw;
    is_solved = true; // whether it is corrent or not 
    nav_msgs::Path path;
    path.header.frame_id = world_frame_id;
    int H = x_sol.size()/2; // dim = 2 assumed
    // extract path from solution x_sol and let's make corresponding points along a path also
    pnts_on_path_marker.points.clear();
    for (int h = 0; h<H;h++){

        double x = x_sol(h*2);
        double y = x_sol(h*2+1);
        double z = ground_rejection_height; // the height  of the path  
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = world_frame_id;
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y; 
        pose_stamped.pose.position.z = z; 
        
        path.poses.push_back(pose_stamped);
        pnts_on_path_marker.points.push_back(pose_stamped.pose.position);
    }    
    // this is where the path is updated inside of class 
    current_path = path;
    std::cout<<"[CHOMP] path uploaded"<<std::endl;
}

void Wrapper::publish_routine(){

    // marker publish 
    if(is_problem_exist)
        pub_vis_problem.publish(cur_problem.get_markers(world_frame_id));
    // if it is solved 
    if(is_solved){
        // solved path publish
        pub_path_cur_solution.publish(current_path);        
        pub_marker_pnts_path.publish(pnts_on_path_marker);
        
        // desired pose publish 
        ros::Time eval_t = ros::Time::now();
        geometry_msgs::Point eval_pnt = recent_optim_result.chomp_traj.evalute_point((eval_t - t_ref).toSec());
        geometry_msgs::Point eval_vel = recent_optim_result.chomp_traj.evalute_vel((eval_t - t_ref).toSec());
        geometry_msgs::Twist eval_input = recent_optim_result.chomp_traj.evalute_input((eval_t - t_ref).toSec());  
        geometry_msgs::TwistStamped eval_input_stamp;
        eval_input_stamp.header.frame_id = desired_frame_id; eval_input_stamp.twist = eval_input; 
        eval_input_stamp.twist = eval_input;
        printf("[CHOMP] (v,w) = (%f,%f) \n",eval_input.linear.x,eval_input.angular.z);
         
        // desired input (v,w) publish
        geometry_msgs::PoseStamped eval_poseStamped; 
        eval_poseStamped.header.frame_id = world_frame_id;
        eval_poseStamped.header.stamp = ros::Time::now();
        eval_poseStamped.pose.position = eval_pnt;
        double target_yaw = atan2(eval_vel.y,eval_vel.x);
        tf::Quaternion q; q.setRPY(0,0,target_yaw);
        eval_poseStamped.pose.orientation.x = q.getX();
        eval_poseStamped.pose.orientation.y = q.getY();
        eval_poseStamped.pose.orientation.z = q.getZ();
        eval_poseStamped.pose.orientation.w = q.getW();
        pub_cur_control_pose.publish(eval_poseStamped);
        pub_cur_control_input.publish(eval_input_stamp);

        // desired transform publish. The tf will be used as referance frame for twist stamped
        tf::Transform transform;
        transform.setRotation(q);
        transform.setOrigin(tf::Vector3(eval_pnt.x,eval_pnt.y,eval_pnt.z));
        tf_br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),world_frame_id,desired_frame_id));

    }
    // esdf publish 
    if(this->map_type == 1 and is_voxblox_pub){
        this->voxblox_server->setSliceLevel(ground_rejection_height);
        this->voxblox_server->publishSlices();
        this->voxblox_server->publishPointclouds();
        this->voxblox_server->publishTsdfSurfacePoints();
    }
}

// return path. the returned path will be endowed with times. This function returns the solution path in the form of matrix  
MatrixXd Wrapper::get_current_prediction_path(){

    MatrixXd prediction_path; // N x 3 matrix 
    if(current_path.poses.size()){

        int N = current_path.poses.size(); // number of path  
        
        prediction_path  = MatrixXd(N,3);
        for (int n = 0; n<current_path.poses.size(); n++){
            prediction_path(n,0)  = current_path.poses[n].pose.position.x; 
            prediction_path(n,1)  = current_path.poses[n].pose.position.y; 
            prediction_path(n,2)  = current_path.poses[n].pose.position.z;         
        }

    }else    
        cerr<<"No prediction path loaded"<<endl;


        
    return prediction_path;     
}

// return default optim_param 
OptimParam Wrapper::get_default_optim_param(){
    return optim_param_default;
}


/**
 * @brief generate optimal trajectory given corridor structure and (start,goal) 
 * 
 * @param corridor 
 * @param start  
 * @param goal 
 * @return true : if succeed in path planning  
 * @return false 
 */


void Wrapper::optim_traj_gen(OptimProblem problem){

    // update and save problem 
    set_problem(problem);
    
    MatrixXd M,C,d; VectorXd h;
    Corridor2D corridor = problem.corridor;
    geometry_msgs::Point start = problem.start;
    geometry_msgs::Point start_velocity = problem.start_velocity;
    geometry_msgs::Point goal= problem.goal;

    double horizon = (problem.tf - problem.t0).toSec(); // temporal information is required for velocity consideration
    build_matrix(M,h,C,d,corridor,start,start_velocity,goal,horizon);
    VectorXd x0 = prepare_chomp(M,h,C,d,corridor,start,goal);
    solve_chomp(x0);        
}


