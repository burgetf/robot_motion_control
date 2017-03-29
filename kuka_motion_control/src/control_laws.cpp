

#include <kuka_motion_control/control_laws.h>

namespace kuka_motion_controller
{


//Constructor
RobotController::RobotController(string robot_desciption_param, string kinematic_chain, string ns_prefix)
{

    //Get package path of "kuka_motion_control"
    package_path_ = ros::package::getPath("kuka_motion_control");


    //Set obstacle marker topic
    obstacle_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_obstacles", 10);

    //Set base platform as collision object marker topic
    base_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_base_platform", 10);


    // -------------------------- START: SETUP PLANNING SCENE AND ROBOT MODEL -------------

    //Create planning scene monitor
    //psm_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor(robot_desciption_param));
    psm_ =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_desciption_param);

    //Get topic prefix in constructor argument list -> to set topic names e.g. "robotino_group/planning_scene"
    string planning_scene_ns = ns_prefix + "planning_scene";
    string planning_scene_service_ns = ns_prefix + "get_planning_scene";
    string endeffector_trajectory_ns = ns_prefix + "endeffector_trajectory";

    //string joint_states_ns = ns_prefix + "joint_states";
    //string attached_collision_object_ns = ns_prefix + "attached_collision_object";
    //string collision_object_ns = ns_prefix + "collision_object";
    //string planning_scene_world_ns = ns_prefix + "planning_scene_world";


    //Name of Planning Scene Service;
    m_planning_scene_service = planning_scene_service_ns;

    //Planning Scene Publisher
    scene_pub_ = nh.advertise<moveit_msgs::PlanningScene>(planning_scene_ns, 10);
    ros::Duration(0.5).sleep();

    //Set Kinematic group
    kinematic_goup_ = kinematic_chain;

    //Create Robot model
    RobotModel = boost::shared_ptr<KDLRobotModel>(new KDLRobotModel(robot_desciption_param, planning_scene_ns, endeffector_trajectory_ns, kinematic_goup_));

    //Get the kinematic chain
    //if (kinematic_goup_ == "pr2_base" || kinematic_goup_ == "pr2_base_arm" || kinematic_goup_ == "pr2_arm" || kinematic_goup_ == "kuka_complete_arm" || kinematic_goup_ == "omnirob_lbr_sdh" || kinematic_goup_ == "omnirob_base" || kinematic_goup_ == "robotino_robot")
    //{
        kin_chain_ = RobotModel->getCompleteArmChain();
        //ROS_INFO("Kinematic Chain perceived ...");
    //}
    //else
    //    ROS_ERROR("The selected chain is not available for control here!");


    //Get Number of joints for the chain
    num_joints_ = kin_chain_.getNrOfJoints();

    //Provides the min, max and center value for all joint contained in a given kinematic chain
    q_min_.resize(num_joints_);
    q_max_.resize(num_joints_);
    opt_pos_.resize(num_joints_);
    RobotModel->getJointRangeData(kin_chain_ , q_min_, q_max_, opt_pos_);


    //Initialize current configuration
    current_config_ = KDL::JntArray(num_joints_);
    joint_weights_ = KDL::JntArray(num_joints_);

    //Set Random start configuration
    setRandomStartConf();

    // -------------------------- END: SETUP PLANNING SCENE AND ROBOT MODEL ---------------



    // -------------------------- START: CONTROL PARAMETER SETUP --------------------------
    //Init PD Motion Strategy
    pd_motion_strategy_ = 0;

    //Velocity output for joints
    q_dot_out_ = KDL::JntArray(num_joints_);

    //Initialize control parameters

    //Joint Velocity Integration
    delta_t_ = 0.007;
    //Experience
    //  -> 0.0025 from Paper "A control-based approach to task-constrained motion planning"
    //  -> previously used 0.008

    //Secondary task gain
    gain_sec_task_ = -2.2;//-2.2;
    //best value is -1.1 according to Jonas
    // - between 0 and -0.38 very good convergence but higher failrate due to JL violation
    // - between -0.38 and -1.1 higher failrate due to convergence, but lower failrate due to JL violation
    // - from -1.1 and lower, higher failrate due to convergence, lower failrate due to JL violation, but also more IK iterations

    min_change_rate_ = 0.1;//0.008;
    index_last_joint_activated_ = 0;

    //Flag indicating whether joint values are within their admissible range
    jl_ok_ = true;

    //Initialize error norm
    error_norm_ = 1000.0;

    //Gain for the cartesian endeffector error (used in controller)
    error_gain_ = 1.0;

    //Init Variables Constraint Vector (all variables constraint, i.e. var_constraint_vec_ = 1 for all task dimensions)
    for (int i = 0 ; i < 6 ; i++)
        var_constraint_vec_.push_back(1);

    //Activation parameter (for shaping JL Gradient function used in Null Space Projection Methods, must be in the range 0 < activation_param < 0.5)
    jl_activation_param_ = 0.4; //0.2

    //Constant Damping factor for damped-least squares method
    const_lambda_fac_ = 0.05; //0.03;

    //Variables for variable damping with DLS (based on manipulability analysis)
    min_manip_treshold_ = 0.003; //0.0004
    max_damping_factor_ = 0.07; //0.07;

    //Treshold for error norm
    error_treshold_ = 0.01;//0.001;

    // -------------------------- END: CONTROL PARAMETER SETUP --------------------------


    // -------------------------- START: COLLISION AVOIDANCE SETUP --------------------------

    //Select control points at the following joints
    joint_indices_for_CA_.push_back(8); //at endeffector
    joint_indices_for_CA_.push_back(5); //at upper arm
    joint_indices_for_CA_.push_back(4); //at elbow
    joint_indices_for_CA_.push_back(3); //at lower arm
    joint_indices_for_CA_.push_back(2); //at base

    //Radius of spheres used as safety margin around control points and to compute potential field
    ca_sphere_radius_ = 0.175;
    //Shape factor to influence behavior of repulsive vector magnitude
    ca_alpha_shape_fac_ = 6.0;
    //Maximum magnitude of repulsive vector
    max_mag_rep_vec_ = 3.0; // 3 [m/s]

    //Set CA inactive
    collision_avoidance_active_ = false;

    //Parameter used for Joint velocity bounds shaping for robot body collision avoidance
    max_joint_velocity_ = 0.6;

    // -------------------------- END: COLLISION AVOIDANCE SETUP --------------------------


    // -------------------------- START: FLAGS AND CONSTANTS ----------------------------
    //Set color for endeffector trace
    color_rgb_.push_back(0.54); //red
    color_rgb_.push_back(0.168); //green
    color_rgb_.push_back(0.883); //blue

    //Set sleep duration between configuration
    sleep_duration_between_confs_ = 0.0008;//0.0008;

    //Maximum number of points for the endeffector trajectory trace
    max_ee_trace_points_ = 5000; //Note: RViz crashed when using more than 8000 Line strip points!!!!!

    //Flag indicating whether ee trajectory tracking is enabled
    traj_tracking_enabled_ = false;

    //Flag indicating when joint weight gradient changes direction (in that case the joint weight update rate is lowered)
    joint_weight_gradient_direction_ = 0;

    //Maximum value for joint weight change
    for (int i = 0 ; i < num_joints_ ; i++)
        max_joint_weight_modification_.push_back(0.5);

    //Store mean trajectory error using the joint weights of the previous iteration
    previous_mean_joint_trajectory_error_ = 1000.0;

    // -------------------------- END: FLAGS AND CONSTANTS ----------------------------

    //ROS_INFO("Robot Controller Constructor done!!!");

}


RobotController::RobotController(boost::shared_ptr<kuka_motion_controller::KDLRobotModel> kdl_robot_model, string robot_desciption_param, string kinematic_chain, string ns_prefix)
{
    //Get package path of "kuka_motion_control"
    package_path_ = ros::package::getPath("kuka_motion_control");


    //Set obstacle marker topic
    obstacle_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_obstacles", 10);

    //Set base platform as collision object marker topic
    base_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_base_platform", 10);


    // -------------------------- START: SETUP PLANNING SCENE AND ROBOT MODEL -------------

    //Create planning scene monitor
    //psm_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor(robot_desciption_param));
    psm_ =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_desciption_param);

    //Get topic prefix in constructor argument list -> to set topic names e.g. "robotino_group/planning_scene"
    string planning_scene_ns = ns_prefix + "planning_scene";
    string planning_scene_service_ns = ns_prefix + "get_planning_scene";

    //string joint_states_ns = ns_prefix + "joint_states";
    //string attached_collision_object_ns = ns_prefix + "attached_collision_object";
    //string collision_object_ns = ns_prefix + "collision_object";
    //string planning_scene_world_ns = ns_prefix + "planning_scene_world";
    //string endeffector_trajectory_ns = ns_prefix + "endeffector_trajectory";

//    psm_->startSceneMonitor(planning_scene_ns);
//    psm_->startStateMonitor(joint_states_ns, attached_collision_object_ns);
//    psm_->startWorldGeometryMonitor(collision_object_ns,planning_scene_world_ns, false);
    //m_planning_scene_monitor->DEFAULT_PLANNING_SCENE_SERVICE = planning_scene_service_ns;

    //Name of Planning Scene Service;
    m_planning_scene_service = planning_scene_service_ns;

    //Planning Scene Publisher
    scene_pub_ = nh.advertise<moveit_msgs::PlanningScene>(planning_scene_ns, 10);
    ros::Duration(0.5).sleep();

    //Set Kinematic group
    kinematic_goup_ = kinematic_chain;

    //Create Robot model
    RobotModel = kdl_robot_model;

    //Get the kinematic chain
    //if (kinematic_goup_ == "pr2_base" || kinematic_goup_ == "pr2_base_arm" || kinematic_goup_ == "pr2_arm" || kinematic_goup_ == "kuka_complete_arm" || kinematic_goup_ == "omnirob_lbr_sdh" || kinematic_goup_ == "omnirob_base" || kinematic_goup_ == "robotino_robot")
    //{
        kin_chain_ = RobotModel->getCompleteArmChain();
        //ROS_INFO("Kinematic Chain perceived ...");
    //}
    //else
    //    ROS_ERROR("The selected chain is not available for control here!");


    //Get Number of joints for the chain
    num_joints_ = kin_chain_.getNrOfJoints();

    //Provides the min, max and center value for all joint contained in a given kinematic chain
    q_min_.resize(num_joints_);
    q_max_.resize(num_joints_);
    opt_pos_.resize(num_joints_);
    RobotModel->getJointRangeData(kin_chain_ , q_min_, q_max_, opt_pos_);


    //Initialize current configuration
    current_config_ = KDL::JntArray(num_joints_);
    joint_weights_ = KDL::JntArray(num_joints_);

    //Set Random start configuration
    setRandomStartConf();

    // -------------------------- END: SETUP PLANNING SCENE AND ROBOT MODEL ---------------



    // -------------------------- START: CONTROL PARAMETER SETUP --------------------------
    //Init PD Motion Strategy
    pd_motion_strategy_ = 0;

    //Velocity output for joints
    q_dot_out_ = KDL::JntArray(num_joints_);

    //Initialize control parameters

    //Joint Velocity Integration
    delta_t_ = 0.007;
    //Experience
    //  -> 0.0025 from Paper "A control-based approach to task-constrained motion planning"
    //  -> previously used 0.008

    //Secondary task gain
    gain_sec_task_ = -2.2;//-2.2;
    //best value is -1.1 according to Jonas
    // - between 0 and -0.38 very good convergence but higher failrate due to JL violation
    // - between -0.38 and -1.1 higher failrate due to convergence, but lower failrate due to JL violation
    // - from -1.1 and lower, higher failrate due to convergence, lower failrate due to JL violation, but also more IK iterations

    min_change_rate_ = 0.1;//0.008;
    index_last_joint_activated_ = 0;

    //Flag indicating whether joint values are within their admissible range
    jl_ok_ = true;

    //Initialize error norm
    error_norm_ = 1000.0;

    //Gain for the cartesian endeffector error (used in controller)
    error_gain_ = 1.0;

    //Init Variables Constraint Vector (all variables constraint, i.e. var_constraint_vec_ = 1 for all task dimensions)
    for (int i = 0 ; i < 6 ; i++)
        var_constraint_vec_.push_back(1);

    //Activation parameter (for shaping JL Gradient function used in Null Space Projection Methods, must be in the range 0 < activation_param < 0.5)
    jl_activation_param_ = 0.4; //0.2

    //Constant Damping factor for damped-least squares method
    const_lambda_fac_ = 0.05; //0.03;

    //Variables for variable damping with DLS (based on manipulability analysis)
    min_manip_treshold_ = 0.003; //0.0004
    max_damping_factor_ = 0.07; //0.07;

    //Treshold for error norm
    error_treshold_ = 0.01;//0.001;

    // -------------------------- END: CONTROL PARAMETER SETUP --------------------------


    // -------------------------- START: COLLISION AVOIDANCE SETUP --------------------------

    //Select control points at the following joints
    joint_indices_for_CA_.push_back(8); //at endeffector
    joint_indices_for_CA_.push_back(5); //at upper arm
    joint_indices_for_CA_.push_back(4); //at elbow
    joint_indices_for_CA_.push_back(3); //at lower arm
    joint_indices_for_CA_.push_back(2); //at base

    //Radius of spheres used as safety margin around control points and to compute potential field
    ca_sphere_radius_ = 0.175;
    //Shape factor to influence behavior of repulsive vector magnitude
    ca_alpha_shape_fac_ = 6.0;
    //Maximum magnitude of repulsive vector
    max_mag_rep_vec_ = 3.0; // 3 [m/s]

    //Set CA inactive
    collision_avoidance_active_ = false;

    //Parameter used for Joint velocity bounds shaping for robot body collision avoidance
    max_joint_velocity_ = 0.6;

    // -------------------------- END: COLLISION AVOIDANCE SETUP --------------------------


    // -------------------------- START: FLAGS AND CONSTANTS ----------------------------
    //Set color for endeffector trace
    color_rgb_.push_back(0.54); //red
    color_rgb_.push_back(0.168); //green
    color_rgb_.push_back(0.883); //blue

    //Set sleep duration between configuration
    sleep_duration_between_confs_ = 0.0008;//0.0008;

    //Maximum number of points for the endeffector trajectory trace
    max_ee_trace_points_ = 5000; //Note: RViz crashed when using more than 8000 Line strip points!!!!!

    //Flag indicating whether ee trajectory tracking is enabled
    traj_tracking_enabled_ = false;

    //Flag indicating when joint weight gradient changes direction (in that case the joint weight update rate is lowered)
    joint_weight_gradient_direction_ = 0;

    //Maximum value for joint weight change
    for (int i = 0 ; i < num_joints_ ; i++)
        max_joint_weight_modification_.push_back(0.5);

    //Store mean trajectory error using the joint weights of the previous iteration
    previous_mean_joint_trajectory_error_ = 1000.0;

    // -------------------------- END: FLAGS AND CONSTANTS ----------------------------

    //ROS_INFO("Robot Controller Constructor done!!!");

}


//Destructor
RobotController::~RobotController()
{

//    //Read files
     //delete[] file_path_start_config_;
     //delete[] file_path_ee_pos_trajectory_;
     //delete[] file_path_ee_vel_trajectory_;

//    //Write files
//    delete[] file_path_joint_trajectory_;
//    delete[] file_path_joint_weights_;
//    delete[] file_path_endeffector_trajectory_;

//    //Delete array storing endeffector poses
//    for (int i = 0; i < num_ee_poses_; i++)
//        delete [] ee_ref_pos_trajectory_[i];
//    delete [] ee_ref_pos_trajectory_;

//    //Delete array storing endeffector velocities
//    for (int i = 0; i < num_ee_vels_; i++)
//        delete [] ee_ref_vel_trajectory_[i];
//    delete [] ee_ref_vel_trajectory_;




}

//Set Planning Scene Service Name
void RobotController::setPlanningSceneServiceName(string service_name)
{
    m_planning_scene_service = service_name;
}

//Add base platform as obstacle to be considered for collision avoidance
void RobotController::add_Base_Platform_Obstacle()
{
    //Add base platform as collision obstacle

    //Base Platform dimensions in meter
    double table_width = 2.0;
    double table_length = 2.0;
    double table_thickness = 0.025;

    //Base Platform marker
    visualization_msgs::Marker base_platform;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    base_platform.header.frame_id = "/lbr_0_link";
    base_platform.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    base_platform.ns = "base_platform_shapes";
    base_platform.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    base_platform.type = visualization_msgs::Marker::CUBE;

    // Set the Base Platform action.  Options are ADD and DELETE
    base_platform.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    base_platform.pose.position.x = 0.0;
    base_platform.pose.position.y = 0.0;
    base_platform.pose.position.z =  -table_thickness/2;
    base_platform.pose.orientation.x = 0.0;
    base_platform.pose.orientation.y = 0.0;
    base_platform.pose.orientation.z = 0.0;
    base_platform.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    base_platform.scale.x = table_width;
    base_platform.scale.y = table_length;
    base_platform.scale.z = table_thickness;

    // Set the color -- be sure to set alpha to something non-zero!
    base_platform.color.r = 0.0f;
    base_platform.color.g = 1.0f;
    base_platform.color.b = 0.0f;
    base_platform.color.a = 1.0;

    //table.lifetime = ros::Duration();

    //Store obstacle data
    vector<double> base_platform_data;
    base_platform_data.push_back(0.0); //Xpos
    base_platform_data.push_back(0.0); //Ypos
    base_platform_data.push_back(-table_thickness/2); //Zpos
    base_platform_data.push_back(table_thickness/2); //thickness of base platform
    base_platform_data.push_back(1.0); //indicates that only z coordinate should be used to compute distance of control points to this object
    //Store obstacle in obstacle array
    scene_obstacles_.push_back(base_platform_data);

    // Publish the marker
    base_pub_.publish(base_platform);

}


//Activate collision avoidance
void RobotController::add_Collision_Obstacle(vector<double> object_data)
{
    //Set collision avoidance active
    collision_avoidance_active_ = true;


    // ------------------------------- START: ADD OBSTACLES ----------------------

    obstacle_array_.header.frame_id = "/lbr_0_link";
    obstacle_array_.header.stamp = ros::Time::now();
    obstacle_array_.ns = "obstacle_shapes";
    obstacle_array_.action = visualization_msgs::Marker::ADD;
    obstacle_array_.pose.orientation.w = 1.0;
    obstacle_array_.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    obstacle_array_.type = visualization_msgs::Marker::SPHERE_LIST;


    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    geometry_msgs::Point p;
    p.x = object_data[0];
    p.y = object_data[1];
    p.z = object_data[2];

    //Add object to obstacle array
    obstacle_array_.points.push_back(p);


    // Set the scale of the marker (here diameter of the obstacle spheres)
    obstacle_array_.scale.x = 2 * object_data[3]; //2 * radius
    obstacle_array_.scale.y = 2 * object_data[3];
    obstacle_array_.scale.z = 2 * object_data[3];
    //obstacle_array.scale.z = table_thickness;

    // Set the color -- be sure to set alpha to something non-zero!
    obstacle_array_.color.r = 1.0f;
    obstacle_array_.color.g = 0.0f;
    obstacle_array_.color.b = 0.0f;
    obstacle_array_.color.a = 1.0;

    //obstacle_array.lifetime = ros::Duration();

    // Publish the marker
    obstacle_pub_.publish(obstacle_array_);

    //Store obstacle data
    object_data.push_back(0.0); //indicates that all coordinate should be used to compute distance of control point to this object
    scene_obstacles_.push_back(object_data);

    //------------------------------- END: ADD OBSTACLES -----------------------


}



//Get random EE pose
vector<double> RobotController::getRandEEPose()
{
    //Random configuration
    KDL::JntArray rand_conf(num_joints_);

    //Current joint number
    int joint_num = 0;

    //Generate rand config until associated endeffector position is located above the base platform
    while(true)
    {
        //Reset joint number
        joint_num = 0;

        for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
        {
           if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
           {
               rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
               //current_config_(j) = random_number_generator_.gaussian(opt_pos_[j],0.05);
               //root_config(j) = predefined_config(j);

               //cout<<"Random init value for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<rand_conf(joint_num)<<endl;

               joint_num++;
           }
        }

        //Compute ee pose
        vector<double> rand_ee_pose = RobotModel->compute_FK(kin_chain_,rand_conf);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        if(0.0 < rand_ee_pose[2])
            return rand_ee_pose;
    }

}


//Sets the start configuration and the cartesian endeffector goal based on the first and last entry of the joint trajectory file
void RobotController::init(char* start_conf_filename, char *ee_traj_filename, char *ee_vel_traj_filename)
{

    //Store path to files containing the input trajectory data
    file_path_start_config_ = start_conf_filename; //Start configuration
    file_path_ee_pos_trajectory_ = ee_traj_filename; //Endeffector Pose Trajectory
    file_path_ee_vel_trajectory_ = ee_vel_traj_filename; //Endeffector Velocity Trajectory


    //----------------------------- SET START CONFIG ------------------------
    //Map containing the start config map
    map<string, double> start_config;

    //Read first config of the motion segments joint trajectory (of the subject)
    start_config = readStartConfig(file_path_start_config_);

    //Set the start config
    //setStartConf(start_config);



    //--------------------------- READ EE POS TRAJECTORY AND SET FIRST WAYPOINT POS ------------------------
    //Read the endeffector position trajectory from file
    readEEposTrajectory(file_path_ee_pos_trajectory_);

    //Set Flag indicating whether ee trajectory tracking is enabled true
    traj_tracking_enabled_ = true;

    //cout<<"Initial ee goal pose"<<endl;
    vector <double> first_goal_pose;
    for (int i = 0 ; i < 7 ; i++)
    {
        //X   Y   Z   quatX   quatY   quatZ   W
        first_goal_pose.push_back(ee_ref_pos_trajectory_[0][i]);
        //cout<<ee_ref_pos_trajectory_[0][i]<<" ";
    }
    //cout<<endl;

    //Set cartesian endeffector pose for controller
    set_EE_goal_pose(first_goal_pose,file_path_ee_pos_trajectory_);

    //--------------------------- READ EE VEL TRAJECTORY AND SET FIRST WAYPOINT VEL ------------------------
    //Read the endeffector velocity trajectory from file
    readEEvelTrajectory(file_path_ee_vel_trajectory_);


    //Solve forward kinematics for chain and final configuration
    //vector<double> final_ee_pose = RobotModel->compute_FK(kin_chain_,final_config);

//    cout<<"final ee pose"<<endl;
//    cout<<"X: "<<final_ee_pose[0]<<endl;
//    cout<<"Y: "<<final_ee_pose[1]<<endl;
//    cout<<"Z: "<<final_ee_pose[2]<<endl;
//    cout<<"quat_x: "<<final_ee_pose[3]<<endl;
//    cout<<"quat_y: "<<final_ee_pose[4]<<endl;
//    cout<<"quat_z: "<<final_ee_pose[5]<<endl;
//    cout<<"quat_w: "<<final_ee_pose[5]<<endl;

//    cout<<"final ee pose"<<endl;
//    cout<<"X: "<<final_ee_pose[0]<<endl;
//    cout<<"Y: "<<final_ee_pose[1]<<endl;
//    cout<<"Z: "<<final_ee_pose[2]<<endl;
//    cout<<"rot_x: "<<final_ee_pose[3]<<endl;
//    cout<<"rot_y: "<<final_ee_pose[4]<<endl;
//    cout<<"rot_z: "<<final_ee_pose[5]<<endl;


}


//Read first and last config of the motion segments joint trajectory (of the subject)
map<string, double> RobotController::readStartConfig(char* joint_traj_filename)
{

    map<string, double> start_conf_map;
    //map<string, double> goal_conf_map;
    //vector<map<string, double> > SG_config;

    // --------------- Get the number of rows of the file --------------------------
    string line;
    int num_configs = 0;
    int num_vals_per_line = 0;
    bool num_vals_read = false;

    char * pEnd;
    char char_line[1000];
    vector<string> joint_names_from_file;

    ifstream jVal_file(joint_traj_filename);
    if (jVal_file.is_open())
    {
        while ( jVal_file.good() )
        {
             std::getline(jVal_file,line);

             //Get the number of values per line once
             if (num_vals_read == false)
             {
                 //Set flag to true
                 num_vals_read = true;

                 for (int r = 0; r<2; r++)
                 {
                     istringstream buf(line);
                     istream_iterator<string> beg(buf), end;
                     vector<string> substrings(beg, end); // done!

                     //Get number of values per line
                     num_vals_per_line = num_vals_per_line + substrings.size();

                     //Transform string into char array
                     strcpy(char_line,line.c_str());
                     char * j_names = std::strtok (char_line," ");

                     while (j_names != 0)
                     {
                       joint_names_from_file.push_back(j_names);
                       //cout<< j_names<<endl;
                       j_names = std::strtok(NULL," ");
                     }

                     //Next Line of joint names
                     std::getline(jVal_file,line);
                 }
             }

             //Increment number of rows found in the file
             num_configs++;
        }
        //Delete last line containing eof
        num_configs--;
        //std::cout <<"File cotains " <<num_configs << " configurations" << std::endl;
        //std::cout <<"Line has " <<num_vals_per_line << " values" << std::endl;
        jVal_file.close();
    }

    else std::cout << "Unable to open file (readStartConfig)";


    //--------------- Fill the 2D Array --------------------------
    //Row number counter
    int curr_row_num = 1;
    jVal_file.open(joint_traj_filename);
    if (jVal_file.is_open())
    {
        while (jVal_file.good() )
        {
         //read next line from file (either a configuration or eof)
         std::getline (jVal_file,line);

         //Detect when end of file is reached
         if (jVal_file.eof())
             break;

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp = char_line;

         //Get start and end frame config of motion segment
         for (int i = 0 ; i < num_vals_per_line ; i++)
         {
           if (curr_row_num == 4) //row 1 and 2 = j_names ,  row 3 = joint_weights
           {
               start_conf_map[joint_names_from_file[i]] = strtod(tmp,&pEnd);
               //cout<<"Init value for joint: "<<joint_names_from_file[i]<<" is: "<<start_conf_map[joint_names_from_file[i]]<<endl;
           }
//           else if (curr_row_num == num_configs) //last config of file
//           {
//               goal_conf_map[joint_names_from_file[i]] = strtod(tmp,&pEnd);
//               //cout<<"Goal value for joint: "<<joint_names_from_file[i]<<" is: "<<goal_conf_map[joint_names_from_file[i]]<<endl;
//           }
           else
               break;

           //Assign the remaining char elements (pEnd) to the current char pointer
           tmp = pEnd;

         }


         //Leave while loop when first config has been stored
         if (curr_row_num == 4)
            break;

         //Increment row counter
         curr_row_num++;

        }
        //Close file
        jVal_file.close();
    }

    else std::cout << "Unable to open file (readStartConfig)";


    //Return vector containing a map for the start and goal configuration
    //SG_config.push_back(start_conf_map);
    //SG_config.push_back(goal_conf_map);
    //return SG_config;

    //Return vector containing a map for the start configuration
    return start_conf_map;

}


//Read endeffector trajectory of the motion segments (of the subject)
void RobotController::readEEposTrajectory(char* ee_traj_filename)
{

    // --------------- Get the number of rows of the file --------------------------
    string line;
    int num_vals_per_line = 0;
    bool num_vals_read = false;
    num_ee_poses_ = 0;

    char * pEnd;
    char char_line[1000];

    ifstream eeVal_file(ee_traj_filename);
    if (eeVal_file.is_open())
    {
        while ( eeVal_file.good() )
        {
             // Get first line with Parameter names
             std::getline(eeVal_file,line);

             //Get the number of values per line once
             if (num_vals_read == false)
             {
                 //Get second line with first endeffector poses
                 std::getline(eeVal_file,line);

                 istringstream buf(line);
                 istream_iterator<string> beg(buf), end;
                 vector<string> substrings(beg, end); // done!
                 //Set flag to true
                 num_vals_read = true;
                 //Get number of values per line
                 num_vals_per_line = substrings.size();
             }

             //Increment number of rows found in the file
             num_ee_poses_++;
        }
        //Delete first line containing ee pose parameter names
        //num_ee_poses_--;
        //Delete last line containing eof
        num_ee_poses_--;
        //std::cout <<"File cotains " <<num_ee_poses_ << " endeffector poses" << std::endl;
        //std::cout <<"Line has " <<num_vals_per_line << " values" << std::endl;
        eeVal_file.close();
    }

    else std::cout << "Unable to open file (readEEposTrajectory)";


    //cout<<"Total number of ee poses: "<<num_ee_poses_<<endl;
    //cout<<num_vals_per_line<<endl;

    // --------------- Initialize Array storing the values from the file --------------------------
    ee_ref_pos_trajectory_=new double*[num_ee_poses_]; //creates a new array of pointers to double objects
    for(int i=0; i<num_ee_poses_; ++i)
    {
        ee_ref_pos_trajectory_[i]=new double[7];
    }
        //ee_ref_pos_trajectory_[i]=new double[num_vals_per_line-1];


    //--------------- Fill the 2D Array --------------------------
    //Row number counter
    int curr_row_num = 1;
    //ee pose row counter
    int j = 0;
    eeVal_file.open(ee_traj_filename);

    if (eeVal_file.is_open())
    {
        while (eeVal_file.good() )
        {
         //read next line from file (either a configuration or eof)
         std::getline (eeVal_file,line);

         //Detect when end of file is reached
         if (eeVal_file.eof())
             break;

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp = char_line;

         //Get current endeffector pose of frame
         //vector<double> curr_ee_pose;
         for (int i = 0 ; i < num_vals_per_line ; i++)
         {
           if (curr_row_num > 1) //row 1 = pose parameter names (i.e. x,y,z,qx,qy,qz,qw)
           {
               //Get current line element value
               double curr_val = strtod(tmp,&pEnd);

               //Store endeffector pose in trajectory array
               ee_ref_pos_trajectory_[j][i] = curr_val;

               //curr_ee_pose.push_back(strtod(tmp,&pEnd));
               //cout<<ee_ref_pos_trajectory_[j][i]<<endl;
           }
           else
               break;

           //Assign the remaining char elements (pEnd) to the current char pointer
           tmp = pEnd;

         }
         //cout<<endl;

         //Next goal pose (don't increment for the first line containing the ee pose parameter names)
         if (curr_row_num > 1)
            j++;

         //ee_ref_pos_trajectory_.push_back(curr_ee_pose);


         //Increment row counter
         curr_row_num++;

        }

        //Close file
        eeVal_file.close();
    }
    else std::cout << "Unable to open file (readEEposTrajectory)";

}


//Read endeffector velocity trajectory of the motion segments (of the subject)
void RobotController::readEEvelTrajectory(char* ee_vel_traj_filename)
{
    // --------------- Get the number of rows of the file --------------------------
    string line;
    int num_vals_per_line = 0;
    bool num_vals_read = false;
    num_ee_vels_ = 0;

    char * pEnd;
    char char_line[1000];

    ifstream ee_vel_file(ee_vel_traj_filename);
    if (ee_vel_file.is_open())
    {
        while ( ee_vel_file.good() )
        {
             std::getline(ee_vel_file,line);

             //Get the number of values per line once
             if (num_vals_read == false)
             {
                 //Get second line with first endeffector pose velocities
                 std::getline(ee_vel_file,line);

                 istringstream buf(line);
                 istream_iterator<string> beg(buf), end;
                 vector<string> substrings(beg, end); // done!
                 //Set flag to true
                 num_vals_read = true;
                 //Get number of values per line
                 num_vals_per_line = substrings.size();
             }

             //Increment number of rows found in the file
             num_ee_vels_++;
        }
        //Delete first line containing ee pose vel parameter names
        //num_ee_vels_--;
        //Delete last line containing eof
        num_ee_vels_--;
        //std::cout <<"File cotains " <<num_ee_vels_ << " endeffector velocities" << std::endl;
        //std::cout <<"Line has " <<num_vals_per_line << " values" << std::endl;
        ee_vel_file.close();
    }

    else std::cout << "Unable to open file (readEEvelTrajectory)";


//    //cout<<"Total number of ee velocitities: "<<num_ee_vels_<<endl;
//    //cout<<num_vals_per_line<<endl;

    // --------------- Initialize Array storing the values from the file --------------------------
    ee_ref_vel_trajectory_=new double*[num_ee_vels_]; //creates a new array of pointers to double objects
    for(int i=0; i<num_ee_vels_; ++i)
          ee_ref_vel_trajectory_[i]=new double[6];
        //ee_ref_vel_trajectory_[i]=new double[num_vals_per_line];


    //--------------- Fill the 2D Array --------------------------
    //Row number counter
    int curr_row_num = 1;
    //ee pose row counter
    int j = 0;
    ee_vel_file.open(ee_vel_traj_filename);

    if (ee_vel_file.is_open())
    {

        while (ee_vel_file.good() )
        {
         //read next line from file (either a configuration or eof)
         std::getline (ee_vel_file,line);

         //Detect when end of file is reached
         if (ee_vel_file.eof())
             break;

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp = char_line;

         //Get current endeffector pose vel of frame
         //cout<<j<<" ";
         for (int i = 0 ; i < num_vals_per_line ; i++)
         {
           if (curr_row_num > 1) //row 1 = pose parameter names (i.e. Xdot, Ydot, Zdot, rotXdot, rotYdot, rotZdot)
           {
               //Get current line element value
               double curr_val = strtod(tmp,&pEnd);

               //Store endeffector pose vel in trajectory array
               ee_ref_vel_trajectory_[j][i] = curr_val;

           }
           else
               break;

           //Assign the remaining char elements (pEnd) to the current char pointer
           tmp = pEnd;

         }
         //cout<<endl;

         //Next goal pose (don't increment for the first line containing the ee pose parameter names)
         if (curr_row_num > 1)
            j++;

         //Increment row counter
         curr_row_num++;

       }

        //Close file
       ee_vel_file.close();
    }

    else std::cout << "Unable to open file (readEEvelTrajectory)";
}


//Get Random Configuration (by uniform sampling in the joint range)
// -> sample base pose in square environment borders, here min_x = max_x and min_y = max_y
KDL::JntArray RobotController::getRandomConf(double env_size_x, double env_size_y)
{
    //Current joint number
    int joint_num = 0;

    //Random config to be returned
    KDL::JntArray rand_conf;
    rand_conf = KDL::JntArray(num_joints_);

    //Generate rand config until associated endeffector position is located above the base platform
    while(true)
    {
        //Reset joint number
        joint_num = 0;

        for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
        {
           if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
           {
               //For revolute joints
               if(kin_chain_.getSegment(j).getJoint().getTypeName() == "RotAxis")
               {
                    rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
                    //rand_conf(joint_num) = random_number_generator_.gaussian(opt_pos_[j],0.05);
               }
               else //For prismatic joints (translation of base in x,y plane)
               {
                   //Prismatic joint belongs to manipulator chain
                   if(1<joint_num)
                   {
                       rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
                   }
                   else //Prismatic joint belongs to base chain
                   {
                       //If environment is unconfined , i.e. has no borders -> sample from range of prismatic joint
                       if(env_size_x == 0.0 && env_size_y == 0.0)
                            rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
                       else
                       {
                            if(joint_num == 0)
                                rand_conf(joint_num) = random_number_generator_.uniformReal(-env_size_x/2.0, env_size_x/2.0);
                            else if(joint_num == 1)
                                rand_conf(joint_num) = random_number_generator_.uniformReal(-env_size_y/2.0, env_size_y/2.0);
                            //else
                            //    ROS_ERROR("getRandomConf does not work for kinematic structures composed of more than two (for the mobile base) prismatic joints!");
                       }
                   }
               }

               //Store joint_names
               joint_names_.push_back(kin_chain_.getSegment(j).getJoint().getName());

               //cout<<"Random value for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<rand_conf(joint_num)<<endl;

               joint_num++;
           }
        }

        //Compute ee pose
        vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,rand_conf);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        if(0.0 <= current_ee_pose[2])
            break;
    }

    //Return random config
    return rand_conf;
}


//Get Random Configuration (by uniform sampling in the joint range)
// -> sample base pose in non square environment borders, here min_x != max_x and min_y != max_y
KDL::JntArray RobotController::getRandomConf(vector<double> env_size_x, vector<double> env_size_y)
{
    //Current joint number
    int joint_num = 0;

    //Random config to be returned
    KDL::JntArray rand_conf;
    rand_conf = KDL::JntArray(num_joints_);

    //Generate rand config until associated endeffector position is located above the base platform
    while(true)
    {
        //Reset joint number
        joint_num = 0;

        for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
        {
           if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
           {
               //For revolute joints
               if(kin_chain_.getSegment(j).getJoint().getTypeName() == "RotAxis")
               {
                    rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
                    //rand_conf(joint_num) = random_number_generator_.gaussian(opt_pos_[j],0.05);
               }
               else //For prismatic joints (translation of base in x,y plane)
               {
                   //Prismatic joint belongs to manipulator chain
                   if(1<joint_num)
                   {
                       rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
                   }
                   else //Prismatic joint belongs to base chain
                   {
                       //If environment is unconfined , i.e. has no borders -> sample from range of prismatic joint
                       if(env_size_x[0] == 0.0 && env_size_x[1] == 0.0 && env_size_y[0] == 0.0 && env_size_y[1] == 0.0)
                            rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
                       else
                       {
                            if(joint_num == 0)
                                rand_conf(joint_num) = random_number_generator_.uniformReal(env_size_x[0], env_size_x[1]);
                            else if(joint_num == 1)
                                rand_conf(joint_num) = random_number_generator_.uniformReal(env_size_y[0], env_size_y[1]);
                            //else
                            //    ROS_ERROR("getRandomConf does not work for kinematic structures composed of more than two (for the mobile base) prismatic joints!");
                       }
                   }
               }

               //Store joint_names
               joint_names_.push_back(kin_chain_.getSegment(j).getJoint().getName());

               //cout<<"Random value for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<rand_conf(joint_num)<<endl;

               joint_num++;
           }
        }

        //Compute ee pose
        vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,rand_conf);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        if(0.0 <= current_ee_pose[2])
            break;
    }

    //Return random config
    return rand_conf;
}


//Get Random Configuration (by gaussian sampling around a mean configuration vector with standard deviation "std_dev")
// -> sample base pose in square environment borders, here min_x = max_x and min_y = max_y
KDL::JntArray RobotController::getRandomConf(vector<double> mean_config, double std_dev, double env_size_x, double env_size_y)
{
    //Current joint number
    int joint_num = 0;

    //Random config to be returned
    KDL::JntArray rand_conf;
    rand_conf = KDL::JntArray(num_joints_);

    //Generate rand config until associated endeffector position is located above the base platform
    while(true)
    {
        //Reset joint number
        joint_num = 0;

        //Gaussian sampling for revolute and uniform sampling for prismatic joints
        for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
        {
           if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
           {
               //For rotational joints: Gaussian Sampling around mean_config
               if(kin_chain_.getSegment(j).getJoint().getTypeName() == "RotAxis")
               {
                   rand_conf(joint_num) = random_number_generator_.gaussian(mean_config[joint_num], std_dev);

                   //For base rotational joint
                   if(joint_num == 2)
                       rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);

                   //Check for joint limits violation
                   if(rand_conf(joint_num) < q_min_[joint_num])
                       rand_conf(joint_num) = q_min_[joint_num] + 0.01;
                   if(q_max_[joint_num] < rand_conf(joint_num))
                       rand_conf(joint_num) = q_max_[joint_num] - 0.01;

               }
               else  //For prismatic joints: Uniform Sampling from joint range
               {
                   //Prismatic joint belongs to manipulator chain
                   if(1<joint_num)
                   {
                       rand_conf(joint_num) = random_number_generator_.gaussian(mean_config[joint_num], std_dev);

                       //Check for joint limits violation
                       if(rand_conf(joint_num) < q_min_[joint_num])
                           rand_conf(joint_num) = q_min_[joint_num] + 0.01;
                       if(q_max_[joint_num] < rand_conf(joint_num))
                           rand_conf(joint_num) = q_max_[joint_num] - 0.01;
                   }
                   else //Prismatic joint belongs to base chain
                   {
                       //If environment is unconfined , i.e. has no borders -> sample from range of prismatic joint
                       if(env_size_x == 0.0 && env_size_y == 0.0)
                            rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
                       else
                       {
                           //For base translational joint in x-direction
                           if(joint_num == 0)
                               rand_conf(joint_num) = random_number_generator_.uniformReal(-env_size_x/2, env_size_x/2);
                           else if(joint_num == 1) //For base translational joint in y-direction
                               rand_conf(joint_num) = random_number_generator_.uniformReal(-env_size_y/2, env_size_y/2);
                           //else
                               //ROS_ERROR("getRandomConf does not work for kinematic structures composed of more than two (for the mobile base) prismatic joints!");
                       }
                   }

               }


               //cout<<"Mean config value: "<<mean_config[j]<<endl;
               //cout<<"Sampled config value: "<<rand_conf(joint_num)<<endl;

               //Store joint_names
               joint_names_.push_back(kin_chain_.getSegment(j).getJoint().getName());

               //cout<<"Random value for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<rand_conf(joint_num)<<endl;

               joint_num++;
           }
        }
        //cout<<endl;

        //Compute ee pose
        vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,rand_conf);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        if(0.0 <= current_ee_pose[2])
            break;
    }

    //Return random config
    return rand_conf;
}


//Get Random Configuration (by gaussian sampling around a mean configuration vector with standard deviation "std_dev")
// -> sample base pose in non square environment borders, here min_x != max_x and min_y != max_y
KDL::JntArray RobotController::getRandomConf(vector<double> mean_config, double std_dev, vector<double> env_size_x, vector<double> env_size_y)
{
    //Current joint number
    int joint_num = 0;

    //Random config to be returned
    KDL::JntArray rand_conf;
    rand_conf = KDL::JntArray(num_joints_);

    //Generate rand config until associated endeffector position is located above the base platform
    while(true)
    {
        //Reset joint number
        joint_num = 0;

        //Gaussian sampling for revolute and uniform sampling for prismatic joints
        for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
        {
           if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
           {
               //For rotational joints: Gaussian Sampling around mean_config
               if(kin_chain_.getSegment(j).getJoint().getTypeName() == "RotAxis")
               {
                   rand_conf(joint_num) = random_number_generator_.gaussian(mean_config[joint_num], std_dev);

                   //For base rotational joint
                   if(joint_num == 2)
                       rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);

                   //Check for joint limits violation
                   if(rand_conf(joint_num) < q_min_[joint_num])
                       rand_conf(joint_num) = q_min_[joint_num] + 0.01;
                   if(q_max_[joint_num] < rand_conf(joint_num))
                       rand_conf(joint_num) = q_max_[joint_num] - 0.01;

               }
               else  //For prismatic joints: Uniform Sampling from joint range
               {
                   //Prismatic joint belongs to manipulator chain
                   if(1<joint_num)
                   {
                       rand_conf(joint_num) = random_number_generator_.gaussian(mean_config[joint_num], std_dev);

                       //Check for joint limits violation
                       if(rand_conf(joint_num) < q_min_[joint_num])
                           rand_conf(joint_num) = q_min_[joint_num] + 0.01;
                       if(q_max_[joint_num] < rand_conf(joint_num))
                           rand_conf(joint_num) = q_max_[joint_num] - 0.01;
                   }
                   else //Prismatic joint belongs to base chain
                   {
                       //If environment is unconfined , i.e. has no borders -> sample from range of prismatic joint
                       if(env_size_x[0] == 0.0 && env_size_x[1] == 0.0 && env_size_y[0] == 0.0 && env_size_y[1] == 0.0)
                            rand_conf(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
                       else
                       {
                           //For base translational joint in x-direction
                           if(joint_num == 0)
                               rand_conf(joint_num) = random_number_generator_.uniformReal(env_size_x[0], env_size_x[1]);
                           else if(joint_num == 1) //For base translational joint in y-direction
                               rand_conf(joint_num) = random_number_generator_.uniformReal(env_size_y[0], env_size_y[1]);
                           //else
                               //ROS_ERROR("getRandomConf does not work for kinematic structures composed of more than two (for the mobile base) prismatic joints!");
                       }
                   }

               }


               //cout<<"Mean config value: "<<mean_config[j]<<endl;
               //cout<<"Sampled config value: "<<rand_conf(joint_num)<<endl;

               //Store joint_names
               joint_names_.push_back(kin_chain_.getSegment(j).getJoint().getName());

               //cout<<"Random value for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<rand_conf(joint_num)<<endl;

               joint_num++;
           }
        }
        //cout<<endl;

        //Compute ee pose
        vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,rand_conf);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        if(0.0 <= current_ee_pose[2])
            break;
    }

    //Return random config
    return rand_conf;
}


//Set Random Start Configuration
void RobotController::setRandomStartConf()
{
    //Current joint number
    int joint_num = 0;

    //Generate rand config until associated endeffector position is located above the base platform
    while(true)
    {
        //Reset joint number
        joint_num = 0;

        for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
        {
           if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
           {
               current_config_(joint_num) = random_number_generator_.uniformReal(q_min_[joint_num], q_max_[joint_num]);
               //current_config_(j) = random_number_generator_.gaussian(opt_pos_[j],0.05);
               //root_config(j) = predefined_config(j);

               //Store joint_names
               joint_names_.push_back(kin_chain_.getSegment(j).getJoint().getName());

               //cout<<"Random init value for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<current_config_(joint_num)<<endl;

               //Init joint weights
               joint_weights_(joint_num) = 1.0;

               //Set map entry
               conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(joint_num);


               joint_num++;
           }
        }


        //Compute ee pose
        vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        if(0.0 <= current_ee_pose[2])
            break;
    }
}


//Set a specific Start Configuration as std::map
void RobotController::setStartConf(map<string, double> start_config)
{
    //Set start and goal configuration
    int joint_num = 0;
    for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
    {
       if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
       {
           //Start config
           current_config_(joint_num) = start_config[kin_chain_.getSegment(j).getJoint().getName()];

           //Print initial value for current joint
           //cout<<"Init value from file for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<current_config_(joint_num)<<endl;

           //Goal config
           //final_config(joint_num) = goal_config[kin_chain_.getSegment(j).getJoint().getName()];

           //Print initial value for current joint
           //cout<<"Goal value from file for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<final_config(joint_num)<<endl;

           //Set map entry
           //conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(joint_num);

           joint_num++;
       }
    }
}



//Set a specific Start Configuration as double vector
void RobotController::setStartConf(vector<double> start_config)
{
    if (start_config.size() < kin_chain_.getNrOfJoints() && start_config.size() > kin_chain_.getNrOfJoints())
    {
        ROS_ERROR("Size of start config vector does not match number of DOF of the robot");
    }

    //Set start and goal configuration
    int joint_num = 0;
    for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
    {
       if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
       {
           //Start config
           current_config_(joint_num) = start_config[joint_num];

           //Print initial value for current joint
           //cout<<"Init value from file for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<current_config_(joint_num)<<endl;

           //Goal config
           //final_config(joint_num) = goal_config[kin_chain_.getSegment(j).getJoint().getName()];

           //Print initial value for current joint
           //cout<<"Goal value from file for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is :"<<final_config(joint_num)<<endl;

           //Set map entry
           conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(joint_num);

           joint_num++;
       }
    }
}


//Set endeffector goal pose and file paths to write result
void RobotController::set_EE_goal_pose(vector<double> ee_pose, char *traj_file_name)
{
    //Set number of desired ee poses to one when trajectory tracking is off
    if (traj_tracking_enabled_ == false)
        num_ee_poses_ = 1;

//    //Enter first map containing root config into the "configurations" vector
//    configurations_.push_back(conf_);

    //-------------------------------- SET ENDEFFECTOR POSE ---------------------------------
    //Set the endeffector cartesian goal pose
    current_goal_ee_pose_ = ee_pose;

    //Orientation Error computation based on literature:
    // - Robotics Modeling Planning and Control.pdf (page 140), Equation (3.91)

    //Skew-symmetric matrix from endeffector goal pose
    vector< vector<double> > current_goal_ee_pose_skew_mat;
    vector<double> skew_mat_first_row, skew_mat_second_row, skew_mat_third_row;
    skew_mat_first_row.push_back(0.0);
    skew_mat_first_row.push_back(-current_goal_ee_pose_[5]);
    skew_mat_first_row.push_back(current_goal_ee_pose_[4]);

    skew_mat_second_row.push_back(current_goal_ee_pose_[5]);
    skew_mat_second_row.push_back(0.0);
    skew_mat_second_row.push_back(-current_goal_ee_pose_[3]);

    skew_mat_third_row.push_back(-current_goal_ee_pose_[4]);
    skew_mat_third_row.push_back(current_goal_ee_pose_[3]);
    skew_mat_third_row.push_back(0.0);

    current_goal_ee_pose_skew_mat.push_back(skew_mat_first_row);
    current_goal_ee_pose_skew_mat.push_back(skew_mat_second_row);
    current_goal_ee_pose_skew_mat.push_back(skew_mat_third_row);


    //Dimension of task space
    dim_task_space_ = ee_pose.size() -1;


    //Initialize error
    error_.resize(dim_task_space_);

    //Solve forward kinematics for chain and current configuration
    vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

    //Store initial endeffector pose
    //ee_poses_.push_back(current_ee_pose);

    //Compute cartesian error and error_norm
    double error_sum = 0;
    for(int i = 0; i < error_.size() ; i++)
    {
        //        //Don't care variable
        //        if (current_goal_ee_pose_[i] == 1000.0)
        //             error_[i] = 0.0;
        //        //Position error
        //        if ( i < 3 && current_goal_ee_pose_[i] != 1000.0)
        //            error_[i] = current_goal_ee_pose_[i] - current_ee_pose[i];
        //        //Orientation error
        //        if ( i >= 3 && current_goal_ee_pose_[i] != 1000.0)
        //        {
        //            error_[i] = current_ee_pose[6] * current_goal_ee_pose_[i] - current_goal_ee_pose_[6] * current_ee_pose[i] - (current_goal_ee_pose_skew_mat[i-3][0] * current_ee_pose[3] + current_goal_ee_pose_skew_mat[i-3][1] * current_ee_pose[4] + current_goal_ee_pose_skew_mat[i-3][2] * current_ee_pose[5]);
        //        }

        //Don't care variable
        if (var_constraint_vec_[i] == 0)
             error_[i] = 0.0;
        //Position error
        if ( i < 3 && var_constraint_vec_[i] == 1)
            error_[i] = current_goal_ee_pose_[i] - current_ee_pose[i];
        //Orientation error
        if ( i >= 3 && var_constraint_vec_[i] == 1)
        {
            error_[i] = current_ee_pose[6] * current_goal_ee_pose_[i] - current_goal_ee_pose_[6] * current_ee_pose[i] - (current_goal_ee_pose_skew_mat[i-3][0] * current_ee_pose[3] + current_goal_ee_pose_skew_mat[i-3][1] * current_ee_pose[4] + current_goal_ee_pose_skew_mat[i-3][2] * current_ee_pose[5]);
        }

        error_sum += (error_[i]*error_[i]);

    }
    error_norm_ =sqrt(error_sum);



    //-------------------------------- SET ENDEFFECTOR POSE VEL ---------------------------------
    //Init endeffector velocity to zero
    //Linear velocity (Xdot, Ydot, Zdot)
    current_goal_ee_vel_.push_back(0.0);
    current_goal_ee_vel_.push_back(0.0);
    current_goal_ee_vel_.push_back(0.0);
    //Angular velocity (rotXdot   rotYXdot   rotZXdot)
    current_goal_ee_vel_.push_back(0.0);
    current_goal_ee_vel_.push_back(0.0);
    current_goal_ee_vel_.push_back(0.0);




    //------------------------ START: FILES TO WRITE -------------------------
    //Set path to the file that will store the generated endeffector trajectory
    string folder_path = package_path_ + "/trajectories/ee_pos_trajectories_output/";
    const char* path_to_EET_file_1 = folder_path.c_str();
    //Get filename from input file
    string file_path(traj_file_name);
    unsigned path_slash_position = file_path.find_last_of("/");
    string filename = file_path.substr(path_slash_position+1);
    string entire_path = path_to_EET_file_1 + filename;
    file_path_endeffector_trajectory_ = new char[entire_path.size() + 1];
    copy(entire_path.begin(), entire_path.end(), file_path_endeffector_trajectory_);
    file_path_endeffector_trajectory_[entire_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_endeffector_trajectory_<<endl;

    //Set path to the file that will store the generated joint trajectory
    folder_path = package_path_ + "/trajectories/joint_pos_trajectories_output/";
    const char* path_to_JT_file_1 = folder_path.c_str();
    unsigned path_underscore_position = filename.find("_ee"); //_first_of
    string traj_name = filename.substr(0, path_underscore_position+1);
    entire_path = path_to_JT_file_1 + traj_name + "joint_trajectory.txt";
    file_path_joint_trajectory_ = new char[entire_path.size() + 1];
    copy(entire_path.begin(), entire_path.end(), file_path_joint_trajectory_);
    file_path_joint_trajectory_[entire_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_joint_trajectory_<<endl;

    //Path for the file storing the final/learned joint weights
    folder_path = package_path_ + "/trajectories/learned_joint_weights/";
    const char* path_to_JW_file_1 = folder_path.c_str();
    path_underscore_position = filename.find("_ee"); //_first_of
    traj_name = filename.substr(0, path_underscore_position+1);
    entire_path = path_to_JW_file_1 + traj_name + "_joint_weights.txt";
    file_path_joint_weights_ = new char[entire_path.size() + 1];
    copy(entire_path.begin(), entire_path.end(), file_path_joint_weights_);
    file_path_joint_weights_[entire_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_joint_weights_<<endl;
    //------------------------ END: FILES TO WRITE -------------------------


}



//Set endeffector goal pose without storing trajectory data
void RobotController::set_EE_goal_pose(vector<double> ee_pose)
{
    //Set number of desired ee poses to one when trajectory tracking is off
    if (traj_tracking_enabled_ == false)
        num_ee_poses_ = 1;

//    //Enter first map containing root config into the "configurations" vector
//    configurations_.push_back(conf_);

    //-------------------------------- SET ENDEFFECTOR POSE ---------------------------------
    //Set the endeffector cartesian goal pose
    current_goal_ee_pose_ = ee_pose;

    //Orientation Error computation based on literature:
    // - Robotics Modeling Planning and Control.pdf (page 140), Equation (3.91)

    //Skew-symmetric matrix from endeffector goal pose
    vector< vector<double> > current_goal_ee_pose_skew_mat;
    vector<double> skew_mat_first_row, skew_mat_second_row, skew_mat_third_row;
    skew_mat_first_row.push_back(0.0);
    skew_mat_first_row.push_back(-current_goal_ee_pose_[5]);
    skew_mat_first_row.push_back(current_goal_ee_pose_[4]);

    skew_mat_second_row.push_back(current_goal_ee_pose_[5]);
    skew_mat_second_row.push_back(0.0);
    skew_mat_second_row.push_back(-current_goal_ee_pose_[3]);

    skew_mat_third_row.push_back(-current_goal_ee_pose_[4]);
    skew_mat_third_row.push_back(current_goal_ee_pose_[3]);
    skew_mat_third_row.push_back(0.0);

    current_goal_ee_pose_skew_mat.push_back(skew_mat_first_row);
    current_goal_ee_pose_skew_mat.push_back(skew_mat_second_row);
    current_goal_ee_pose_skew_mat.push_back(skew_mat_third_row);


    //Dimension of task space
    dim_task_space_ = ee_pose.size() -1;


    //Initialize error
    error_.resize(dim_task_space_);

    //Solve forward kinematics for chain and current configuration
    vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

    //Store initial endeffector pose
    //ee_poses_.push_back(current_ee_pose);

    //Compute cartesian error and error_norm
    double error_sum = 0;
    for(int i = 0; i < error_.size() ; i++)
    {
        //        //Don't care variable
        //        if (current_goal_ee_pose_[i] == 1000.0)
        //             error_[i] = 0.0;
        //        //Position error
        //        if ( i < 3 && current_goal_ee_pose_[i] != 1000.0)
        //            error_[i] = current_goal_ee_pose_[i] - current_ee_pose[i];
        //        //Orientation error
        //        if ( i >= 3 && current_goal_ee_pose_[i] != 1000.0)
        //        {
        //            error_[i] = current_ee_pose[6] * current_goal_ee_pose_[i] - current_goal_ee_pose_[6] * current_ee_pose[i] - (current_goal_ee_pose_skew_mat[i-3][0] * current_ee_pose[3] + current_goal_ee_pose_skew_mat[i-3][1] * current_ee_pose[4] + current_goal_ee_pose_skew_mat[i-3][2] * current_ee_pose[5]);
        //        }

        //Don't care variable
        if (var_constraint_vec_[i] == 0)
             error_[i] = 0.0;
        //Position error
        if ( i < 3 && var_constraint_vec_[i] == 1)
            error_[i] = current_goal_ee_pose_[i] - current_ee_pose[i];
        //Orientation error
        if ( i >= 3 && var_constraint_vec_[i] == 1)
        {
            error_[i] = current_ee_pose[6] * current_goal_ee_pose_[i] - current_goal_ee_pose_[6] * current_ee_pose[i] - (current_goal_ee_pose_skew_mat[i-3][0] * current_ee_pose[3] + current_goal_ee_pose_skew_mat[i-3][1] * current_ee_pose[4] + current_goal_ee_pose_skew_mat[i-3][2] * current_ee_pose[5]);
        }

        error_sum += (error_[i]*error_[i]);

    }
    error_norm_ =sqrt(error_sum);



    //-------------------------------- SET ENDEFFECTOR POSE VEL ---------------------------------
    //Init endeffector velocity to zero
    //Linear velocity (Xdot, Ydot, Zdot)
    current_goal_ee_vel_.push_back(0.0);
    current_goal_ee_vel_.push_back(0.0);
    current_goal_ee_vel_.push_back(0.0);
    //Angular velocity (rotXdot   rotYXdot   rotZXdot)
    current_goal_ee_vel_.push_back(0.0);
    current_goal_ee_vel_.push_back(0.0);
    current_goal_ee_vel_.push_back(0.0);

}


//Set Variable Constraint Vector (constraint variables and don't care variables)
// -> Considered in error computation of control loop
void RobotController::setVariableConstraints(vector<int> var_constraint_vec, vector<pair<double, double> > var_coordinate_dev)
{
    //Check dimension of constraint vector
    if(var_constraint_vec.size() > 6)
        ROS_ERROR("Dimension of constraint vector larger than task space dimension!!!");
    if(var_constraint_vec.size() < 6)
        ROS_ERROR("Dimension of constraint vector smaller than task space dimension!!!");

    //Set Constraint Vector
    var_constraint_vec_ = var_constraint_vec;

    //Set permitted displacement of contraint variables
    // -> specifying which axes of the task frame permit valid displacement
    // -> specifies an lower and upper deviation boundary
    coordinate_dev_ = var_coordinate_dev;

//    for (int i = 0 ; i < 6 ; i++)
//        cout<<var_constraint_vec_[i]<<endl;

//    cout<<endl;

}


void RobotController::set_motion_strategy(int strategy_index)
{
    //Set strategy
    pd_motion_strategy_ = strategy_index;

    //Current element index
    int curr_idx = 0;


    //Check first whether optimal joint weights are available, otherwise set the joint weights manually to default (see below)
    if (pd_motion_strategy_ == 5)
    {
        //Try to load the learned joint weights
        bool joint_weights_available = load_joint_weights(file_path_joint_weights_);

        //Set default values for joint weights if no optimal weights are available
        if (joint_weights_available == false)
        {
            cout << "NO LEARNED JOINT WEIGHTS AVAILABLE !!! -> using all Joints weighted equally strategy instead"<<endl;
            pd_motion_strategy_ = 0;
        }
    }

    if (pd_motion_strategy_ != 5)
    {
        //Set joint weights
        for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
        {
            if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
            {
                //Joints weights are all equal (i.e. PD_MOTION_STRATEGY = 0)
                if (pd_motion_strategy_ == 0)
                {
                  joint_weights_(curr_idx) = 1.0;
                }

                //Joints are weighted differently (weights decreases polynominal along the chain from root to tips)
                if (pd_motion_strategy_ == 1)
                {
                   if (curr_idx == 0)
                     joint_weights_(curr_idx) = 1.0;

                    if (curr_idx > 0)
                     joint_weights_(curr_idx) = joint_weights_(curr_idx-1) * 0.5;
                }


                //Prefer arm over base motion
                if (pd_motion_strategy_ == 2)
                {
                   //Joint belongs to the base
                   if (curr_idx <= 1)
                     joint_weights_(curr_idx) = 0.37;
                   else
                     joint_weights_(curr_idx) = 1.0;
                }

                //Prefer base over arm motion
                if (pd_motion_strategy_ == 3)
                {
                   //Joint belongs to the base
                   if (curr_idx <= 1)
                     joint_weights_(curr_idx) = 1.0;
                   else
                     joint_weights_(curr_idx) = 0.5;
                }


                //Stop last joint
                if (pd_motion_strategy_ == 4)
                {
                   //Joint belongs to the base
                   if (curr_idx == 6)
                     joint_weights_(curr_idx) = 0.0;
                   else
                     joint_weights_(curr_idx) = 1.0;
                }


                //cout<<"Weight for joint "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<joint_weights_(curr_idx)<<endl;
                w_joints_[kin_chain_.getSegment(j).getJoint().getName()] = joint_weights_(curr_idx);


                //Increment element counter
                curr_idx++;
            }

        }

    }


}


//Load the learned joint weights from file
bool RobotController::load_joint_weights(char* file_path)
{
    // --------------- Get the number of joint_weights stored in the file --------------------------
    string line;
    int num_vals_per_line = 0;
    bool num_vals_read = false;

    char * pEnd;
    char char_line[1000];

    ifstream jwVal_file(file_path);
    if (jwVal_file.is_open())
    {
        while ( jwVal_file.good() )
        {
             std::getline(jwVal_file,line);

             //Get the number of values per line once
             if (num_vals_read == false)
             {
                 istringstream buf(line);
                 istream_iterator<string> beg(buf), end;
                 vector<string> substrings(beg, end); // done!
                 //Set flag to true
                 num_vals_read = true;
                 //Get number of values per line
                 num_vals_per_line = substrings.size();
             }

        }
        //std::cout <<"Line has " <<num_vals_per_line << " joint weights" << std::endl;
        jwVal_file.close();
    }

    else
    {
        std::cout << "Unable to open file (load_joint_weights)";
        return false;
    }

    //--------------- Fill the Joint Weights Vector --------------------------
    //Row number counter
    int curr_row_num = 1;

    //Open File
    jwVal_file.open(file_path);

    if (jwVal_file.is_open())
    {
        while (jwVal_file.good() )
        {
         //read next line from file (either a configuration or eof)
         std::getline (jwVal_file,line);

         //Detect when end of file is reached
         if (jwVal_file.eof())
             break;

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp = char_line;

         //Get current endeffector pose of frame
         //vector<double> curr_ee_pose;
         for (int i = 0 ; i < num_vals_per_line ; i++)
         {
           if (curr_row_num > 1) //row 1 = joint names
           {
               joint_weights_(i) = strtod(tmp,&pEnd);
               //cout<<"Weight for joint "<<joint_names_[i]<<" is: "<<joint_weights_(i)<<endl;
           }
           else
               break;

           //Assign the remaining char elements (pEnd) to the current char pointer
           tmp = pEnd;

         }
         //cout<<endl;


         //Increment row counter
         curr_row_num++;

        }

        //Close file
        jwVal_file.close();

        //True is returned when joint values for the subject and motion segment is available
        return true;
    }

    else
    {
        std::cout << "Unable to open file (load_joint_weights)";
        return false;
    }
}


//Adapt the joint weight (for sequential and circular motion strategy)
//void RobotController::adapt_joint_weights(int joint_num)
//{
//    //Activate joint sequentially with equal weights (activate one after the other, the first then add the second and so on)
//    if (pd_motion_strategy_ == 2)
//    {
//        if (joint_num == index_last_joint_activated_ && joint_num < num_joints_-1)
//        {
//            if (q_dot_out_(joint_num) <= min_change_rate_)
//            {
//                //Activate next joint
//                joint_weights_(joint_num+1) = 1.0;
//                //Update index of last joint activated
//                index_last_joint_activated_++;
//            }
//        }
//    }

//    //Circular joint activation (only one joint active at a time)
//    if (pd_motion_strategy_ == 3)
//    {
//        if (joint_num == index_last_joint_activated_ && joint_num <= num_joints_-1)
//        {
//            if (q_dot_out_(joint_num) <= min_change_rate_)
//            {
//                //Stop current joint
//                joint_weights_(joint_num) = 0.0;
//                //Activate next joint
//                if (joint_num < num_joints_-1)
//                {
//                    joint_weights_(joint_num+1) = 1.0;
//                    index_last_joint_activated_++;
//                }
//                //Activate first joint (when last joint has reached the minimal change rate)
//                if (joint_num == num_joints_-1)
//                {
//                    joint_weights_(0) = 1.0;
//                    index_last_joint_activated_ = 0;
//                }
//            }
//        }
//    }

//     //cout<<"Weight for joint "<<kin_chain_.getSegment(joint_num).getJoint().getName()<<" is: "<<joint_weights_(joint_num)<<endl;

//}


//Move only joint with the highest q_dot_out (i.e. the one minimizing the error at most)
//void RobotController::adapt_joint_weights()
//{
//    int max_q_vel_entry_index = 0;
//    //Find entry in joint velocity vector with the highest value
//    for (int j = 0 ; j < q_dot_out_.rows() ; j++)
//    {
//        if (fabs(q_dot_out_(max_q_vel_entry_index)) < fabs(q_dot_out_(j)))
//        {
//            max_q_vel_entry_index = j;
//        }
//    }
//    //Reset all other joint velocities to zero
//    for (int k = 0 ; k < q_dot_out_.rows() ; k++)
//    {
//        if (k != max_q_vel_entry_index)
//            q_dot_out_(k) = 0.0;
//    }
//}


//Show intermediate configuration of IK loop
void RobotController::show_ik_intermediate_config(double sleep_duration = 0.02)
{

    //Set name of planning scene service
    const std::string PLANNING_SCENE_SERVICE = m_planning_scene_service;

    //Get curent planning scene
    psm_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
    ps->getCurrentStateNonConst().update();
    //if you want to modify it
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();


    robot_state::RobotState state(psm_->getRobotModel());
    state.setToDefaultValues();

    //Set current robot state
    state.setVariablePositions(nvalues_);

    //Apply robot state to planning scene
    //psm_->getPlanningScene()->setCurrentState(state);
    scene->setCurrentState(state);

    //Publish state on planning scene
    moveit_msgs::PlanningScene psmsg;
    scene->getPlanningSceneMsg(psmsg);
    psmsg.robot_state.is_diff = true;
    psmsg.is_diff = true;
    scene_pub_.publish(psmsg);
    ros::Duration(sleep_duration).sleep();

    //cout<<"Published"<<endl;
}


//Update the error and its norm(pos+orientation)
void RobotController::update_error_norm(vector<double> curr_ee_pose, vector<double> des_ee_pose, bool error_clamping)
{

    //Orientation Error computation based on literature:
    // - Robotics Modeling Planning and Control.pdf (page 140), Equation (3.91)

    //Skew-symmetric matrix from endeffector goal pose
    vector< vector<double> > current_goal_ee_pose_skew_mat;
    vector<double> skew_mat_first_row, skew_mat_second_row, skew_mat_third_row;
    skew_mat_first_row.push_back(0.0);
    skew_mat_first_row.push_back(-des_ee_pose[5]);
    skew_mat_first_row.push_back(des_ee_pose[4]);

    skew_mat_second_row.push_back(des_ee_pose[5]);
    skew_mat_second_row.push_back(0.0);
    skew_mat_second_row.push_back(-des_ee_pose[3]);

    skew_mat_third_row.push_back(-des_ee_pose[4]);
    skew_mat_third_row.push_back(des_ee_pose[3]);
    skew_mat_third_row.push_back(0.0);

    current_goal_ee_pose_skew_mat.push_back(skew_mat_first_row);
    current_goal_ee_pose_skew_mat.push_back(skew_mat_second_row);
    current_goal_ee_pose_skew_mat.push_back(skew_mat_third_row);


    //Upper bounds for error components
    vector <double> error_upper_bounds(error_.size());
    error_upper_bounds[0] = 0.01;//X bound
    error_upper_bounds[1] = 0.01;//Y bound
    error_upper_bounds[2] = 0.01;//Z bound
    error_upper_bounds[3] = 0.087; //= 40 degree //rotX bound (rad)
    error_upper_bounds[4] = 0.087; //= 40 degree//rotY bound (rad)
    error_upper_bounds[5] = 0.087; //= 40 degree//rotZ bound (rad)

    //Compute new error
    double error_sum = 0.0;
    double error_bound_sum = 0.0;
    for(int i = 0; i < error_.size() ; i++)
    {
        //        //Don't care variable
        //        if (des_ee_pose[i] == 1000.0)
        //             error_[i] = 0.0;
        //        //Position error
        //        if ( i < 3 && des_ee_pose[i] != 1000.0)
        //            error_[i] = des_ee_pose[i] - curr_ee_pose[i];
        //        //Orientation error
        //        if ( i >= 3 && des_ee_pose[i] != 1000.0)
        //        {
        //            error_[i] = curr_ee_pose[6] * des_ee_pose[i] - des_ee_pose[6] * curr_ee_pose[i] - (current_goal_ee_pose_skew_mat[i-3][0] * curr_ee_pose[3] + current_goal_ee_pose_skew_mat[i-3][1] * curr_ee_pose[4] + current_goal_ee_pose_skew_mat[i-3][2] * curr_ee_pose[5]);
        //        }


        //Don't care variable
        if (var_constraint_vec_[i] == 0)
             error_[i] = 0.0;
        //Position error
        if ( i < 3 && var_constraint_vec_[i] == 1)
            error_[i] = des_ee_pose[i] - curr_ee_pose[i];
        //Orientation error
        if ( i >= 3 && var_constraint_vec_[i] == 1)
        {
            error_[i] = curr_ee_pose[6] * des_ee_pose[i] - des_ee_pose[6] * curr_ee_pose[i] - (current_goal_ee_pose_skew_mat[i-3][0] * curr_ee_pose[3] + current_goal_ee_pose_skew_mat[i-3][1] * curr_ee_pose[4] + current_goal_ee_pose_skew_mat[i-3][2] * curr_ee_pose[5]);
        }

        error_sum += (error_[i]*error_[i]);
        error_bound_sum += (error_upper_bounds[i]*error_upper_bounds[i]);
    }

    //Compute error norm
    error_norm_ =sqrt(error_sum);

    //Compute Norm/Length of error_bound (i.e. maximum length of error vector)
    double error_bound_norm = sqrt(error_bound_sum);


    //Clamping the magnitude of the error
    // -> reduce oscillation when target is out of reach
    // -> allows to use smaller damping factor thus quicker convergence of DLS solution
    if (error_clamping == true)
        clamp_error_magnitude(error_bound_norm);

//    cout<<"Current error: "<<error_norm_<<endl;
//    cout<<"X: "<<error_[0]<<endl;
//    cout<<"Y: "<<error_[1]<<endl;
//    cout<<"Z: "<<error_[2]<<endl;
//    cout<<"rotX: "<<error_[3]<<endl;
//    cout<<"rotY: "<<error_[4]<<endl;
//    cout<<"rotZ: "<<error_[5]<<endl;

    //cout<<"Current error norm: "<<error_norm_<<endl;
    //cout<<"Maximum error norm: "<<error_bound_norm<<endl;
}


//Update the error vector(pos+orientation)
void RobotController::update_error_vec(vector<double> curr_ee_pose, vector<double> des_ee_pose)
{
    //Orientation Error computation based on literature:
    // - Robotics Modeling Planning and Control.pdf (page 140), Equation (3.91)

    //Skew-symmetric matrix from endeffector goal pose
    vector< vector<double> > current_goal_ee_pose_skew_mat;
    vector<double> skew_mat_first_row, skew_mat_second_row, skew_mat_third_row;
    skew_mat_first_row.push_back(0.0);
    skew_mat_first_row.push_back(-des_ee_pose[5]);
    skew_mat_first_row.push_back(des_ee_pose[4]);

    skew_mat_second_row.push_back(des_ee_pose[5]);
    skew_mat_second_row.push_back(0.0);
    skew_mat_second_row.push_back(-des_ee_pose[3]);

    skew_mat_third_row.push_back(-des_ee_pose[4]);
    skew_mat_third_row.push_back(des_ee_pose[3]);
    skew_mat_third_row.push_back(0.0);

    current_goal_ee_pose_skew_mat.push_back(skew_mat_first_row);
    current_goal_ee_pose_skew_mat.push_back(skew_mat_second_row);
    current_goal_ee_pose_skew_mat.push_back(skew_mat_third_row);


    //Upper bounds for error components
    vector <double> error_upper_bounds(error_.size());
    error_upper_bounds[0] = 0.01;//X bound
    error_upper_bounds[1] = 0.01;//Y bound
    error_upper_bounds[2] = 0.01;//Z bound
    error_upper_bounds[3] = 0.087; //= 40 degree //rotX bound (rad)
    error_upper_bounds[4] = 0.087; //= 40 degree//rotY bound (rad)
    error_upper_bounds[5] = 0.087; //= 40 degree//rotZ bound (rad)


    //Compute new error components
    for(int i = 0; i < error_.size() ; i++)
    {
        //Don't care variable
        if (var_constraint_vec_[i] == 0)
             error_[i] = 0.0;
        if (var_constraint_vec_[i] == 1)
        {
            if (i < 3) //Position error
            {
                error_[i] = des_ee_pose[i] - curr_ee_pose[i];
            }
            else //Orientation error
            {
               error_[i] = curr_ee_pose[6] * des_ee_pose[i] - des_ee_pose[6] * curr_ee_pose[i] - (current_goal_ee_pose_skew_mat[i-3][0] * curr_ee_pose[3] + current_goal_ee_pose_skew_mat[i-3][1] * curr_ee_pose[4] + current_goal_ee_pose_skew_mat[i-3][2] * curr_ee_pose[5]);
            }
        }
    }


    //Check whether constraint coordinates are within their permitted deviation boundaries
    for (int i = 0; i < var_constraint_vec_.size(); i++)
    {
        //Is the coordinate constraint?
        if(var_constraint_vec_[i] == 1)
        {
            //Set error to zero if deviation is within tolerance otherwise keep the error
            //  coordinate_dev[i][0] = lower bound and  //  coordinate_dev[i][1] = upper bound
            error_[i] = (error_[i] < coordinate_dev_[i].first || error_[i] > coordinate_dev_[i].second) ? error_[i] : 0.0;
        }
        else
        {
            //Coordinate is unconstraints
            error_[i] = 0.0;
        }

        //cout<<coordinate_dev[i].first<<" "<<coordinate_dev[i].second<<endl;
    }
    //cout<<endl;


    //ROS_INFO_STREAM("Current error variable damped least squares:"<<error_[0]<<" "<<error_[1]<<" "<<error_[2]<<" "<<error_[3]<<" "<<error_[4]<<" "<<error_[5]);

}


//Clamping the magnitude of the error
void RobotController::clamp_error_magnitude(double error_norm_upper_bound)
{
    //Clamped error sum
    double clamped_error_sum = 0;

    //Clamp error vector if length is larger than a given upper error length bound
    if (error_norm_upper_bound < error_norm_)
    {
        for(int i = 0; i < error_.size() ; i++)
        {
            error_[i] = error_norm_upper_bound * (error_[i]/error_norm_);

            clamped_error_sum += (error_[i]*error_[i]);
        }

        //Compute clamped error norm
        error_norm_ =sqrt(clamped_error_sum);

        //cout<<"Error vector clamped to maximum"<<endl;
    }
    else
    {
        //cout<<"No error clamping performed"<<endl;
    }

}


//Control Approach: Jacobian Pseudoinverse Method
bool RobotController::run_J_pinv_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data)
{
    //Eigen Matrices storing the Jacobian for the chain
    Eigen::MatrixXf eigen_current_jac(dim_task_space_,num_joints_);
    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_jac_pinv;
    //Null Space of the Jacobian (arm_chain)
    double **null_space_projector;
    //Value of the JL-Gradient function (to be used in the control loop)
    vector<double> jl_grad_func_val(num_joints_);

    //Init iteration counter
    int current_iter = 0;

    //Init goal pose index
    int current_goal_pose_index = 0;

    while (current_goal_pose_index < num_ee_poses_)
    {
        //Reset iteration counter
        current_iter = 0;

        if (traj_tracking_enabled_ == true)
        {
            //Set current goal pose
            //cout<<"Current ee goal pose num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_pose;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_pose.push_back(ee_ref_pos_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_pos_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_pose_ = curr_goal_pose;

            //Get EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Update the error
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);


            //Set current goal vel
            //cout<<"Current ee goal vel num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_vel;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_vel.push_back(ee_ref_vel_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_vel_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal velocity
            current_goal_ee_vel_ = curr_goal_vel;


            //Enter new map into the "configurations" vector
            if (store_traj_data == true)
                configurations_.push_back(conf_);

        }


        //------Numerical inverse kinematic loop --------
        //while (error_norm_ > error_treshold_ || jl_ok_ == false || traj_tracking_enabled_ == true)
        while (error_norm_ > error_treshold_ || traj_tracking_enabled_ == true)
        {
            if (traj_tracking_enabled_ == false && store_traj_data == true)
            {
                //Enter new map into the "configurations" vector
                configurations_.push_back(conf_);
            }

            //Compute jacobian for current configuration of body chain
            eigen_current_jac = getJacobian(kin_chain_,current_config_);

            //JL Gradient computation (used in methods using Null Space Projection Method)
            jl_grad_func_val = computeJLgradient(current_config_, q_min_, q_max_, jl_activation_param_);


            //1)Pseudoinverse (unstable in the neighbourhood of singularities)  Null Space Optimization (for joint limit avoidance)
            current_jac_pinv = compute_J_pinv(eigen_current_jac);

            //Compute the null space of the pseudoinvere
            null_space_projector = compute_null_space_projection(eigen_current_jac,current_jac_pinv);

            //++++++ Compute joint velocity
            for (int i = 0; i < num_joints_ ; i++)
            {
                //Init q_dot_out
                q_dot_out_(i) = 0.0;

                //Compute first term of control law -> J# * error
                //if(jl_ok_ == true)
                //{
                    for (int j = 0; j < dim_task_space_ ; j++)
                        q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + error_gain_ * error_[j]);
                //}

                //cout<<"Primary task vel for joint: "<<i+1<<" is: "<<q_dot_out_(i)<<endl;


                //Compute term of control law -> (I - J#J) * jl_gradient
                double q_dot_sec_task = 0.0;
                for (int s = 0; s < num_joints_ ; s++)
                {
                   q_dot_sec_task = q_dot_sec_task + (null_space_projector[i][s] * jl_grad_func_val[s]);
                   //if(current_iter < 5)
                   //   cout<<jl_grad_func_val[s]<<"  ";

                }
                //if(current_iter < 5)
                //cout<<endl;

                //cout<<"Secondary task vel for joint: "<<i+1<<" is: "<<q_dot_sec_task<<endl;

                //Add second term to first joint velocoty term
                q_dot_out_(i) = joint_weights_(i) * (q_dot_out_(i) + gain_sec_task_ * q_dot_sec_task);

                //cout<<"Total task vel for joint: "<<i+1<<" is: "<<q_dot_out_(i)<<endl;


                //Weight Joint velocity according to the PD strategy (joint weights are now considered in the jacobian pseudoinversion)
                //q_dot_out_(i) = joint_weights_(i) *  q_dot_out_(i);

            }

            //cout<<endl;


            //++++++ Update Joint Value
            //cout<<"New Configuration: "<<endl;
            //Current element index
            int curr_idx = 0;
            //Joint Limits check
            bool jl_respected = true;
            for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
            {
                if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
                {
                    //Store new joint value in temporary variable
                    double new_joint_val = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

                    //Check if joint is within joint limits
                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, false))
                    {
                        //Joint Limits are violated
                        jl_respected = false;

                        //Note: Keep fomer joint value, i.e. no update

//                        //Lower joint value bound violated
//                        if (new_joint_val < q_min_[curr_idx])
//                            current_config_(curr_idx) = q_min_[curr_idx] + 0.05; //0.05 rad = 3 degree distance to joint range border
//                        //Upper joint value bound violated
//                        if (q_max_[curr_idx] < new_joint_val)
//                            current_config_(curr_idx) = q_max_[curr_idx] - 0.05; //0.05 rad = 3 degree distance to joint range border
                    }
                    else
                    {
                        //Joint Limits respected
                        jl_respected = true;

                        //Assign new joint value to current joint value
                        current_config_(curr_idx) = new_joint_val;
                    }


                    //Check if joint is within joint limits
                    RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),current_config_(curr_idx), true);


                     //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

                    //Set Configuration to be shown by "show_ik_intermediate_config"-function
                    nvalues_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    //Set map entry
                    conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    curr_idx++;
                }
            }
            //When at least one of the joints violates the joint limits
            if(!jl_respected)
                 jl_ok_ = false;
             else
                 jl_ok_ = true;



//            //++++++ Check joint limits
//            int config_element = 0;
//            bool jl_respected = true;
//            for (unsigned int i = 0; i != kin_chain_.getNrOfSegments() ; i++)
//            {
//                if (kin_chain_.getSegment(i).getJoint().getTypeName() != "None")
//                {
//                    //Check if joint is within joint limits
//                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(i).getJoint().getName(),current_config_(config_element)))
//                        jl_respected = false;

//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" has value: "<<current_config(config_element)<<endl;

//                    //Set Configuration to be shown by "show_ik_intermediate_config"-function
//                    nvalues_[kin_chain_.getSegment(i).getJoint().getName()] = current_config_(config_element);

//                    //Increment config vector element
//                    config_element++;
//                }
//                else
//                {
//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" is a fixed joint"<<endl;
//                }
//            }
//            //When at least one of the joints violates the joint limits
//            if(!jl_respected)
//                 jl_ok_ = false;
//             else
//                 jl_ok_ = true;



            //Get new EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Store current endeffector pose
            if(store_traj_data == true)
                ee_poses_.push_back(current_ee_pose);


            //++++++ ERROR UPDATE
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);


            //++++++ Show Motion
            if(show_motion == 1 || current_iter == max_iter-1 || error_norm_ < error_treshold_)
                show_ik_intermediate_config(sleep_duration_between_confs_);


            //Show endeffector trace
            if(show_ee_trace == true)
                RobotModel->show_ee_trace(current_ee_pose,color_rgb_,max_ee_trace_points_);

            //Stop numerical IK iteration
            if (error_norm_ < error_treshold_ && jl_ok_==true)
                  break;


            //++++++ Increment iteration counter
            current_iter++;
            //cout<<current_iter<<endl;
            //Check whether max_iterations has been reached
            if(current_iter == max_iter)
            {
                cout<<"Maximum iterations reached"<<endl;

                cout<<"Final error: "<<endl;
                cout<<"X: "<<error_[0]<<endl;
                cout<<"Y: "<<error_[1]<<endl;
                cout<<"Z: "<<error_[2]<<endl;
                cout<<"rotX: "<<error_[3]<<endl;
                cout<<"rotY: "<<error_[4]<<endl;
                cout<<"rotZ: "<<error_[5]<<endl;

                cout<<"Final error norm: "<<error_norm_<<endl<<endl;
                break;
            }

            //Only perform one ik iteration is trajectory tracking is enabled
            //if (traj_tracking_enabled_ == true)
            //    break;

        } //end of while loop


        //Switch to next endeffector pose of the trajectory
        current_goal_pose_index++;

    }


    //Delete endeffector trace (for next ee trajectory)
    RobotModel->delete_ee_trace();

    if(store_traj_data == true)
    {
        //Write joint trajectory, i.e. the sequence of configurations to a file
        RobotModel->writeJointTrajectoryToFile(configurations_, w_joints_, file_path_joint_trajectory_);
        //Clear vector storing configurations
        configurations_.clear();

        //Write endeffector trajectory to a file
        RobotModel->writeEndeffectorTrajectoryToFile(ee_poses_, file_path_endeffector_trajectory_);

        //Clear vector storing ee poses
        ee_poses_.clear();
    }



    //If the maximum iterations has been reached without reducing the error norm below 'error_treshold_'
    if (current_iter == max_iter)
        return false;
    else
    {
        cout<<"Goal pose reached"<<endl;

        cout<<"Final error: "<<endl;
        cout<<"X: "<<error_[0]<<endl;
        cout<<"Y: "<<error_[1]<<endl;
        cout<<"Z: "<<error_[2]<<endl;
        cout<<"rotX: "<<error_[3]<<endl;
        cout<<"rotY: "<<error_[4]<<endl;
        cout<<"rotZ: "<<error_[5]<<endl;

        cout<<"Final error norm: "<<error_norm_<<endl<<endl;
        return true;    //error norm is smaller than 'error_treshold_'
    }

}


bool RobotController::run_J_dls_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data)
{

    //Eigen Matrices storing the Jacobian for the chain
    Eigen::MatrixXf eigen_current_jac(dim_task_space_,num_joints_);
    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_jac_pinv;
    //Null Space of the Jacobian (arm_chain)
    double **null_space_projector;
    //Value of the JL-Gradient function (to be used in the control loop)
    vector<double> jl_grad_func_val(num_joints_);

    //Init iteration counter
    int current_iter = 0;

    //Init goal pose index
    int current_goal_pose_index = 0;

    //Reset vectors storing configurations and ee poses
    configurations_.clear(); //Clear vector storing configurations
    ee_poses_.clear(); //Clear vector storing ee poses


    while (current_goal_pose_index < num_ee_poses_)
    {
        //Reset iteration counter
        current_iter = 0;

        if (traj_tracking_enabled_ == true)
        {
            //Set current goal pose
            //cout<<"Current ee goal pose num: "<<current_goal_pose_index<<endl;
            //cout<<"num_ee_poses: "<<num_ee_poses_<<endl;
            vector <double> curr_goal_pose;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_pose.push_back(ee_ref_pos_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_pos_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_pose_ = curr_goal_pose;

            //Get EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Update the error
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);

            //Set current goal vel
            //cout<<"Current ee goal vel num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_vel;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_vel.push_back(ee_ref_vel_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_vel_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_vel_ = curr_goal_vel;


            //Enter new map into the "configurations" vector
            configurations_.push_back(conf_);
        }

        //------Numerical inverse kinematic loop --------
        //while (error_norm_ > error_treshold_ || jl_ok_ == false || traj_tracking_enabled_ == true)
        while (error_norm_ > error_treshold_ || traj_tracking_enabled_ == true)
        {
            if (traj_tracking_enabled_ == false)
            {
                //Enter new map into the "configurations" vector
                configurations_.push_back(conf_);
            }

            //Compute jacobian for current configuration of body chain
            eigen_current_jac = getJacobian(kin_chain_,current_config_);

            //JL Gradient computation (used in methods using Null Space Projection Method)
            jl_grad_func_val = computeJLgradient(current_config_, q_min_, q_max_, jl_activation_param_);

            //2)Damped-least squares Pseudoinverse (stable in the neighbourhood of singularities)  Null Space Optimization (for joint limit avoidance)
            current_jac_pinv = compute_J_dls(eigen_current_jac, const_lambda_fac_);

            //Compute the null space of the pseudoinvere
            null_space_projector = compute_null_space_projection(eigen_current_jac,current_jac_pinv);

            //++++++ Compute joint velocity
            for (int i = 0; i < num_joints_ ; i++)
            {
                //Init q_dot_out
                q_dot_out_(i) = 0.0;

                //Compute first term of control law -> J * error
                //if (jl_ok_ == true)
                //{
                    for (int j = 0; j < dim_task_space_ ; j++)
                          q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + error_gain_ * error_[j]);
                //}

                //cout<<"Primary task q_dot:"<<endl;
                //cout<<q_dot_out_(i)<<endl;

                 //Compute term of control law -> (I - J'J) * jl_gradient
                 double q_dot_sec_task = 0.0;
                 for (int s = 0; s < num_joints_ ; s++)
                 {
                    q_dot_sec_task = q_dot_sec_task + (null_space_projector[i][s] * jl_grad_func_val[s]);
                 }

                 //cout<<"Secondary task q_dot:"<<endl;
                 //cout<<gain_sec_task_ * q_dot_sec_task<<endl;

                 //Add second term to first joint velocoty term
                 q_dot_out_(i) = q_dot_out_(i) + gain_sec_task_ * q_dot_sec_task;

                 //cout<<"Final q_dot:"<<endl;
                 //cout<<q_dot_out_(i)<<endl;


                 //Weight Joint velocity according to the PD strategy (joint weights are now considered in the jacobian pseudoinversion)
                 q_dot_out_(i) = joint_weights_(i) *  q_dot_out_(i);

            }


            //++++++ Update Joint Value
            //cout<<"New Configuration: "<<endl;
            //Current element index
            int curr_idx = 0;
            //Joint Limits check
            bool jl_respected = true;
            for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
            {
                if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
                {
                    //Store new joint value in temporary variable
                    double new_joint_val = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

                    //Check if joint is within joint limits
                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, false))
                    {
                        //Joint Limits are violated
                        jl_respected = false;

                        //Note: Keep fomer joint value, i.e. no update

//                        //Lower joint value bound violated
//                        if (new_joint_val < q_min_[curr_idx])
//                            current_config_(curr_idx) = q_min_[curr_idx] + 0.05; //0.05 rad = 3 degree distance to joint range border
//                        //Upper joint value bound violated
//                        if (q_max_[curr_idx] < new_joint_val)
//                            current_config_(curr_idx) = q_max_[curr_idx] - 0.05; //0.05 rad = 3 degree distance to joint range border
                    }
                    else
                    {
                        //Joint Limits respected
                        jl_respected = true;

                        //Assign new joint value to current joint value
                        current_config_(curr_idx) = new_joint_val;
                    }

                     //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

                    //Set Configuration to be shown by "show_ik_intermediate_config"-function
                    nvalues_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    //Set map entry
                    conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    curr_idx++;
                }
            }
            //When at least one of the joints violates the joint limits
            if(!jl_respected)
                 jl_ok_ = false;
             else
                 jl_ok_ = true;


//            //++++++ Check joint limits
//            int config_element = 0;
//            bool jl_respected = true;
//            for (unsigned int i = 0; i != kin_chain_.getNrOfSegments() ; i++)
//            {
//                if (kin_chain_.getSegment(i).getJoint().getTypeName() != "None")
//                {
//                    //Check if joint is within joint limits
//                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(i).getJoint().getName(),current_config_(config_element)))
//                        jl_respected = false;

//                    //cout<<"Joint "<<kin_chain_.getSegment(i).getJoint().getName()<<" has value: "<<current_config_(config_element)<<endl;
//                    nvalues_[kin_chain_.getSegment(i).getJoint().getName()] = current_config_(config_element);

//                    //Increment config vector element
//                    config_element++;
//                }
//                else
//                {
//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" is a fixed joint"<<endl;
//                }
//            }


//            //When at least one of the joints violates the joint limits
//            if(!jl_respected)
//                 jl_ok_ = false;
//             else
//                 jl_ok_ = true;


            //Get new EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Store current endeffector pose
            ee_poses_.push_back(current_ee_pose);

            //++++++ ERROR UPDATE
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);

            //++++++ Show Motion
            if(show_motion == 1 || current_iter == max_iter-1 || error_norm_ < error_treshold_)
                show_ik_intermediate_config(sleep_duration_between_confs_);


            //Show endeffector trace
            if(show_ee_trace == true)
                RobotModel->show_ee_trace(current_ee_pose,color_rgb_,max_ee_trace_points_);

            //Stop numerical IK iteration
            if (error_norm_ < error_treshold_ && jl_ok_==true)
                  break;



            //++++++ Increment iteration counter
            current_iter++;
            //Check whether max_iterations has been reached
            if(current_iter == max_iter)
            {
                cout<<"Maximum iterations reached"<<endl;

                cout<<"Final error: "<<endl;
                cout<<"X: "<<error_[0]<<endl;
                cout<<"Y: "<<error_[1]<<endl;
                cout<<"Z: "<<error_[2]<<endl;
                cout<<"rotX: "<<error_[3]<<endl;
                cout<<"rotY: "<<error_[4]<<endl;
                cout<<"rotZ: "<<error_[5]<<endl;

                cout<<"Final error norm: "<<error_norm_<<endl<<endl;
                break;
            }

            //Only perform one ik iteration is trajectory tracking is enabled
            //if (traj_tracking_enabled_ == true)
            //    break;

        } //end of ik loop

        //Switch to next endeffector pose of the trajectory
        current_goal_pose_index++;

    }



    //Delete endeffector trace (for next ee trajectory)
    RobotModel->delete_ee_trace();

    if(store_traj_data == true)
    {
        //Write joint trajectory, i.e. the sequence of configurations to a file
        RobotModel->writeJointTrajectoryToFile(configurations_, w_joints_, file_path_joint_trajectory_);

        //Write endeffector trajectory to a file
        RobotModel->writeEndeffectorTrajectoryToFile(ee_poses_, file_path_endeffector_trajectory_);
    }




    //If the maximum iterations has been reached without reducing the error norm below 'error_treshold_'
    if (current_iter == max_iter)
        return false;
    else
    {
        cout<<"Goal pose reached"<<endl;

        cout<<"Final error: "<<endl;
        cout<<"X: "<<error_[0]<<endl;
        cout<<"Y: "<<error_[1]<<endl;
        cout<<"Z: "<<error_[2]<<endl;
        cout<<"rotX: "<<error_[3]<<endl;
        cout<<"rotY: "<<error_[4]<<endl;
        cout<<"rotZ: "<<error_[5]<<endl;

        cout<<"Final error norm: "<<error_norm_<<endl<<endl;
        return true;    //error norm is smaller than 'error_treshold_'
    }

}



bool RobotController::run_J_vdls_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data)
{

    //Eigen Matrices storing the Jacobian for the chain
    Eigen::MatrixXf eigen_current_jac(dim_task_space_,num_joints_);
    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_jac_pinv;
    //Null Space of the Jacobian (arm_chain)
    double **null_space_projector;
    //Value of the JL-Gradient function (to be used in the control loop)
    vector<double> jl_grad_func_val(num_joints_);

    //Init iteration counter
    int current_iter = 0;

    //Variables for variable damping with DLS (based on manipulability analysis)
    min_manip_treshold_ = 0.03;

    //Init goal pose index
    int current_goal_pose_index = 0;

    //Reset vectors storing configurations and ee poses
    configurations_.clear(); //Clear vector storing configurations
    ee_poses_.clear(); //Clear vector storing ee poses


    //Repulsive vector
    vector<double> ee_rep_vec;
    //Control Points
    vector<vector<double> > ca_control_points;

    //Current joint velocity bounds (manipulated by robot body collision avoidance)
    vector<vector<double> > joint_vel_bounds;


    if(collision_avoidance_active_ == true)
    {
        //----------------- START: CA TEST -------------------------

        //Compute Control Points (Position of Spheres located alon the endeffector chain, subsequently used for collision avoidance)
        ca_control_points = compute_CA_ControlPoints(joint_indices_for_CA_);

        //Compute Repulsive vector (for obstacle avoidance)
        ee_rep_vec = compute_EE_RepulsiveVector(ca_control_points[0]);

        //Show first control point setting
        if (show_motion)
            RobotModel->show_CA_Control_Points(ca_control_points);

        //----------------- END: CA TEST -------------------------
    }



    while (current_goal_pose_index < num_ee_poses_)
    {
        //Reset iteration counter
        current_iter = 0;

        if (traj_tracking_enabled_ == true)
        {
            //Set current goal pose
            //cout<<"Current ee goal pose num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_pose;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_pose.push_back(ee_ref_pos_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_pos_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_pose_ = curr_goal_pose;

            //Get EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);


            //Update the error
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);

            //Set current goal vel
            //cout<<"Current ee goal vel num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_vel;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_vel.push_back(ee_ref_vel_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_vel_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_vel_ = curr_goal_vel;

            //Enter new map into the "configurations" vector
            configurations_.push_back(conf_);
        }


        //------Numerical inverse kinematic loop --------
        //while (error_norm_ > error_treshold_ || jl_ok_ == false || traj_tracking_enabled_ == true)
        while (error_norm_ > error_treshold_ || traj_tracking_enabled_ == true)
        {
            if (traj_tracking_enabled_ == false)
            {
                //Enter new map into the "configurations" vector
                configurations_.push_back(conf_);
            }

            //Compute jacobian for current configuration of body chain
            eigen_current_jac = getJacobian(kin_chain_,current_config_);

            if(collision_avoidance_active_ == true)
            {
                //Compute joint velocity bounds for robot body collision avoidance
                joint_vel_bounds = compute_risk_func_vector(ca_control_points,eigen_current_jac);
            }

            //JL Gradient computation (used in methods using Null Space Projection Method)
            jl_grad_func_val = computeJLgradient(current_config_, q_min_, q_max_, jl_activation_param_);


            //Compute the Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
            current_jac_pinv = compute_J_vdls(eigen_current_jac, min_manip_treshold_, max_damping_factor_);


            //Compute the null space of the pseudoinvere
            null_space_projector = compute_null_space_projection(eigen_current_jac,current_jac_pinv);


            //++++++ Compute joint velocity
            for (int i = 0; i < num_joints_ ; i++)
            {
                //Init q_dot_out
                q_dot_out_(i) = 0.0;

                //Compute first term of control law -> J * error
                //if(jl_ok_ == true)
                //{
                    for (int j = 0; j < dim_task_space_ ; j++)
                    {
                        if(collision_avoidance_active_ == true)
                            q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + ee_rep_vec[j] + error_gain_ * error_[j]);
                        else
                            q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + error_gain_ * error_[j]);

                    }
                //}

                //Compute term of control law -> (I - J'J) * jl_gradient
                double q_dot_sec_task = 0.0;
                for (int s = 0; s < num_joints_ ; s++)
                   q_dot_sec_task = q_dot_sec_task + (null_space_projector[i][s] * jl_grad_func_val[s]);

                //cout<<q_dot_sec_task<<" "<<endl;

                //Add second term to first joint velocoty term
                q_dot_out_(i) = q_dot_out_(i) + gain_sec_task_ * q_dot_sec_task;


                //Weight Joint velocity according to the PD strategy (joint weights are now considered in the jacobian pseudoinversion)
                q_dot_out_(i) = joint_weights_(i) *  q_dot_out_(i);

                //Joint velocity/motion scaling for collision avoidance (for robot body parts / without endeffector)
                if(collision_avoidance_active_ == true)
                {
                    //cout<<"before:"<<q_dot_out_(i)<<endl;
                    q_dot_out_(i) = joint_vel_bounds[i][0] < q_dot_out_(i) ? joint_vel_bounds[i][0] : q_dot_out_(i);
                    //cout<<"after 1:"<<q_dot_out_(i)<<endl;
                    q_dot_out_(i) = q_dot_out_(i) < joint_vel_bounds[i][1]  ? joint_vel_bounds[i][1] : q_dot_out_(i);
                    //cout<<"after 2:"<<q_dot_out_(i)<<endl;
                }

            }
            //cout<<endl;


            //++++++ Update Joint Value
            //cout<<"New Configuration: "<<endl;
            //Current element index
            int curr_idx = 0;
            //Joint Limits check
            bool jl_respected = true;
            for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
            {
                if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
                {
                    //Store new joint value in temporary variable
                    double new_joint_val = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

                    //Check if joint is within joint limits
                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, true))
                    {
                        //Joint Limits are violated
                        jl_respected = false;

                        //Note: Keep fomer joint value, i.e. no update

//                        //Lower joint value bound violated
//                        if (new_joint_val < q_min_[curr_idx])
//                            current_config_(curr_idx) = q_min_[curr_idx] + 0.05; //0.05 rad = 3 degree distance to joint range border
//                        //Upper joint value bound violated
//                        if (q_max_[curr_idx] < new_joint_val)
//                            current_config_(curr_idx) = q_max_[curr_idx] - 0.05; //0.05 rad = 3 degree distance to joint range border
                    }
                    else
                    {
                        //Joint Limits respected
                        jl_respected = true;

                        //Assign new joint value to current joint value
                        current_config_(curr_idx) = new_joint_val;
                    }

                     //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

                    //Set Configuration to be shown by "show_ik_intermediate_config"-function
                    nvalues_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    //Set map entry
                    conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    curr_idx++;
                }
            }
            //When at least one of the joints violates the joint limits
            if(!jl_respected)
                 jl_ok_ = false;
             else
                 jl_ok_ = true;



            //Get new EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

//            cout<<"EE pos:"<<endl;
//            cout<<"X:"<<current_ee_pose[0]<<endl;
//            cout<<"Y:"<<current_ee_pose[1]<<endl;
//            cout<<"Z:"<<current_ee_pose[2]<<endl;

            //Store current endeffector pose
            ee_poses_.push_back(current_ee_pose);


            //++++++ ERROR UPDATE
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);


            //++++++ Show Motion
            if(show_motion == 1 || current_iter == max_iter-1 || error_norm_ < error_treshold_)
                show_ik_intermediate_config(sleep_duration_between_confs_);


            if(collision_avoidance_active_ == true)
            {
                //----------------- START: CA TEST -------------------------
                //Select control points

                //Compute Control Points (Position of Spheres located alon the endeffector chain, subsequently used for collision avoidance)
                ca_control_points = compute_CA_ControlPoints(joint_indices_for_CA_);

//                cout<<"CP pos:"<<endl;
//                cout<<"X:"<<ca_control_points[0][0]<<endl;
//                cout<<"Y:"<<ca_control_points[0][1]<<endl;
//                cout<<"Z:"<<ca_control_points[0][2]<<endl;

                //Show first control point setting
                if (show_motion)
                    RobotModel->show_CA_Control_Points(ca_control_points);


                //Compute Repulsive vector (for obstacle avoidance)
                ee_rep_vec = compute_EE_RepulsiveVector(ca_control_points[0]);

//                cout<<"Rep.Field X: "<<ee_rep_vec[0]<<endl;
//                cout<<"Rep.Field Y: "<<ee_rep_vec[1]<<endl;
//                cout<<"Rep.Field Z: "<<ee_rep_vec[2]<<endl;
//                cout<<"Rep.Field : "<<ee_rep_vec[3]<<endl;
//                cout<<"Rep.Field : "<<ee_rep_vec[4]<<endl;
//                cout<<"Rep.Field : "<<ee_rep_vec[5]<<endl;

                //----------------- END: CA TEST -------------------------
            }


            //Show endeffector trace
            if(show_ee_trace == true)
                RobotModel->show_ee_trace(current_ee_pose,color_rgb_,max_ee_trace_points_);

            //Stop numerical IK iteration
            if (error_norm_ < error_treshold_ && jl_ok_==true)
                  break;



            //++++++ Increment iteration counter
            current_iter++;
            //Check whether max_iterations has been reached
            if(current_iter == max_iter)
            {
                  cout<<"Maximum iterations reached"<<endl;

                cout<<"Final error: "<<endl;
                cout<<"X: "<<error_[0]<<endl;
                cout<<"Y: "<<error_[1]<<endl;
                cout<<"Z: "<<error_[2]<<endl;
                cout<<"rotX: "<<error_[3]<<endl;
                cout<<"rotY: "<<error_[4]<<endl;
                cout<<"rotZ: "<<error_[5]<<endl;

                cout<<"Final error norm: "<<error_norm_<<endl<<endl;
                break;
            }

            //Only perform one ik iteration is trajectory tracking is enabled
//            if (traj_tracking_enabled_ == true && current_goal_pose_index > 0)
//                max_iter = 5; //reset max iterations per ee goal pose once the first point of the trajectory has been reached

        } //end of ik loop

        //Switch to next endeffector pose of the trajectory
        current_goal_pose_index++;

    }


    //Delete endeffector trace (for next ee trajectory)
    RobotModel->delete_ee_trace();

    if(store_traj_data == true)
    {
        //Write joint trajectory, i.e. the sequence of configurations to a file
        RobotModel->writeJointTrajectoryToFile(configurations_, w_joints_, file_path_joint_trajectory_);

        //Write endeffector trajectory to a file
        RobotModel->writeEndeffectorTrajectoryToFile(ee_poses_, file_path_endeffector_trajectory_);

    }


    //If the maximum iterations has been reached without reducing the error norm below 'error_treshold_'
    if (current_iter == max_iter)
        return false;
    else
    {
        cout<<"Goal pose reached"<<endl;

//        cout<<"Final error: "<<endl;
//        cout<<"X: "<<error_[0]<<endl;
//        cout<<"Y: "<<error_[1]<<endl;
//        cout<<"Z: "<<error_[2]<<endl;
//        cout<<"rotX: "<<error_[3]<<endl;
//        cout<<"rotY: "<<error_[4]<<endl;
//        cout<<"rotZ: "<<error_[5]<<endl;

//        cout<<"Final error norm: "<<error_norm_<<endl<<endl;
        return true;    //error norm is smaller than 'error_treshold_'
    }

}




Status RobotController::run_VDLS_Control_Connector(int max_iter, vector<vector<double> > &joint_traj, vector<vector<double> > &ee_traj, bool show_motion, bool show_ee_trace)
{

    //Eigen Matrices storing the Jacobian for the chain
    Eigen::MatrixXf eigen_current_jac(dim_task_space_,num_joints_);
    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_jac_pinv;
    //Null Space of the Jacobian (arm_chain)
    //double **null_space_projector;
    //Value of the JL-Gradient function (to be used in the control loop)
    //vector<double> jl_grad_func_val(num_joints_);

    //Init iteration counter
    int current_iter = 0;

    //Variables for variable damping with DLS (based on manipulability analysis)
    min_manip_treshold_ = 0.03;

    //Init goal pose index
    int current_goal_pose_index = 0;

    //Reset vectors storing configurations and ee poses
    configurations_.clear(); //Clear vector storing configurations
    ee_poses_.clear(); //Clear vector storing ee poses

    //Current config as double vector (used for storing joint trajectory)
    vector<double> robot_config(num_joints_);

    //Repulsive vector
    vector<double> ee_rep_vec;
    //Control Points
    vector<vector<double> > ca_control_points;

    //Current joint velocity bounds (manipulated by robot body collision avoidance)
    vector<vector<double> > joint_vel_bounds;

    //Flag indicating when endeffector is stuck / does not make any significant progress towards goal anymore
    int deadlock_timeout = 500;
    bool controller_deadlock = false;

    //Reset Joint Velocity Integration for RRT* planner
    //delta_t_ = 0.04;  //0.025
    delta_t_ = 0.1;

    //Flag indicating whether all error w.r.t current target frame are within the specified bounds
    bool error_within_bounds = false;


    if(collision_avoidance_active_ == true)
    {
        //----------------- START: CA TEST -------------------------

        //Compute Control Points (Position of Spheres located alon the endeffector chain, subsequently used for collision avoidance)
        ca_control_points = compute_CA_ControlPoints(joint_indices_for_CA_);

        //Compute Repulsive vector (for obstacle avoidance)
        ee_rep_vec = compute_EE_RepulsiveVector(ca_control_points[0]);

        //Show first control point setting
        if (show_motion)
            RobotModel->show_CA_Control_Points(ca_control_points);

        //----------------- END: CA TEST -------------------------
    }



    while (current_goal_pose_index < num_ee_poses_)
    {
        //Reset iteration counter
        current_iter = 0;

        if (traj_tracking_enabled_ == true)
        {
            //Set current goal pose
            //cout<<"Current ee goal pose num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_pose;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_pose.push_back(ee_ref_pos_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_pos_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_pose_ = curr_goal_pose;

//            //Convert current configuration
//            for (int k = 0 ; k < num_joints_ ; k++)
//                robot_config[k] = current_config_(k);
//            //Store current configuration
//            joint_traj.push_back(robot_config);


            //Get EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Update the error
            update_error_vec(current_ee_pose, current_goal_ee_pose_);

            //check whether constraint coordinates are within the permitted bounds
            error_within_bounds = is_error_within_bounds(error_);

            //Set current goal vel
            //cout<<"Current ee goal vel num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_vel;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_vel.push_back(ee_ref_vel_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_vel_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_vel_ = curr_goal_vel;

            //Enter new map into the "configurations" vector
            //configurations_.push_back(conf_);
        }
        else
        {

            //Convert current configuration
            for (int k = 0 ; k < num_joints_ ; k++)
                robot_config[k] = current_config_(k);
            //Store current configuration
            joint_traj.push_back(robot_config);

            //Solve forward kinematics for chain and current configuration
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Store current ee pose
            ee_traj.push_back(current_ee_pose);
        }


        //------Numerical inverse kinematic loop --------
        //while (error_norm_ > error_threshold || jl_ok_ == false || traj_tracking_enabled_ == true)
        //while (error_norm_ > error_threshold || traj_tracking_enabled_ == true)
        while(error_within_bounds == false || traj_tracking_enabled_ == true)
        {
            if (traj_tracking_enabled_ == false)
            {
                //Enter new map into the "configurations" vector
                //configurations_.push_back(conf_);

            }

            //Compute jacobian for current configuration of body chain
            eigen_current_jac = getJacobian(kin_chain_,current_config_);

            if(collision_avoidance_active_ == true)
            {
                //Compute joint velocity bounds for robot body collision avoidance
                joint_vel_bounds = compute_risk_func_vector(ca_control_points,eigen_current_jac);
            }

            //JL Gradient computation (used in methods using Null Space Projection Method)
            //jl_grad_func_val = computeJLgradient(current_config_, q_min_, q_max_, jl_activation_param_);


            //Compute the Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
            current_jac_pinv = compute_J_vdls(eigen_current_jac, min_manip_treshold_, max_damping_factor_);


            //Compute the null space of the pseudoinvere
            //null_space_projector = compute_null_space_projection(eigen_current_jac,current_jac_pinv);


            //++++++ Compute joint velocity
            for (int i = 0; i < num_joints_ ; i++)
            {
                //Init q_dot_out
                q_dot_out_(i) = 0.0;

                //Compute first term of control law -> J * error
                //if(jl_ok_ == true)
                //{
                    for (int j = 0; j < dim_task_space_ ; j++)
                    {
                        if(collision_avoidance_active_ == true)
                            q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + ee_rep_vec[j] + error_gain_ * error_[j]);
                        else
                            q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + error_gain_ * error_[j]);

                    }
                //}

                //Compute term of control law -> (I - J'J) * jl_gradient
                //double q_dot_sec_task = 0.0;
                //for (int s = 0; s < num_joints_ ; s++)
                //   q_dot_sec_task = q_dot_sec_task + (null_space_projector[i][s] * jl_grad_func_val[s]);

                //cout<<q_dot_sec_task<<" "<<endl;

                //Add second term to first joint velocoty term
                //q_dot_out_(i) = q_dot_out_(i) + gain_sec_task_ * q_dot_sec_task;


                //Weight Joint velocity according to the PD strategy (joint weights are now considered in the jacobian pseudoinversion)
                //q_dot_out_(i) = joint_weights_(i) *  q_dot_out_(i);

                //Joint velocity/motion scaling for collision avoidance (for robot body parts / without endeffector)
                if(collision_avoidance_active_ == true)
                {
                    //cout<<"before:"<<q_dot_out_(i)<<endl;
                    q_dot_out_(i) = joint_vel_bounds[i][0] < q_dot_out_(i) ? joint_vel_bounds[i][0] : q_dot_out_(i);
                    //cout<<"after 1:"<<q_dot_out_(i)<<endl;
                    q_dot_out_(i) = q_dot_out_(i) < joint_vel_bounds[i][1]  ? joint_vel_bounds[i][1] : q_dot_out_(i);
                    //cout<<"after 2:"<<q_dot_out_(i)<<endl;
                }

            }
            //cout<<endl;


            //++++++ Update Joint Value
            //cout<<"New Configuration: "<<endl;
            //Current element index
            int curr_idx = 0;
            //Joint Limits check
            bool jl_respected = true;
            for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
            {
                if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
                {
                    //Store new joint value in temporary variable
                    double new_joint_val = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

                    //Check if joint is within joint limits
                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, true))
                    {
                        //Joint Limits are violated
                        jl_respected = false;

                        //Note: Keep fomer joint value, i.e. no update

                    }
                    else
                    {
                        //Joint Limits respected
                        //jl_respected = true;

                        //Assign new joint value to current joint value
                        current_config_(curr_idx) = new_joint_val;
                    }

                    //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

                    //Set Configuration to be shown by "show_ik_intermediate_config"-function
                    nvalues_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    if (kinematic_goup_ == "kuka_complete_arm" || kinematic_goup_ == "omnirob_lbr_sdh")
                    {
                        nvalues_["sdh2_finger_12_joint"] = -1.57;
                        nvalues_["sdh2_finger_22_joint"] = -1.57;
                        nvalues_["sdh2_thumb_2_joint"] = -1.57;

                    }

                    //Set map entry
                    //conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    curr_idx++;
                }
            }
            //When at least one of the joints violates the joint limits
            if(!jl_respected)
                 jl_ok_ = false;
             else
                 jl_ok_ = true;



            //Convert current configuration
            for (int k = 0 ; k < num_joints_ ; k++)
                robot_config[k] = current_config_(k);
            //Store current configuration
            joint_traj.push_back(robot_config);


            //Get new EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Store current ee pose
            ee_traj.push_back(current_ee_pose);

            //Store current endeffector pose
            //ee_poses_.push_back(current_ee_pose);

            //Store previous error
            vector<double> previous_error = error_;

            //++++++ ERROR UPDATE
            update_error_vec(current_ee_pose, current_goal_ee_pose_);

            //check whether constraint coordinates are within the permitted bounds
            error_within_bounds = is_error_within_bounds(error_);


            //Change rate of error
            double error_change_rate_sum = 0.0;
            for (int ei = 0 ; ei < dim_task_space_ ; ei++)
            {
                error_change_rate_sum += ( (error_[ei] - previous_error[ei]) * (error_[ei] - previous_error[ei]));
                //cout<<"Error change: "<<( (error_[ei] - previous_error[ei]) * (error_[ei] - previous_error[ei]))<<endl;
            }
            double error_change_rate_norm = sqrt(error_change_rate_sum);

            //cout<<"error change rate: "<<error_change_rate_norm<<endl;

            //Check if progress of endeffector towards goal is still significant (given the threshold "delta_error_threshold")
            //if (error_change_rate_norm < delta_error_threshold)
            //{
            //    //Decrement Deadlock Counter
            //    deadlock_timeout--;
            //}
            //else
            //{
            //    //Reset Deadlock Counter
            //    deadlock_timeout = 500;
            //}

            //When error reduction has been below threshold for
            //if(deadlock_timeout == 0)
            //{
            //    //controller_deadlock = true;
            //    cout<<"DEADLOCK"<<endl;
            //    cout<<"error_change_rate_norm: "<<error_change_rate_norm<<endl;
            //    cout<<"error_norm_: "<<error_norm_<<endl;

            //    //break;
            //}

            //++++++ Show Motion
            if(show_motion == 1)// || current_iter == max_iter-1 || error_norm_ < error_threshold)
                show_ik_intermediate_config(sleep_duration_between_confs_);


            if(collision_avoidance_active_ == true)
            {
                //----------------- START: CA TEST -------------------------

                //Compute Control Points (Position of Spheres located alon the endeffector chain, subsequently used for collision avoidance)
                ca_control_points = compute_CA_ControlPoints(joint_indices_for_CA_);


                //Show first control point setting
                if (show_motion)
                    RobotModel->show_CA_Control_Points(ca_control_points);


                //Compute Repulsive vector (for obstacle avoidance)
                ee_rep_vec = compute_EE_RepulsiveVector(ca_control_points[0]);

                //----------------- END: CA TEST -------------------------
            }


            //Show endeffector trace
            if(show_ee_trace == true)
                RobotModel->show_ee_trace(current_ee_pose,color_rgb_,max_ee_trace_points_);

            //Stop numerical IK iteration
            //if (error_norm_ < error_treshold_ && jl_ok_==true)
            //if (error_norm_ < error_treshold_)
            //      break;



            //++++++ Increment iteration counter
            current_iter++;

            //cout<<"Control iteration: "<<current_iter<<endl;


            //Check whether max_iterations has been reached
            if(current_iter == max_iter)
            {
                 //cout<<"Maximum controller iterations reached"<<endl;

//                cout<<"Final error: "<<endl;
//                cout<<"X: "<<error_[0]<<endl;
//                cout<<"Y: "<<error_[1]<<endl;
//                cout<<"Z: "<<error_[2]<<endl;
//                cout<<"rotX: "<<error_[3]<<endl;
//                cout<<"rotY: "<<error_[4]<<endl;
//                cout<<"rotZ: "<<error_[5]<<endl;

//                cout<<"Final error norm: "<<error_norm_<<endl<<endl;
                break;
            }

        } //end of ik loop

        //Switch to next endeffector pose of the trajectory
        current_goal_pose_index++;

    }


    //Delete endeffector trace (for next ee trajectory)
    RobotModel->delete_ee_trace();


    //If the maximum iterations has been reached without reducing the error norm below 'error_treshold_'
    if (current_iter == max_iter)
    {
        //cout<<"Controller trapped"<<endl;

        return ADVANCED;

    }
    else if (controller_deadlock == true)
    {
        //cout<<"Controller advanced"<<endl;

        return ADVANCED;

    }
    else
    {
        //cout<<"Node ee_pose reached"<<endl;

        return REACHED;    //error norm is smaller than 'error_treshold_'
    }

}




bool RobotController::run_J_wvdls_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data)
{

    //Eigen Matrices storing the Jacobian for the chain
    Eigen::MatrixXf eigen_current_jac(dim_task_space_,num_joints_);
    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_jac_pinv;
    //Value of the JL-Gradient function (to be used in the control loop)
    vector<double> jl_grad_func_val(num_joints_);
    //Value of the JL-Gradient function in previous iteration
    vector<double> previous_jl_grad_func_val(num_joints_);

    //Init iteration counter
    int current_iter = 0;

    //Variables for variable damping with DLS (based on manipulability analysis)
    min_manip_treshold_ = 0.09;

    //Reset vectors storing configurations and ee poses
    configurations_.clear(); //Clear vector storing configurations
    ee_poses_.clear(); //Clear vector storing ee poses


    //Init Joint Limit Gradient (to be maximized in the methods using null space projection of secondary task)
    for (int s = 0; s < num_joints_ ; s++)
        jl_grad_func_val[s] = (((q_max_[s]- q_min_[s]) * (q_max_[s]- q_min_[s])) *  (2 * current_config_(s) - q_max_[s] - q_min_[s]))   /  (4 *  ((q_max_[s]-current_config_(s))*(q_max_[s]-current_config_(s))) * ((current_config_(s)-q_min_[s])*(current_config_(s)-q_min_[s])) );


    //Init goal pose index
    int current_goal_pose_index = 0;

    while (current_goal_pose_index < num_ee_poses_)
    {
        //Reset iteration counter
        current_iter = 0;

        if (traj_tracking_enabled_ == true)
        {
            //Set current goal pose
            //cout<<"Current ee goal pose num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_pose;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_pose.push_back(ee_ref_pos_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_pos_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_pose_ = curr_goal_pose;

            //Get EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Update the error
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);

            //Set current goal vel
            //cout<<"Current ee goal vel num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_vel;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_vel.push_back(ee_ref_vel_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_vel_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_vel_ = curr_goal_vel;

            //Enter new map into the "configurations" vector
            configurations_.push_back(conf_);
        }


        //------Numerical inverse kinematic loop --------
        //while (error_norm_ > error_treshold_ || jl_ok_ == false || traj_tracking_enabled_ == true)
        while (error_norm_ > error_treshold_ || traj_tracking_enabled_ == true)
        {
            if (traj_tracking_enabled_ == false)
            {
                //Enter new map into the "configurations" vector
                configurations_.push_back(conf_);
            }

            //Compute jacobian for current configuration of body chain
            eigen_current_jac = getJacobian(kin_chain_,current_config_);

            //Store previous gradient value
             previous_jl_grad_func_val = jl_grad_func_val;

            //Compute value of Joint Limits Gradient function (to be maximized in the null space of the jacobian)
            for (int s = 0; s < num_joints_ ; s++)
                 jl_grad_func_val[s] = (((q_max_[s]- q_min_[s]) * (q_max_[s]- q_min_[s])) *  (2 * current_config_(s) - q_max_[s] - q_min_[s]))   /  (4 *  ((q_max_[s]-current_config_(s))*(q_max_[s]-current_config_(s))) * ((current_config_(s)-q_min_[s])*(current_config_(s)-q_min_[s])) );



            //Compute Weighted Damped-least squares Pseudoinverse with variable damping factor (considers joint limits)
            current_jac_pinv =  compute_J_wvdls(eigen_current_jac, min_manip_treshold_, max_damping_factor_, previous_jl_grad_func_val, jl_grad_func_val);
            //Note: Null Space Projection is not required here because joint limts are avoided using the Weighting matrix



            //++++++ Compute joint velocity
            for (int i = 0; i < num_joints_ ; i++)
            {
                //Init q_dot_out
                q_dot_out_(i) = 0.0;

                //Compute first term of control law -> J * error
                for (int j = 0; j < dim_task_space_ ; j++)
                      q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + error_gain_ * error_[j]);


                 //Weight Joint velocity according to the PD strategy (joint weights are now considered in the jacobian pseudoinversion)
                 q_dot_out_(i) = joint_weights_(i) *  q_dot_out_(i);

            }


            //++++++ Update Joint Value
            //cout<<"New Configuration: "<<endl;
            //Current element index
            int curr_idx = 0;
            //Joint Limits check
            bool jl_respected = true;
            for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
            {
                if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
                {
                    //Store new joint value in temporary variable
                    double new_joint_val = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

                    //Check if joint is within joint limits
                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, false))
                    {
                        //Joint Limits are violated
                        jl_respected = false;

//                        //Lower joint value bound violated
//                        if (new_joint_val < q_min_[curr_idx])
//                            current_config_(curr_idx) = q_min_[curr_idx] + 0.1; //0.05 rad = 3 degree distance to joint range border
//                        //Upper joint value bound violated
//                        if (q_max_[curr_idx] < new_joint_val)
//                            current_config_(curr_idx) = q_max_[curr_idx] - 0.1; //0.05 rad = 3 degree distance to joint range border
                    }
                    else
                    {
                        //Joint Limits respected
                        jl_respected = true;

                        //Assign new joint value to current joint value
                        current_config_(curr_idx) = new_joint_val;
                    }

                     //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

                    //Set Configuration to be shown by "show_ik_intermediate_config"-function
                    nvalues_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    //Set map entry
                    conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    curr_idx++;
                }
            }
//            //When at least one of the joints violates the joint limits
//            if(!jl_respected)
//                 jl_ok_ = false;
//             else
//                 jl_ok_ = true;



//            //++++++ Check joint limits
//            int config_element = 0;
//            bool jl_respected = true;
//            for (unsigned int i = 0; i != kin_chain_.getNrOfSegments() ; i++)
//            {
//                if (kin_chain_.getSegment(i).getJoint().getTypeName() != "None")
//                {
//                    //Check if joint is within joint limits
//                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(i).getJoint().getName(),current_config_(config_element)))
//                        jl_respected = false;

//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" has value: "<<current_config(config_element)<<endl;
//                    nvalues_[kin_chain_.getSegment(i).getJoint().getName()] = current_config_(config_element);

//                    //Increment config vector element
//                    config_element++;
//                }
//                else
//                {
//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" is a fixed joint"<<endl;
//                }
//            }
//            //When at least one of the joints violates the joint limits
//            if(!jl_respected)
//                 jl_ok_ = false;
//             else
//                 jl_ok_ = true;


            //Get new EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Store current endeffector pose
            ee_poses_.push_back(current_ee_pose);


            //++++++ ERROR UPDATE
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);


            //++++++ Show Motion
            if(show_motion == 1 || current_iter == max_iter-1 || error_norm_ < error_treshold_)
                show_ik_intermediate_config(sleep_duration_between_confs_);


            //Show endeffector trace
            if(show_ee_trace == true)
                RobotModel->show_ee_trace(current_ee_pose,color_rgb_,max_ee_trace_points_);

            //Stop numerical IK iteration
            if (error_norm_ < error_treshold_ && jl_ok_==true)
                  break;

            //++++++ Increment iteration counter
            current_iter++;
            //Check whether max_iterations has been reached
            if(current_iter == max_iter)
            {
                cout<<"Maximum iterations reached"<<endl;

                cout<<"Final error: "<<endl;
                cout<<"X: "<<error_[0]<<endl;
                cout<<"Y: "<<error_[1]<<endl;
                cout<<"Z: "<<error_[2]<<endl;
                cout<<"rotX: "<<error_[3]<<endl;
                cout<<"rotY: "<<error_[4]<<endl;
                cout<<"rotZ: "<<error_[5]<<endl;

                cout<<"Final error norm: "<<error_norm_<<endl<<endl;
                break;
            }

            //Only perform one ik iteration is trajectory tracking is enabled
            //if (traj_tracking_enabled_ == true)
            //    break;

        } //end of ik loop

        //Switch to next endeffector pose of the trajectory
        current_goal_pose_index++;

    }



    //Delete endeffector trace (for next ee trajectory)
    RobotModel->delete_ee_trace();

    if(store_traj_data == true)
    {
        //Write joint trajectory, i.e. the sequence of configurations to a file
        RobotModel->writeJointTrajectoryToFile(configurations_, w_joints_, file_path_joint_trajectory_);

        //Write endeffector trajectory to a file
        RobotModel->writeEndeffectorTrajectoryToFile(ee_poses_, file_path_endeffector_trajectory_);
    }


    //If the maximum iterations has been reached without reducing the error norm below 'error_treshold_'
    if (current_iter == max_iter)
        return false;
    else
    {
        cout<<"Goal pose reached"<<endl;

        cout<<"Final error: "<<endl;
        cout<<"X: "<<error_[0]<<endl;
        cout<<"Y: "<<error_[1]<<endl;
        cout<<"Z: "<<error_[2]<<endl;
        cout<<"rotX: "<<error_[3]<<endl;
        cout<<"rotY: "<<error_[4]<<endl;
        cout<<"rotZ: "<<error_[5]<<endl;

        cout<<"Final error norm: "<<error_norm_<<endl<<endl;
        return true;    //error norm is smaller than 'error_treshold_'
    }

}




bool RobotController::run_J_wln_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data)
{

    //Reset Joint Velocity Integration
    //delta_t_ = 0.0008;

    //Eigen Matrices storing the Jacobian for the chain
    Eigen::MatrixXf eigen_current_jac(dim_task_space_,num_joints_);
    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_jac_pinv;
    //Value of the JL-Gradient function (to be used in the control loop)
    vector<double> jl_grad_func_val(num_joints_);
    //Value of the JL-Gradient function in previous iteration
    vector<double> previous_jl_grad_func_val(num_joints_);

    //Init iteration counter
    int current_iter = 0;

    //Reset vectors storing configurations and ee poses
    configurations_.clear(); //Clear vector storing configurations
    ee_poses_.clear(); //Clear vector storing ee poses


    //Init Joint Limit Gradient (to be maximized in the methods using null space projection of secondary task)
    for (int s = 0; s < num_joints_ ; s++)
        jl_grad_func_val[s] = (((q_max_[s]- q_min_[s]) * (q_max_[s]- q_min_[s])) *  (2 * current_config_(s) - q_max_[s] - q_min_[s]))   /  (4 *  ((q_max_[s]-current_config_(s))*(q_max_[s]-current_config_(s))) * ((current_config_(s)-q_min_[s])*(current_config_(s)-q_min_[s])) );


    //Init goal pose index
    int current_goal_pose_index = 0;

    while (current_goal_pose_index < num_ee_poses_)
    {
        //Reset iteration counter
        current_iter = 0;

        if (traj_tracking_enabled_ == true)
        {
            //Set current goal pose
            //cout<<"Current ee goal pose num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_pose;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_pose.push_back(ee_ref_pos_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_pos_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_pose_ = curr_goal_pose;

            //Get EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Update the error
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);

            //Set current goal vel
            //cout<<"Current ee goal vel num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_vel;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_vel.push_back(ee_ref_vel_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_vel_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_vel_ = curr_goal_vel;

            //Enter new map into the "configurations" vector
            configurations_.push_back(conf_);
        }


        //------Numerical inverse kinematic loop --------
        //while (error_norm_ > error_treshold_ || jl_ok_ == false || traj_tracking_enabled_ == true)
        while (error_norm_ > error_treshold_ || traj_tracking_enabled_ == true)
        {
            if (traj_tracking_enabled_ == false)
            {
                //Enter new map into the "configurations" vector
                configurations_.push_back(conf_);
            }

            //Compute jacobian for current configuration of body chain
            eigen_current_jac = getJacobian(kin_chain_,current_config_);

            //Store previous gradient value
             previous_jl_grad_func_val = jl_grad_func_val;

            //Compute value of Joint Limits Gradient function (to be maximized in the null space of the jacobian)
            //cout<<"Current joint limit gradient:"<<endl;
            for (int s = 0; s < num_joints_ ; s++)
            {
//                 cout<<"Current joint val:"<<current_config_(s)<<endl;
//                 cout<<"Current joint max:"<<q_max_[s]<<endl;
//                 cout<<"Current joint min:"<<q_min_[s]<<endl;

                 jl_grad_func_val[s] = ( ( (q_max_[s]- q_min_[s]) * (q_max_[s]- q_min_[s]) ) *  (2 * current_config_(s) - q_max_[s] - q_min_[s]) )   /  (4 *  ( ( q_max_[s]-current_config_(s) ) * ( q_max_[s]-current_config_(s) ) ) * ( ( current_config_(s)-q_min_[s] ) * ( current_config_(s)-q_min_[s] ) ) );
                 //cout<<"Current joint gradient:"<<jl_grad_func_val[s]<<endl;;
            }
            //cout<<endl;


//            for (double s = -1.5; s <= 1.5 ; s+=0.1)
//            {
//                cout<<s<<" ";
//            }
//            cout<<endl;
//            for (double s = -1.5; s <= 1.5 ; s+=0.1)
//            {
//                 double test = ( ( (1.00 + 1.00) * (1.00 + 1.00) ) *  (2 * s - 1.00 + 1.00) )   /  (4 *  ( ( 1.00-s ) * ( 1.00-s ) ) * ( ( s+ 1.00 ) * ( s+ 1.00 ) ) );
//                cout<<test<<"  ";
//            }
//            cout<<endl;

            //3)Weighted Least-Norm Pseudoinverse (avoids joint limits, but unstable in the neighbourhood of singularities)
            current_jac_pinv = compute_J_wln(eigen_current_jac, previous_jl_grad_func_val, jl_grad_func_val);
            //Note: Null Space Projection is not required here because joint limts are avoided using the Weighting matrix



            //++++++ Compute joint velocity
            for (int i = 0; i < num_joints_ ; i++)
            {
                //Init q_dot_out
                q_dot_out_(i) = 0.0;

                //Compute first term of control law -> J * error
                for (int j = 0; j < dim_task_space_ ; j++)
                      q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + error_gain_ * error_[j]);


                 //Weight Joint velocity according to the PD strategy (joint weights are now considered in the jacobian pseudoinversion)
                 q_dot_out_(i) = joint_weights_(i) *  q_dot_out_(i);

                 //cout<<q_dot_out_(i)<<" ";

            }
            //cout<<endl;


            //++++++ Update Joint Value
            //cout<<"New Configuration: "<<endl;
            //Current element index
            int curr_idx = 0;
            //Joint Limits check
            bool jl_respected = true;
            for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
            {
                if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
                {
                    //Store new joint value in temporary variable
                    double new_joint_val = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

                    //Check if joint is within joint limits
                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, false))
                    {
                        //Joint Limits are violated
                        jl_respected = false;

                        //Lower joint value bound violated
                        if (new_joint_val < q_min_[curr_idx])
                            current_config_(curr_idx) = q_min_[curr_idx] + 0.05; //0.05 rad = 3 degree distance to joint range border
                        //Upper joint value bound violated
                        if (q_max_[curr_idx] < new_joint_val)
                            current_config_(curr_idx) = q_max_[curr_idx] - 0.05; //0.05 rad = 3 degree distance to joint range border
                    }
                    else
                    {
                        //Joint Limits respected
                        jl_respected = true;

                        //Assign new joint value to current joint value
                        current_config_(curr_idx) = new_joint_val;
                    }

                     //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

                    //Set Configuration to be shown by "show_ik_intermediate_config"-function
                    nvalues_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    //Set map entry
                    conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    curr_idx++;
                }
            }

//            //++++++ Update Joint Value
//            //cout<<"New Configuration: "<<endl;
//            //Current element index
//            int curr_idx = 0;
//            for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
//            {
//                if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
//                {
//                    //Update joint value
//                    current_config_(curr_idx) = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

//                    //Print new joint value
//                    //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

//                    //Set map entry
//                    conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

//                    curr_idx++;
//                }
//            }


//            //++++++ Check joint limits
//            int config_element = 0;
//            bool jl_respected = true;
//            for (unsigned int i = 0; i != kin_chain_.getNrOfSegments() ; i++)
//            {
//                if (kin_chain_.getSegment(i).getJoint().getTypeName() != "None")
//                {
//                    //Check if joint is within joint limits
//                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(i).getJoint().getName(),current_config_(config_element),true))
//                        jl_respected = false;

//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" has value: "<<current_config(config_element)<<endl;
//                    nvalues_[kin_chain_.getSegment(i).getJoint().getName()] = current_config_(config_element);

//                    //Increment config vector element
//                    config_element++;
//                }
//                else
//                {
//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" is a fixed joint"<<endl;
//                }
//            }
//            //When at least one of the joints violates the joint limits
//            if(!jl_respected)
//                 jl_ok_ = false;
//             else
//                 jl_ok_ = true;


            //Get new EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

//            cout<<"Current endeffector pose: "<<endl;
//            cout<<"X: "<<current_ee_pose[0]<<endl;
//            cout<<"Y: "<<current_ee_pose[1]<<endl;
//            cout<<"Z: "<<current_ee_pose[2]<<endl;
//            cout<<"rotX: "<<current_ee_pose[3]<<endl;
//            cout<<"rotY: "<<current_ee_pose[4]<<endl;
//            cout<<"rotZ: "<<current_ee_pose[5]<<endl;

            //Store current endeffector pose
            ee_poses_.push_back(current_ee_pose);


            //++++++ ERROR UPDATE
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);


            //++++++ Show Motion
            if(show_motion == 1 || current_iter == max_iter-1 || error_norm_ < error_treshold_)
                show_ik_intermediate_config(sleep_duration_between_confs_);


            //Show endeffector trace
            if(show_ee_trace == true)
                RobotModel->show_ee_trace(current_ee_pose,color_rgb_,max_ee_trace_points_);

            //Stop numerical IK iteration
            if (error_norm_ < error_treshold_ && jl_ok_==true)
                  break;


            //++++++ Increment iteration counter
            current_iter++;
            //Check whether max_iterations has been reached
            if(current_iter == max_iter)
            {
                cout<<"Maximum iterations reached"<<endl;

                cout<<"Final error: "<<endl;
                cout<<"X: "<<error_[0]<<endl;
                cout<<"Y: "<<error_[1]<<endl;
                cout<<"Z: "<<error_[2]<<endl;
                cout<<"rotX: "<<error_[3]<<endl;
                cout<<"rotY: "<<error_[4]<<endl;
                cout<<"rotZ: "<<error_[5]<<endl;

                cout<<"Final error norm: "<<error_norm_<<endl<<endl;
                break;
            }

            //Only perform one ik iteration is trajectory tracking is enabled
            //if (traj_tracking_enabled_ == true)
            //    break;

        } //end of ik loop

        //Switch to next endeffector pose of the trajectory
        current_goal_pose_index++;

    }


    //Delete endeffector trace (for next ee trajectory)
    RobotModel->delete_ee_trace();

    if(store_traj_data == true)
    {
        //Write joint trajectory, i.e. the sequence of configurations to a file
        RobotModel->writeJointTrajectoryToFile(configurations_, w_joints_, file_path_joint_trajectory_);

        //Write endeffector trajectory to a file
        RobotModel->writeEndeffectorTrajectoryToFile(ee_poses_, file_path_endeffector_trajectory_);

    }

    //If the maximum iterations has been reached without reducing the error norm below 'error_treshold_'
    if (current_iter == max_iter)
        return false;
    else
    {
        cout<<"Goal pose reached"<<endl;

        cout<<"Final error: "<<endl;
        cout<<"X: "<<error_[0]<<endl;
        cout<<"Y: "<<error_[1]<<endl;
        cout<<"Z: "<<error_[2]<<endl;
        cout<<"rotX: "<<error_[3]<<endl;
        cout<<"rotY: "<<error_[4]<<endl;
        cout<<"rotZ: "<<error_[5]<<endl;

        cout<<"Final error norm: "<<error_norm_<<endl<<endl;
        return true;    //error norm is smaller than 'error_treshold_'
    }
}



//Jacobian Transpose (no joint limit avoidance, unstable in the neighbourhood of singularities)
bool RobotController::run_J_trans_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data)
{

    //Eigen Matrices storing the Jacobian for the chain
    Eigen::MatrixXf eigen_current_jac(dim_task_space_,num_joints_);
    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_jac_pinv;

    //Init iteration counter
    int current_iter = 0;

    //Time step is adapted within compute_J_trans (thus delta_t unused for jac transpose method)
    delta_t_ = 1.0;

    //Init goal pose index
    int current_goal_pose_index = 0;

    //Reset vectors storing configurations and ee poses
    configurations_.clear(); //Clear vector storing configurations
    ee_poses_.clear(); //Clear vector storing ee poses

    while (current_goal_pose_index < num_ee_poses_)
    {
        //Reset iteration counter
        current_iter = 0;

        if (traj_tracking_enabled_ == true)
        {
            //Set current goal pose
            //cout<<"Current ee goal pose num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_pose;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_pose.push_back(ee_ref_pos_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_pos_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_pose_ = curr_goal_pose;

            //Get EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Update the error
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);


            //Set current goal vel
            //cout<<"Current ee goal vel num: "<<current_goal_pose_index<<endl;
            vector <double> curr_goal_vel;
            for (int i = 0 ; i <= dim_task_space_ ; i++)
            {
                curr_goal_vel.push_back(ee_ref_vel_trajectory_[current_goal_pose_index][i]);
                //cout<<ee_ref_vel_trajectory_[current_goal_pose_index][i]<<" ";
            }
            //cout<<endl;


            //Set the endeffector cartesian goal pose
            current_goal_ee_vel_ = curr_goal_vel;

            //Enter new map into the "configurations" vector
            configurations_.push_back(conf_);
        }


        //------Numerical inverse kinematic loop --------
        //while (error_norm_ > error_treshold_ || jl_ok_ == false || traj_tracking_enabled_ == true)
        while (error_norm_ > error_treshold_ || traj_tracking_enabled_ == true)
        {
            if (traj_tracking_enabled_ == false)
            {
                //Enter new map into the "configurations" vector
                configurations_.push_back(conf_);
            }

            //Compute jacobian for current configuration of body chain
            eigen_current_jac = getJacobian(kin_chain_,current_config_);


            //4)Jacobian Transpose (no joint limit avoidance, unstable in the neighbourhood of singularities)
            current_jac_pinv = compute_J_trans(eigen_current_jac, error_);


            //++++++ Compute joint velocity
            for (int i = 0; i < num_joints_ ; i++)
            {
                //Init q_dot_out
                q_dot_out_(i) = 0.0;

                //Compute first term of control law -> J * error
                for (int j = 0; j < dim_task_space_ ; j++)
                      q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + error_gain_ * error_[j]);


                 //Weight Joint velocity according to the PD strategy (joint weights are now considered in the jacobian pseudoinversion)
                 q_dot_out_(i) = joint_weights_(i) *  q_dot_out_(i);

            }


            //++++++ Update Joint Value
            //cout<<"New Configuration: "<<endl;
            //Current element index
            int curr_idx = 0;
            //Joint Limits check
            bool jl_respected = true;
            for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
            {
                if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
                {
                    //Store new joint value in temporary variable
                    double new_joint_val = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

                    //Check if joint is within joint limits
                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, true))
                    {
                        //Joint Limits are violated
                        jl_respected = false;

                        //Note: Keep fomer joint value, i.e. no update

//                        //Lower joint value bound violated
//                        if (new_joint_val < q_min_[curr_idx])
//                            current_config_(curr_idx) = q_min_[curr_idx] + 0.05; //0.05 rad = 3 degree distance to joint range border
//                        //Upper joint value bound violated
//                        if (q_max_[curr_idx] < new_joint_val)
//                            current_config_(curr_idx) = q_max_[curr_idx] - 0.05; //0.05 rad = 3 degree distance to joint range border
                    }
                    else
                    {
                        //Joint Limits respected
                        jl_respected = true;

                        //Assign new joint value to current joint value
                        current_config_(curr_idx) = new_joint_val;
                    }

                     //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

                    //Set Configuration to be shown by "show_ik_intermediate_config"-function
                    nvalues_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    //Set map entry
                    conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

                    curr_idx++;
                }
            }
//            //When at least one of the joints violates the joint limits
//            if(!jl_respected)
//                 jl_ok_ = false;
//             else
//                 jl_ok_ = true;


//            //++++++ Check joint limits
//            int config_element = 0;
//            bool jl_respected = true;
//            for (unsigned int i = 0; i != kin_chain_.getNrOfSegments() ; i++)
//            {
//                if (kin_chain_.getSegment(i).getJoint().getTypeName() != "None")
//                {
//                    //Check if joint is within joint limits
//                    if(!RobotModel->checkJointLimits(kin_chain_.getSegment(i).getJoint().getName(),current_config_(config_element)))
//                        jl_respected = false;

//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" has value: "<<current_config(config_element)<<endl;
//                    nvalues_[kin_chain_.getSegment(i).getJoint().getName()] = current_config_(config_element);

//                    //Increment config vector element
//                    config_element++;
//                }
//                else
//                {
//                    //cout<<"Joint "<<arm_chain.getSegment(i).getJoint().getName()<<" is a fixed joint"<<endl;
//                }
//            }
//            //When at least one of the joints violates the joint limits
//            if(!jl_respected)
//                 jl_ok_ = false;
//             else
//                 jl_ok_ = true;


            //Get new EE pose
            vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

            //Store current endeffector pose
            ee_poses_.push_back(current_ee_pose);


            //++++++ ERROR UPDATE
            update_error_norm(current_ee_pose, current_goal_ee_pose_, true);


            //++++++ Show Motion
            if(show_motion == 1 || current_iter == max_iter-1 || error_norm_ < error_treshold_)
                show_ik_intermediate_config(sleep_duration_between_confs_);


            //Show endeffector trace
            if(show_ee_trace == true)
                RobotModel->show_ee_trace(current_ee_pose,color_rgb_,max_ee_trace_points_);

            //Stop numerical IK iteration
            if (error_norm_ < error_treshold_ && jl_ok_==true)
                  break;


            //++++++ Increment iteration counter
            current_iter++;
            //Check whether max_iterations has been reached
            if(current_iter == max_iter)
            {
                cout<<"Maximum iterations reached"<<endl;

                cout<<"Final error: "<<endl;
                cout<<"X: "<<error_[0]<<endl;
                cout<<"Y: "<<error_[1]<<endl;
                cout<<"Z: "<<error_[2]<<endl;
                cout<<"rotX: "<<error_[3]<<endl;
                cout<<"rotY: "<<error_[4]<<endl;
                cout<<"rotZ: "<<error_[5]<<endl;

                cout<<"Final error norm: "<<error_norm_<<endl<<endl;
                break;
            }

            //Only perform one ik iteration is trajectory tracking is enabled
            //if (traj_tracking_enabled_ == true)
            //    break;

        } //end of ik loop

        //Switch to next endeffector pose of the trajectory
        current_goal_pose_index++;

    }


    //Delete endeffector trace (for next ee trajectory)
    RobotModel->delete_ee_trace();

    if(store_traj_data == true)
    {
        //Write joint trajectory, i.e. the sequence of configurations to a file
        RobotModel->writeJointTrajectoryToFile(configurations_, w_joints_, file_path_joint_trajectory_);

        //Write endeffector trajectory to a file
        RobotModel->writeEndeffectorTrajectoryToFile(ee_poses_, file_path_endeffector_trajectory_);

    }

    //If the maximum iterations has been reached without reducing the error norm below 'error_treshold_'
    if (current_iter == max_iter)
        return false;
    else
    {
        cout<<"Goal pose reached"<<endl;

        cout<<"Final error: "<<endl;
        cout<<"X: "<<error_[0]<<endl;
        cout<<"Y: "<<error_[1]<<endl;
        cout<<"Z: "<<error_[2]<<endl;
        cout<<"rotX: "<<error_[3]<<endl;
        cout<<"rotY: "<<error_[4]<<endl;
        cout<<"rotZ: "<<error_[5]<<endl;

        cout<<"Final error norm: "<<error_norm_<<endl<<endl;
        return true;    //error norm is smaller than 'error_treshold_'
    }

}


bool RobotController::learn_joint_trajectory(int control_strategy, int max_ik_iter, int max_learning_iter, double mean_joint_traj_error_threshold, int show_motion, bool show_ee_trace, bool store_traj_data)
{
    //Init joint weights (all weights equal to 1.0)
    //set_motion_strategy(0);

    //Read joint trajectory (of the human subject and motion segment)
    human_joint_trajectory_ = readJointTrajectory(file_path_start_config_);

    //Start Joint (to be optimized first)
    int joint_num = 0;

    while (max_learning_iter != 0)
    {
        //Reinitialize control setup
        init(file_path_start_config_,file_path_ee_pos_trajectory_,file_path_ee_vel_trajectory_);

        switch (control_strategy)
        {
            case 0: run_J_pinv_Controller(max_ik_iter,show_motion,show_ee_trace,store_traj_data); //Run controller with current joint weights
                break;
            case 1: run_J_dls_Controller(max_ik_iter,show_motion,show_ee_trace,store_traj_data);
                break;
            case 2: run_J_vdls_Controller(max_ik_iter,show_motion,show_ee_trace,store_traj_data);
                break;
            case 3: run_J_wvdls_Controller(max_ik_iter,show_motion,show_ee_trace,store_traj_data);
                break;
            case 4: run_J_wln_Controller(max_ik_iter,show_motion,show_ee_trace,store_traj_data);
                break;
            case 5: run_J_trans_Controller(max_ik_iter,show_motion,show_ee_trace,store_traj_data);
                break;
            default: run_J_pinv_Controller(max_ik_iter,show_motion,show_ee_trace,store_traj_data); //Run controller with current joint weights
                break;
        }




        cout<<"Optimize joint num: "<<joint_num<<endl;

        //Update Joint weights based on error between human and control joint trajectory (second argument = mean_joint_traj_error_threshold)
        bool traj_sufficient_close = update_joint_weights(joint_num, mean_joint_traj_error_threshold);

        if (traj_sufficient_close == true)
        {
            //Optimize next joint
            joint_num++;

            //Reset joint update rates
            //max_joint_weight_modification_ = 0.5;

            //Reset previous mean joint trajectory error
            previous_mean_joint_trajectory_error_ = 1000.0;
        }

        //Start next optimization iteration
        if (joint_num == num_joints_)
        {
            joint_num = 0;
            //break;

           //Decrease learning iteration counter
            max_learning_iter--;
        }

    } //End of joint weight optimizations

    //Write the final/learned joint weights to file
    writeJointWeights(joint_weights_,file_path_joint_weights_);

    return true;


}


bool RobotController::update_joint_weights(int j_num, double mean_joint_traj_error_threshold)
{

    //Read joint trajectory generated by the controller
    map<string, vector<double> > control_joint_trajectory = readJointTrajectory(file_path_joint_trajectory_);

//    //----------------------------- Start: Plot human and control joint trajectory ------------------------
//    std::vector<std::pair<double, double> > human_joint_traj;
//    for(double t = 0 ; t < num_configs_; t+=1.0)
//    {
//        human_joint_traj.push_back(std::make_pair( t , human_joint_trajectory_[joint_names_[j_num]][t] ));
//    }

//    std::vector<std::pair<double, double> > control_joint_traj;
//    for(double t = 0 ; t < num_configs_; t+=1.0)
//    {
//        control_joint_traj.push_back(std::make_pair( t , control_joint_trajectory[joint_names_[j_num]][t] ));
//    }


//    //Find max and min values for y axis (that is max/min value in human joint trajectory)
//    double y_min = 1000.0, y_max = -1000.0;
//    for(int conf = 0 ; conf < num_configs_; conf++)
//    {
//        if (human_joint_trajectory_[joint_names_[j_num]][conf] < y_min)
//            y_min = human_joint_trajectory_[joint_names_[j_num]][conf];

//        //if (control_joint_trajectory[joint_names_[j_num]][conf] < y_min)
//        //  y_min = control_joint_trajectory[joint_names_[j_num]][conf];

//        if (human_joint_trajectory_[joint_names_[j_num]][conf] > y_max)
//            y_max = human_joint_trajectory_[joint_names_[j_num]][conf];

//        //if (control_joint_trajectory[joint_names_[j_num]][conf] > y_max)
//        //  y_max = control_joint_trajectory[joint_names_[j_num]][conf];
//    }

//    //cout<<"Y min: "<<y_min<<"   "<<"Y max: "<<y_max<<"   "<<endl;

//    //Convert y_min and y_max (= y plot range) to a string
//    stringstream read_min;
//    read_min << y_min;
//    string y_min_string = read_min.str();
//    stringstream read_max;
//    read_max << y_max;
//    string y_max_string = read_max.str();

//    //Convert number of configurations (= x plot range) to a string
//    stringstream read_num_confs;
//    read_num_confs << num_configs_;
//    string num_configs_string = read_num_confs.str();

//    //Set X axis range
//    gnu_plot_ << "set xrange [0:" + num_configs_string + "]\nset yrange [" + y_min_string + ":" + y_max_string +"]\n";

//    // Data will be sent via a temporary file.  These are erased when you call
//    // gp.clearTmpfiles() or when gp goes out of scope.  If you pass a filename
//    // (e.g. "gp.file1d(pts, 'mydata.dat')"), then the named file will be created
//    // and won't be deleted (this is useful when creating a script).
//    gnu_plot_ << "plot" << gnu_plot_.file1d(human_joint_traj) << "with lines title 'human joint " +joint_names_[j_num]+ " trajectory',"<< gnu_plot_.file1d(control_joint_traj) << "with lines title 'control joint " +joint_names_[j_num]+ " trajectory'" << std::endl;

//    //--------------------------- End: Plot human and control joint trajectory ------------------------




    double human_to_control_val_error_sign = 0.0;

    double agg_total_joint_trajectory_error = 0.0;
    double mean_joint_trajectory_error = 0.0;



    //Compare joint trajectory of human and controller
    for (int conf_num = 1; conf_num < num_configs_ ; conf_num++)
    {
        //Sign of error (required to determine joint weight update direction)
        human_to_control_val_error_sign = human_joint_trajectory_[joint_names_[j_num]][conf_num] - control_joint_trajectory[joint_names_[j_num]][conf_num];


        agg_total_joint_trajectory_error =  agg_total_joint_trajectory_error + fabs(human_to_control_val_error_sign);
    }

    //Determine mean trajectory error using the corrent joint weight
    mean_joint_trajectory_error = agg_total_joint_trajectory_error / double(num_configs_);
    cout<<"Mean trajectory error for joint "<<joint_names_[j_num]<<" is: "<<mean_joint_trajectory_error<<endl;

    cout<<"Previous mean trajectory error for joint "<<joint_names_[j_num]<<" is: "<<previous_mean_joint_trajectory_error_<<endl;


    //Stop optimizing joint weight when a lower threshold of mean joint trajectory error is met
    if (mean_joint_trajectory_error < mean_joint_traj_error_threshold || fabs(mean_joint_trajectory_error - previous_mean_joint_trajectory_error_) < 0.001)
    {
        //Store last mean trajectory error
        previous_mean_joint_trajectory_error_ = mean_joint_trajectory_error;

        cout<<"Final weight for joint "<<joint_names_[j_num]<<" is: "<<joint_weights_(j_num)<<endl;
        return true;
    }
    else
    {

        //Mean trajectory error has increased after last joint weight update
        if (previous_mean_joint_trajectory_error_ < mean_joint_trajectory_error)
        {
            //If joint weight has been modified in the positive direction in the previous iteration
            if (joint_weight_gradient_direction_ == 0)
            {
                max_joint_weight_modification_[j_num] = 0.8 * max_joint_weight_modification_[j_num];
                joint_weight_gradient_direction_ = 1;
            }
            //If joint weight has been modified in the negative direction in the previous iteration
            else if (joint_weight_gradient_direction_ == 1)
            {
                max_joint_weight_modification_[j_num] = 0.8 * max_joint_weight_modification_[j_num];
                joint_weight_gradient_direction_ = 0;
            }
            else
            {}
        }

        //Joint Weight Update
        if (joint_weight_gradient_direction_ == 0)
        {
             joint_weights_(j_num) = joint_weights_(j_num) + max_joint_weight_modification_[j_num];

             cout<<"New increased weight for joint "<<joint_names_[j_num]<<" is: "<<joint_weights_(j_num)<<endl;

             //Set previous joint trajectory error to the current joint trajectory error
             previous_mean_joint_trajectory_error_ = mean_joint_trajectory_error;

             return false;
        }
        if (joint_weight_gradient_direction_ == 1)
        {
            if (max_joint_weight_modification_[j_num] < joint_weights_(j_num))
                joint_weights_(j_num) = joint_weights_(j_num) - max_joint_weight_modification_[j_num];
            else
                joint_weights_(j_num) = joint_weights_(j_num) / 2.0;

             cout<<"New decreased weight for joint "<<joint_names_[j_num]<<" is: "<<joint_weights_(j_num)<<endl;

             //Set previous joint trajectory error to the current joint trajectory error
             previous_mean_joint_trajectory_error_ = mean_joint_trajectory_error;
             return false;
        }
    } //end of else

}



//Write the final/learned joint weights to file
void RobotController::writeJointWeights(KDL::JntArray joint_weights, char* file_path)
{
    //-------------------- Write solution path into file
    //Remove the current solution file
    if( std::remove( file_path ) != 0 )
    {
        std::cout<< "Error deleting file (writeJointWeights)" <<std::endl;
    }

    //File for endeffector trajectory
    std::ofstream joint_weights_path;
    //Open a file
    joint_weights_path.open(file_path); // opens the file
    if( !joint_weights_path)
      { // file couldn't be opened
            std::cerr << "Error: endeffector trajectory file could not be opened" << std::endl;
            exit(1);
      }

    //Write Header
    for (unsigned int i = 0 ; i < joint_weights.rows() ; i++)
    {
        joint_weights_path << joint_names_[i]<<" ";
    }

    //Set cursor to next line (for next configuration)
    joint_weights_path << std::endl;

    //Write the endeffector trajectory into the file
    for (unsigned int i = 0 ; i < joint_weights.rows() ; i++)
    {
        joint_weights_path << joint_weights(i) <<" ";
    }

    //Set cursor to next line (for next configuration)
    joint_weights_path << std::endl;

    //Close the file
    joint_weights_path.close();
    //-------------------------------------------------------------
}



//bool RobotController::update_joint_weights(int j_num, double mean_joint_traj_error_threshold)
//{
//    //Read joint trajectory generated by the controller
//    map<string, vector<double> > control_joint_trajectory = readJointTrajectory(file_path_joint_trajectory_);

//    double delta_j_human = 0.0;
//    double delta_j_control = 0.0;

//    double human_to_control_val_error_sign = 0.0;

//    double weight_increase_count = 0.0;
//    double weight_decrease_count = 0.0;

//    double agg_total_joint_trajectory_error = 0.0;
//    double agg_positive_joint_trajectory_error = 0.0;
//    double agg_negative_joint_trajectory_error = 0.0;

//    double mean_joint_trajectory_error = 0.0;


//    //Compare joint trajectory of human and controller
//    for (int conf_num = 1; conf_num < num_configs_ ; conf_num++)
//    {
//        //Rate of joint value change (from one config to the previous one)
//        delta_j_human = human_joint_trajectory_[joint_names_[j_num]][conf_num] - human_joint_trajectory_[joint_names_[j_num]][conf_num-1];
//        delta_j_control = control_joint_trajectory[joint_names_[j_num]][conf_num] - control_joint_trajectory[joint_names_[j_num]][conf_num-1];

//        //Sign of error (required to determine joint weight update direction)
//        human_to_control_val_error_sign = human_joint_trajectory_[joint_names_[j_num]][conf_num] - control_joint_trajectory[joint_names_[j_num]][conf_num];

//        //Print current joint val error
//        //cout<<"Error between human and control joint value for current frame: "<<human_to_control_val_error_sign<<endl;


//        //Count Joint Weight Increase and Decrease requests
//        if (human_to_control_val_error_sign < 0.0 && fabs(human_to_control_val_error_sign) > 0.01)
//        {
//            if (delta_j_control < 0.0)
//            {
//                //Collect sum of positive joint trajectory errors
//                agg_positive_joint_trajectory_error += fabs(human_to_control_val_error_sign);

//                weight_increase_count++;

//                //weight_increase_count += fabs(human_to_control_val_error_sign);
//            }

//            if (delta_j_control > 0.0)
//            {
//                //Collect sum of negative joint trajectory errors
//                agg_negative_joint_trajectory_error += fabs(human_to_control_val_error_sign);

//                weight_decrease_count++;

//                //weight_decrease_count += fabs(human_to_control_val_error_sign);
//            }
//        }
//        else if (human_to_control_val_error_sign > 0.0 && fabs(human_to_control_val_error_sign) > 0.01)
//        {
//            if (delta_j_control < 0.0)
//            {
//                //Collect sum of negative joint trajectory errors
//                agg_negative_joint_trajectory_error += fabs(human_to_control_val_error_sign);

//                weight_decrease_count++;

//                //weight_decrease_count += fabs(human_to_control_val_error_sign);
//            }
//            if (delta_j_control > 0.0)
//            {
//                //Collect sum of positive joint trajectory errors
//                agg_positive_joint_trajectory_error += fabs(human_to_control_val_error_sign);

//                weight_increase_count++;

//                //weight_increase_count += fabs(human_to_control_val_error_sign);
//            }
//        }
//        else
//        {
//            //cout<<"No joint weight increase or decrease requested, i.e. human and control joint value close enough to each other for current frame"<<endl;
//        }

//        agg_total_joint_trajectory_error =  agg_total_joint_trajectory_error + fabs(human_to_control_val_error_sign);
//    }



//     //weight_increase_count = weight_increase_count * (agg_positive_joint_trajectory_error/ weight_increase_count);
//     //weight_decrease_count = weight_decrease_count * (agg_negative_joint_trajectory_error/ weight_decrease_count);


//    //weight_increase_count = weight_increase_count * (agg_positive_joint_trajectory_error/ double(num_configs_));
//    //weight_decrease_count = weight_decrease_count * (agg_negative_joint_trajectory_error/ double(num_configs_));

//    weight_increase_count = agg_positive_joint_trajectory_error/ double(num_configs_);
//    weight_decrease_count = agg_negative_joint_trajectory_error/ double(num_configs_);


//     cout<<"Weighted increase counter: "<<weight_increase_count<<endl;
//     cout<<"Weighted decrease counter: "<<weight_decrease_count<<endl;


//    //Determine mean trajectory error using the corrent joint weight
//    mean_joint_trajectory_error = agg_total_joint_trajectory_error / double(num_configs_);
//    cout<<"Mean trajectory error for joint "<<joint_names_[j_num]<<" is: "<<mean_joint_trajectory_error<<endl;

//    //Determine which counter is higher
//    //int max_counter_val = (weight_decrease_count < weight_increase_count) ? weight_increase_count : weight_decrease_count ;

//    //Stop optimizing joint weight when a lower threshold of update requests is met
//    //if (max_counter_val < 30)
//    //    return true;


//    cout<<"Weight modification "<<max_joint_weight_modification_<<endl;

//    //Stop optimizing joint weight when a lower threshold of mean joint trajectory error is met
//    if (mean_joint_trajectory_error < mean_joint_traj_error_threshold || fabs(weight_increase_count - weight_decrease_count) < 0.005)
//    {
//        //cout<<"Final mean trajectory error for joint "<<joint_names_[j_num]<<" is: "<<mean_joint_trajectory_error<<endl;
//        cout<<"Final weight for joint "<<joint_names_[j_num]<<" is: "<<joint_weights_(j_num)<<endl;
//        return true;
//    }


// //    if (fabs(weight_increase_count - weight_decrease_count) < 0.001)
// //    {

// //        cout<<"o"<<endl;

// //        //Reduce joint weight update rate
// //        max_joint_weight_modification_ = 0.8 * max_joint_weight_modification_;

// //        if (joint_weight_gradient_direction_ == 0)
// //        {
// //            joint_weights_(j_num) = joint_weights_(j_num) + max_joint_weight_modification_ ;//* (double(weight_increase_count-weight_decrease_count)/double(weight_increase_count));
// //        }
// //        if (joint_weight_gradient_direction_ == 1)
// //        {
// //            if (max_joint_weight_modification_ < joint_weights_(j_num))
// //                joint_weights_(j_num) = joint_weights_(j_num) - max_joint_weight_modification_ ;//* (double(weight_increase_count-weight_decrease_count)/double(weight_increase_count));
// //            else
// //                joint_weights_(j_num) = joint_weights_(j_num) / 2.0;

// //        }

// //        //Update joint weight gradient direction
// //        if (joint_weight_gradient_direction_  == 0)
// //            joint_weight_gradient_direction_ = 1;
// //        else
// //            joint_weight_gradient_direction_ = 0;


// //        //cout<<"Gradient direction "<<joint_weight_gradient_direction_<<endl;
// //        cout<<"New weight for joint "<<joint_names_[j_num]<<" is: "<<joint_weights_(j_num)<<endl;

// //        //Continue optimizing joint weight
// //        return false;

// //        //cout<<"Number of Joint weight increase and decrease requests is equal, thus no further optimization possible!"<<endl;
// //        //return true;
// //    }

//    if (weight_decrease_count < weight_increase_count)
//    {
//        //Reduce joint weight update rate (if joint weight has been modified in the negative direction in the previous iteration)
//        if (joint_weight_gradient_direction_ == 1)
//            max_joint_weight_modification_ = 0.8 * max_joint_weight_modification_;

//        joint_weights_(j_num) = joint_weights_(j_num) + max_joint_weight_modification_ * (double(weight_increase_count-weight_decrease_count)/double(weight_increase_count));

//        //Set flag indicating that joint weight has been modified in the positive direction
//        joint_weight_gradient_direction_ = 0;

//        cout<<"New weight for joint "<<joint_names_[j_num]<<" is: "<<joint_weights_(j_num)<<endl;

//        //Continue optimizing joint weight
//        return false;
//    }

//    if (weight_increase_count < weight_decrease_count)
//    {
//        //Reduce joint weight update rate (if joint weight has been modified in the positive direction in the previous iteration)
//        if (joint_weight_gradient_direction_ == 0)
//            max_joint_weight_modification_ = 0.8 * max_joint_weight_modification_;

//        if (max_joint_weight_modification_ < joint_weights_(j_num))
//            joint_weights_(j_num) = joint_weights_(j_num) - max_joint_weight_modification_ * double(double(weight_decrease_count-weight_increase_count)/double(weight_decrease_count));
//        else
//            joint_weights_(j_num) = joint_weights_(j_num) / 2.0;

//        //Set flag indicating that joint weight has been modified in the negative direction
//        joint_weight_gradient_direction_ = 1;

//        cout<<"New weight for joint "<<joint_names_[j_num]<<" is: "<<joint_weights_(j_num)<<endl;

//        //Continue optimizing joint weight
//        return false;
//    }

//    cout<<endl;


//}


//Read joint trajectory (of the human subject and motion segment)
map<string, vector<double> > RobotController::readJointTrajectory(char* joint_traj_filename)
{

    //Map to be returned
    map<string, vector<double> > joint_trajectory;

    // --------------- Get the number of rows of the file --------------------------
    string line;
    num_configs_ = 0;
    int num_vals_per_line = 0;
    bool num_vals_read = false;

    char * pEnd;
    char char_line[1000];
    vector<string> joint_names_from_file;

    ifstream jVal_file(joint_traj_filename);
    if (jVal_file.is_open())
    {
        while ( jVal_file.good() )
        {
             std::getline(jVal_file,line);

             //Get the number of values per line once
             if (num_vals_read == false)
             {
                 //Set flag to true
                 num_vals_read = true;

                 for (int r = 0; r<2; r++)
                 {

                     istringstream buf(line);
                     istream_iterator<string> beg(buf), end;
                     vector<string> substrings(beg, end); // done!

                     //Get number of values per line
                     num_vals_per_line = num_vals_per_line + substrings.size();

                     //Transform string into char array
                     strcpy(char_line,line.c_str());
                     char * j_names = std::strtok (char_line," ");

                     while (j_names != 0)
                     {
                       joint_names_from_file.push_back(j_names);
                       //cout<< j_names<<endl;
                       j_names = std::strtok(NULL," ");
                     }

                     //Next Line of joint names
                     std::getline(jVal_file,line);
                 }
             }

             //Increment number of rows found in the file
             num_configs_++;
        }
        //Delete last line containing eof
        num_configs_--;
        //Delete first two lines containing joint names and joint weights
        num_configs_ = num_configs_ - 2;
        //std::cout <<"File cotains " <<num_configs_ << " configurations" << std::endl;
        //std::cout <<"Line has " <<num_vals_per_line << " values" << std::endl;
        jVal_file.close();
    }

    else std::cout << "Unable to open file (readHumanJointTrajectory)";


    //--------------- Fill the 2D Array --------------------------
    //Row number counter
    int curr_row_num = 1;
    jVal_file.open(joint_traj_filename);
    if (jVal_file.is_open())
    {
        while (jVal_file.good() )
        {
         //read next line from file (either a configuration or eof)
         std::getline (jVal_file,line);

         //Detect when end of file is reached
         if (jVal_file.eof())
             break;

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp = char_line;

         //Get start and end frame config of motion segment
         for (int i = 0 ; i < num_vals_per_line ; i++)
         {
           if (curr_row_num >= 4) //row 1 and 2 = j_names ,  row 3 = joint_weights
           {
               joint_trajectory[joint_names_from_file[i]].push_back(strtod(tmp,&pEnd));
               //if (i == 0)
               // cout<<"Value for joint: "<<joint_names_from_file[i]<<" is: "<<human_joint_trajectory_[joint_names_from_file[i]][curr_row_num-4]<<endl;
           }
           else
               break;

           //Assign the remaining char elements (pEnd) to the current char pointer
           tmp = pEnd;

         }


         //Increment row counter
         curr_row_num++;

        }

        //Close file
        jVal_file.close();
    }

    else std::cout << "Unable to open file (readHumanJointTrajectory)";


    //Return the map storing the joint trajectory
    return joint_trajectory;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++ AUXILIARY FUNCTIONS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//Compute the forward kinematics for a kinematic chain
//std::vector<double> RobotController::compute_FK(KDL::Chain kin_chain, KDL::JntArray chain_config)
//{
//    //Forward Kinematic Solver for kinematic chain
//    KDL::ChainFkSolverPos_recursive fk_solver_body_chain(kin_chain);

//    //Stores the FK result: Current Endeffector pose in cartesian space expressd as transformation matrix-> expressed w.r.t base frame
//    KDL::Frame ee_cart_pose;

//    //Stores the EE position and orientation, expressed as a vector (x,y,z, Xrot, Yrot, Zrot)
//    std::vector<double> ee_pose(6);

//    //Solve forward kinematics for body chain
//    int success = fk_solver_body_chain.JntToCart(chain_config, ee_cart_pose);

//    //Get EE position
//    ee_pose[0] =  ee_cart_pose.p.x();
//    ee_pose[1] =  ee_cart_pose.p.y();
//    ee_pose[2] =  ee_cart_pose.p.z();
//    //Get EE orientation (ZYZ-euler angles)
//    ee_cart_pose.M.GetEulerZYX(ee_pose[5], ee_pose[4], ee_pose[3]); //GetEulerZYX gets the euler ZYX parameters of a rotation : First rotate around Z with alfa, then around the new Y with beta, then around new X with gamma.


//    return ee_pose;
//}



//Get the Jacobian for a given chain and configuration
Eigen::MatrixXf RobotController::getJacobian(KDL::Chain kin_chain, KDL::JntArray config)
{
    //Jacobian Solver
    KDL::ChainJntToJacSolver jac_solver(kin_chain);

    KDL::Jacobian jac(kin_chain.getNrOfJoints());

    //Copmute the Jacobian
    jac_solver.JntToJac(config,jac);

    //Eigen Matrix storing Jacobian
    Eigen::MatrixXf eigen_jac(jac.rows(),jac.columns());

    //Convert to Eigen Matrix
    //std::cout<<"Jacobian: "<<std::endl;
    for (int i = 0; i <  jac.rows(); i++)
    {
        for (int j = 0; j <  jac.columns(); j++)
        {
                eigen_jac(i,j) = jac.operator ()(i,j);
                //std::cout<<eigen_jac(i,j)<<" ";
        }
        //std::cout<<std::endl;
    }
    //std::cout<<std::endl;

    //Return Jacobian
    return eigen_jac;
}



//Compute the Pseudoinverse of a Matrix
double** RobotController::compute_J_pinv(Eigen::MatrixXf jacobian)
{

    //Jacobian Pseudoinverse (pinv_J has number of columns equal to the number of rows of the original jacobian)
    double **jac_pinv;
    jac_pinv = new double*[jacobian.cols()];
    for (int i = 0; i < jacobian.cols(); i++)
       jac_pinv[i] = new double[jacobian.rows()];


    //Eigen Matrix storing the jacobian matrix
    Eigen::MatrixXd mat(jacobian.rows(),jacobian.cols()); // = Eigen::MatrixXd::Random(6, 11);
    //Copy Jacobian into Eigen Matrix
    for (int i = 0; i < jacobian.rows(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            mat(i,j) = jacobian(i,j);
        }
    }
    //std::cout << "Here is the matrix mat:" << std::endl << mat << std::endl;



    //Build a matrix with the joint weights on the diagonal (i.e. introducing joint preference for the motion)
    Eigen::MatrixXd j_weights_test(jacobian.cols(),jacobian.cols());

    for (int i = 0; i < jacobian.cols(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            if (i == j)
            {
                j_weights_test(i,j) = joint_weights_(i);
//                if (i < 9)
//                    j_weights_test(i,j) = 0.0;
//                else
//                    j_weights_test(i,j) = 1.0;
            }
            else
                j_weights_test(i,j) = 0.0;
        }
    }
    //std::cout << "Here is the matrix j_weights_test:" << std::endl << j_weights_test << std::endl;


    //Apply Joint Weighting, i.e. set the joint motion preference
    Eigen::MatrixXd mat2(jacobian.rows(),jacobian.cols());
    mat2= mat*j_weights_test;
    //std::cout << "Here is the matrix test:" << std::endl << mat2 << std::endl;


    // Compute SVD (as a result we obtain U*S*V)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat2,  Eigen::ComputeFullU | Eigen::ComputeFullV);
    //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    //std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
    //std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;

    Eigen::MatrixXd eigen_J_pinv(jacobian.cols(),jacobian.rows());
    eigen_J_pinv = Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());

    //    int num_sing_vals = 0;
    //    if( jacobian.rows() <= jacobian.cols())
    //        num_sing_vals = jacobian.rows();
    //    else
    //        num_sing_vals = jacobian.cols();

    int num_sing_vals = svd.singularValues().size();


    //Compute Pseudoinverse using the svd
    for (int i = 0 ; i < num_sing_vals ; i++)
    {
        //cout<<svd.singularValues()[i]<<"  ";
        eigen_J_pinv = eigen_J_pinv + ( 1 / svd.singularValues()[i]) *  svd.matrixV().col(i) * svd.matrixU().col(i).transpose();
        //std::cout<< ( (svd.singularValues()[i]/(svd.singularValues()[i] * svd.singularValues()[i] + damping_factor * damping_factor)) *  svd.matrixV().col(i) * svd.matrixU().col(i).transpose())<<std::endl;

    }
    //cout<<eigen_J_pinv<<endl<<endl;


    //std::cout<<"Pseudoinverse"<<std::endl;
    for (int i = 0 ; i < eigen_J_pinv.rows() ; i++)
    {
        for (int j = 0 ; j < eigen_J_pinv.cols() ; j++)
        {
            jac_pinv[i][j] =  eigen_J_pinv(i,j);
            //std::cout<<jac_pinv[i][j]<<" ";
        }
        //std::cout<<std::endl;
    }
    //std::cout<<std::endl;


    //return the pseudoinverse
    return jac_pinv;

}



//Compute the Damped-least squares jacobian matrix
double **RobotController::compute_J_dls(Eigen::MatrixXf jacobian, float damping_factor)
{
    //Stores resulting damped-least squares jacobian
    double **jac_dls;
    jac_dls = new double*[jacobian.cols()];
    for (int i = 0; i < jacobian.cols(); i++)
       jac_dls[i] = new double[jacobian.rows()];


    //Eigen Matrix storing the jacobian matrix
    Eigen::MatrixXd mat(jacobian.rows(),jacobian.cols()); // = Eigen::MatrixXd::Random(6, 11);
    //Copy Jacobian into Eigen Matrix
    for (int i = 0; i < jacobian.rows(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            mat(i,j) = jacobian(i,j);
        }
    }
    //std::cout << "Here is the matrix m:" << std::endl << m << std::endl;


    //Build a matrix with the joint weights on the diagonal (i.e. introducing joint preference for the motion)
    Eigen::MatrixXd j_weights_test(jacobian.cols(),jacobian.cols());

    for (int i = 0; i < jacobian.cols(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            if (i == j)
            {
                j_weights_test(i,j) = joint_weights_(i);
//                if (i < 9)
//                    j_weights_test(i,j) = 0.0;
//                else
//                    j_weights_test(i,j) = 1.0;
            }
            else
                j_weights_test(i,j) = 0.0;
        }
    }
    //std::cout << "Here is the matrix j_weights_test:" << std::endl << j_weights_test << std::endl;



    //Apply Joint Weighting, i.e. set the joint motion preference
    Eigen::MatrixXd mat2(jacobian.rows(),jacobian.cols());
    mat2= mat*j_weights_test;
    //std::cout << "Here is the matrix test:" << std::endl << mat2 << std::endl;


    // Compute SVD (as a result we obtain U*S*V)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat2,  Eigen::ComputeFullU | Eigen::ComputeFullV);
    //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    //std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
    //std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;

    Eigen::MatrixXd eigen_J_dls(jacobian.cols(),jacobian.rows());
    eigen_J_dls = Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());

//    int num_sing_vals = 0;
//    if( jacobian.rows() <= jacobian.cols())
//        num_sing_vals = jacobian.rows();
//    else
//        num_sing_vals = jacobian.cols();

    int num_sing_vals = svd.singularValues().size();


    //Compute J_dls as Eigen Matrix
    for (int i = 0 ; i < num_sing_vals ; i++)
    {
        eigen_J_dls = eigen_J_dls + ( (svd.singularValues()[i]/(svd.singularValues()[i] * svd.singularValues()[i] + damping_factor * damping_factor)) *  svd.matrixV().col(i) * svd.matrixU().col(i).transpose());
        //std::cout<< ( (svd.singularValues()[i]/(svd.singularValues()[i] * svd.singularValues()[i] + damping_factor * damping_factor)) *  svd.matrixV().col(i) * svd.matrixU().col(i).transpose())<<std::endl;

    }


   //Compute J_dls as Eigen Matrix
    //std::cout<<"Jacobian damped_least squares"<<std::endl;
    for (int i = 0 ; i < eigen_J_dls.rows() ; i++)
    {
        for (int j = 0 ; j < eigen_J_dls.cols() ; j++)
        {
            jac_dls[i][j] =  eigen_J_dls(i,j);
            //std::cout<<jac_dls[i][j]<<" ";
        }
        //std::cout<<std::endl;
    }
    //std::cout<<std::endl;

    //Return the damped least squares jacobian
    return jac_dls;
}


//Compute the Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
double **RobotController::compute_J_vdls(Eigen::MatrixXf jacobian , float min_manip_treshold, float max_damping_factor)
{
    //Stores resulting damped-least squares jacobian
    double **jac_vdls;
    jac_vdls = new double*[jacobian.cols()];
    for (int i = 0; i < jacobian.cols(); i++)
       jac_vdls[i] = new double[jacobian.rows()];



    //Eigen Matrix storing the jacobian matrix
    Eigen::MatrixXd mat(jacobian.rows(),jacobian.cols()); // = Eigen::MatrixXd::Random(6, 11);
    for (int i = 0; i < jacobian.rows(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            mat(i,j) = jacobian(i,j);
        }
    }
    //std::cout << "Here is the matrix m:" << std::endl << m << std::endl;


    //Build a matrix with the joint weights on the diagonal (i.e. introducing joint preference for the motion)
    Eigen::MatrixXd j_weights_test(jacobian.cols(),jacobian.cols());

    for (int i = 0; i < jacobian.cols(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            if (i == j)
            {
                j_weights_test(i,j) = joint_weights_(i);
//                if (i < 9)
//                    j_weights_test(i,j) = 0.0;
//                else
//                    j_weights_test(i,j) = 1.0;
            }
            else
                j_weights_test(i,j) = 0.0;
        }
    }
    //std::cout << "Here is the matrix j_weights_test:" << std::endl << j_weights_test << std::endl;



    //Apply Joint Weighting, i.e. set the joint motion preference
    Eigen::MatrixXd mat2(jacobian.rows(),jacobian.cols());
    mat2= mat*j_weights_test;
    //std::cout << "Here is the matrix test:" << std::endl << mat2 << std::endl;



    //Compute Manipulability Measure for the given Jacobian Matrix
    double manip_measure = computeManipulabilityMeasure(jacobian);


    //Compute the damping factor based on the Manipulability Measure (proposed by Nakamura)
    double damping_factor = 0.0;
    if(manip_measure < min_manip_treshold)
    {
        damping_factor = max_damping_factor * (( 1 - (manip_measure/min_manip_treshold)) * ( 1 - (manip_measure/min_manip_treshold)));
        //std::cout<<"Damping active with factor: "<<damping_factor<<std::endl;
    }
    else
        damping_factor = 0.0;



    // Compute SVD (as a result we obtain U*S*V)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat2,  Eigen::ComputeFullU | Eigen::ComputeFullV);
    //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    //std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
    //std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;

    Eigen::MatrixXd eigen_J_vdls(jacobian.cols(),jacobian.rows());
    eigen_J_vdls = Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());

    //    int num_sing_vals = 0;
    //    if( jacobian.rows() <= jacobian.cols())
    //        num_sing_vals = jacobian.rows();
    //    else
    //        num_sing_vals = jacobian.cols();

    int num_sing_vals = svd.singularValues().size();

    //Compute J_dls as Eigen Matrix
    for (int i = 0 ; i < num_sing_vals ; i++)
    {
        eigen_J_vdls = eigen_J_vdls + ( (svd.singularValues()[i]/(svd.singularValues()[i] * svd.singularValues()[i] + damping_factor * damping_factor)) *  svd.matrixV().col(i) * svd.matrixU().col(i).transpose());
        //std::cout<< ( (svd.singularValues()[i]/(svd.singularValues()[i] * svd.singularValues()[i] + damping_factor * damping_factor)) *  svd.matrixV().col(i) * svd.matrixU().col(i).transpose())<<std::endl;

    }


    //cout<<"Task frame jacobian pseudoinverse"<<endl;
    //cout<<eigen_J_vdls<<endl;

    //Compute J_vdls as Eigen Matrix
    for (int i = 0 ; i < eigen_J_vdls.rows() ; i++)
    {
        for (int j = 0 ; j < eigen_J_vdls.cols() ; j++)
        {
            jac_vdls[i][j] =  eigen_J_vdls(i,j);
        }
    }

    //Return the damped least squares jacobian
    return jac_vdls;

}



//Compute the Weighted Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
double **RobotController::compute_J_wvdls(Eigen::MatrixXf jacobian, float min_manip_treshold, float max_damping_factor, std::vector<double> previous_jl_grad_func_val, std::vector<double> jl_grad_func_val)
{

    //Stores resulting damped-least squares jacobian
    double **jac_wvdls;
    jac_wvdls = new double*[jacobian.cols()];
    for (int i = 0; i < jacobian.cols(); i++)
       jac_wvdls[i] = new double[jacobian.rows()];



    //std::cout << "Here is the jacobian:" << std::endl << jacobian << std::endl;

//    //Eigen Matrix storing the jacobian matrix
//    Eigen::MatrixXd m(jacobian.rows(),jacobian.cols()); // = Eigen::MatrixXd::Random(6, 11);
//    for (int i = 0; i < jacobian.rows(); i++)
//    {
//        for (int j = 0; j < jacobian.cols(); j++)
//        {
//            m(i,j) = jacobian(i,j);
//        }
//    }
//    //std::cout << "Here is the matrix m:" << std::endl << m << std::endl;


    //Build a matrix with the joint weights on the diagonal (i.e. introducing joint preference for the motion)
    Eigen::MatrixXf j_weights_test(jacobian.cols(),jacobian.cols());

    for (int i = 0; i < jacobian.cols(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            if (i == j)
            {
                j_weights_test(i,j) = joint_weights_(i);
//                if (i < 9)
//                    j_weights_test(i,j) = 0.0;
//                else
//                    j_weights_test(i,j) = 1.0;
            }
            else
                j_weights_test(i,j) = 0.0;
        }
    }
    //std::cout << "Here is the matrix j_weights_test:" << std::endl << j_weights_test << std::endl;



    //Apply Joint Weighting, i.e. set the joint motion preference
    Eigen::MatrixXf jac_weighted(jacobian.rows(),jacobian.cols());
    jac_weighted= jacobian*j_weights_test;

    //std::cout << "Here is the weighted jacobian:" << std::endl << jac_weighted << std::endl;


    //Compute the transpose of the jacobian
    Eigen::MatrixXf jac_transpose = jac_weighted.transpose();


    //Compute Manipulability Measure for the given Jacobian Matrix
    double manip_measure = computeManipulabilityMeasure(jacobian);


    //Compute the damping factor based on the Manipulability Measure (proposed by Nakamura)
    double damping_factor = 0.0;

    if(manip_measure < min_manip_treshold)
    {
        damping_factor = max_damping_factor * (( 1 - (manip_measure/min_manip_treshold)) * ( 1 - (manip_measure/min_manip_treshold)));
        std::cout<<"Damping active with factor: "<<damping_factor<<std::endl;
    }
    else
        damping_factor = 0.0;


    //Generate Damping Matrix (damping_factor_sq * IdentityMatrix)
    Eigen::MatrixXf damping_matrix(jacobian.rows(),jacobian.rows());
    damping_matrix = Eigen::MatrixXf::Identity(jacobian.rows(), jacobian.rows());
    damping_matrix = (damping_factor*damping_factor) * damping_matrix;


//    std::cout<<"damping_matrix: "<<std::endl;
//    std::cout<<damping_matrix<<std::endl<<std::endl;



    //Weighting Matrix
    Eigen::MatrixXf weighting_matrix(jacobian.cols(),jacobian.cols());
    weighting_matrix = Eigen::MatrixXf::Zero(jacobian.cols(), jacobian.cols());

    //Compute Weighting matrix based on current and previous JL Gradient vector
    //Change of the gradient function value
    double delta_grad_func = 0.0;
    //Current joint weight
    double weight = 0.0;
    for (int i = 0 ; i < jacobian.cols() ; i++)
    {
        //Change of the gradient function value
        delta_grad_func = fabs(jl_grad_func_val[i]) - fabs(previous_jl_grad_func_val[i]);

        //Compute the weight for joint i, depending on the change of the gradient function value
        if(delta_grad_func >= 0.0)
        {
            weight = 1.0 +  std::abs(jl_grad_func_val[i]);
        }
        else
        {
            weight = 1.0;
        }
        //Insert weight on the diagonal of the weighting matrix
        weighting_matrix(i,i) = weight;
    }

//    std::cout<<"weighting_matrix: "<<std::endl;
//    std::cout<<weighting_matrix<<std::endl<<std::endl;

//    std::cout<<"weighting_matrix inverse: "<<std::endl;
//    std::cout<<weighting_matrix.inverse()<<std::endl<<std::endl;

    //Second term of eigen_J_wvdls computation, i.e. [J*W_inv*J' + lambda_sq * I]
    Eigen::MatrixXf second_term(jacobian.rows(),jacobian.rows());
    second_term = Eigen::MatrixXf::Zero(jacobian.rows(), jacobian.rows());
    second_term = jac_weighted * weighting_matrix.inverse() * jac_transpose + damping_matrix;

    //Compute the Weighted Least-Norm Pseudoinverse W_inv * J' * [J*W_inv*J' + lambda_sq * I]^-1 (using eigen matrices)
    Eigen::MatrixXf eigen_J_wvdls(jacobian.cols(),jacobian.rows());
    eigen_J_wvdls = Eigen::MatrixXf::Zero(jacobian.cols(), jacobian.rows());
    eigen_J_wvdls = (weighting_matrix.inverse() * jac_transpose) * second_term.inverse();



    //Compute J_vdls as Eigen Matrix
    for (int i = 0 ; i < eigen_J_wvdls.rows() ; i++)
    {
        for (int j = 0 ; j < eigen_J_wvdls.cols() ; j++)
        {
            jac_wvdls[i][j] =  eigen_J_wvdls(i,j);
        }
    }

    //Return the damped least squares jacobian
    return jac_wvdls;

}


//Compute the Weighted Least-Norm jacobian matrix (considers joint limit avoidance, but no damping)
double **RobotController::compute_J_wln(Eigen::MatrixXf jacobian, std::vector<double> previous_jl_grad_func_val, std::vector<double> jl_grad_func_val)
{
    //Stores resulting damped-least squares jacobian
    double **jac_wln;
    jac_wln = new double*[jacobian.cols()];
    for (int i = 0; i < jacobian.cols(); i++)
       jac_wln[i] = new double[jacobian.rows()];


    //Build a matrix with the joint weights on the diagonal (i.e. introducing joint preference for the motion)
    Eigen::MatrixXf j_weights_test(jacobian.cols(),jacobian.cols());

    for (int i = 0; i < jacobian.cols(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            if (i == j)
            {
                j_weights_test(i,j) = joint_weights_(i);
            }
            else
                j_weights_test(i,j) = 0.0;
        }
    }
    //std::cout << "Here is the matrix j_weights_test:" << std::endl << j_weights_test << std::endl;



    //Apply Joint Weighting, i.e. set the joint motion preference
    Eigen::MatrixXf jac_weighted(jacobian.rows(),jacobian.cols());
    jac_weighted= jacobian*j_weights_test;
    //std::cout << "Here is the matrix test:" << std::endl << mat2 << std::endl;


    //Compute the transpose of the jacobian
    Eigen::MatrixXf jac_transpose = jac_weighted.transpose();

    //    //Eigen Matrix storing the transpose of the jacobian
    //    Eigen::MatrixXf jac_transpose2(jacobian.cols(),jacobian.rows());
    //    //Copy Jacobian into Eigen Matrix
    //    for (int i = 0; i < jacobian.cols(); i++)
    //    {
    //        for (int j = 0; j < jacobian.rows(); j++)
    //        {
    //            jac_transpose2(i,j) = jac_transpose(i,j);
    //        }
    //    }


    //Weighting Matrix
    Eigen::MatrixXf weighting_matrix(jacobian.cols(),jacobian.cols());
    weighting_matrix = Eigen::MatrixXf::Zero(jacobian.cols(), jacobian.cols());

    //Compute Weighting matrix based on current and previous JL Gradient vector
    //Change of the gradient function value
    double delta_grad_func = 0.0;
    //Current joint weight
    double weight = 0.0;
    for (int i = 0 ; i < jacobian.cols() ; i++)
    {
        //Change of the gradient function value
        delta_grad_func = fabs(jl_grad_func_val[i]) - fabs(previous_jl_grad_func_val[i]);

//        cout<<"Current gradient for joint"<<i+1<<": "<<fabs(jl_grad_func_val[i])<<endl;
//        cout<<"Previous gradient for joint"<<i+1<<": "<< fabs(previous_jl_grad_func_val[i])<<endl;


        //Compute the weight for joint i, depending on the change of the gradient function value
        if(delta_grad_func >= 0.0)
        {
            weight = 1.0 +  fabs(jl_grad_func_val[i]);
        }
        else
        {
            weight = 1.0;
        }
        //Insert weight on the diagonal of the weighting matrix
        weighting_matrix(i,i) = weight;
    }


    //Second term of eigen_J_wln computation, i.e. [J*W_inv*J']
    Eigen::MatrixXf second_term(jacobian.rows(),jacobian.rows());
    second_term = Eigen::MatrixXf::Zero(jacobian.rows(), jacobian.rows());
    second_term = jac_weighted * weighting_matrix.inverse() * jac_transpose;


    //Compute the Weighted Least-Norm Pseudoinverse W_inv * J' * [J*W_inv*J']_inv (using eigen matrices)
    Eigen::MatrixXf eigen_J_wln(jacobian.cols(),jacobian.rows());
    eigen_J_wln = Eigen::MatrixXf::Zero(jacobian.cols(), jacobian.rows());

    eigen_J_wln = (weighting_matrix.inverse() * jac_transpose) * second_term.inverse();


//      std::cout<<"Transpose"<<std::endl;
//      std::cout<<jac_transpose<<std::endl<<std::endl;

//      std::cout<<"Weighting matrix"<<std::endl;
//      std::cout<<weighting_matrix<<std::endl<<std::endl;

//      std::cout<<"Weighting matrix Inverse"<<std::endl;
//      std::cout<<weighting_matrix.inverse()<<std::endl<<std::endl;

//      std::cout<<"Jacobian"<<std::endl;
//      std::cout<<jacobian<<std::endl<<std::endl;

//    std::cout<<"First Term "<<std::endl;
//    std::cout<<(weighting_matrix.inverse() * jac_transpose)<<std::endl<<std::endl;

//      std::cout<<"Second Term "<<std::endl;
//      std::cout<<second_term<<std::endl<<std::endl;

//      std::cout<<"Second Term inverse"<<std::endl;
//      std::cout<<second_term.inverse()<<std::endl<<std::endl;

//      std::cout<<"Jacobian WLN"<<std::endl;
//      std::cout<<eigen_J_wln<<std::endl<<std::endl;


    //Copy eigen Matrix into double pointer array
    for (int i = 0 ; i < eigen_J_wln.rows() ; i++)
    {
        for (int j = 0 ; j < eigen_J_wln.cols() ; j++)
        {
            jac_wln[i][j] =  eigen_J_wln(i,j);
        }
    }

    //Return the weighted least-norm pseudoinverse
    return jac_wln;

}




//Compute the transpose of the jacobian matrix (including variable time step)
double **RobotController::compute_J_trans(Eigen::MatrixXf jacobian, std::vector<double> error)
{


    //Build a matrix with the joint weights on the diagonal (i.e. introducing joint preference for the motion)
    Eigen::MatrixXf j_weights_test(jacobian.cols(),jacobian.cols());

    for (int i = 0; i < jacobian.cols(); i++)
    {
        for (int j = 0; j < jacobian.cols(); j++)
        {
            if (i == j)
            {
                j_weights_test(i,j) = joint_weights_(i);
//                if (i < 9)
//                    j_weights_test(i,j) = 0.0;
//                else
//                    j_weights_test(i,j) = 1.0;
            }
            else
                j_weights_test(i,j) = 0.0;
        }
    }
    //std::cout << "Here is the matrix j_weights_test:" << std::endl << j_weights_test << std::endl;



    //Apply Joint Weighting, i.e. set the joint motion preference
    Eigen::MatrixXf jac_weighted(jacobian.rows(),jacobian.cols());
    jac_weighted= jacobian*j_weights_test;
    //std::cout << "Here is the matrix test:" << std::endl << mat2 << std::endl;

    //Compute transpose
    Eigen::MatrixXf jacobian_trans = jac_weighted.transpose();

    //Convert double vector to eigen vector
    Eigen::VectorXf eig_error(error.size());

    for (int p = 0; p < error.size(); p++)
        eig_error[p] = error[p];


    //Stores Jacobian transpose
    double **jac_trans;
    jac_trans = new double*[jacobian.cols()];
    for (int i = 0; i < jacobian.cols(); i++)
       jac_trans[i] = new double[jacobian.rows()];

    //Compute time step based on current error
    float time_step = 0.0;
    Eigen::MatrixXf jxjtrans = jac_weighted*jacobian_trans;
    Eigen::VectorXf jxjtransxerror = jxjtrans * eig_error;
    time_step = eig_error.dot(jxjtransxerror) / jxjtransxerror.dot(jxjtransxerror);


    for (int i = 0 ; i < jacobian.cols(); i++)
    {
        for (int j = 0 ; j < jacobian.rows(); j++)
        {
            jac_trans[i][j] = time_step * jacobian_trans(i,j) ;
        }
    }

    return jac_trans;
}


//Inversion of a Square Matrix
double **RobotController::compute_inverse(double** matrix, int num_rows, int num_cols)
{

    if (num_rows == num_cols)
    {
        //Resulting inverse matrix
        double **m_inv;
        m_inv = new double*[num_rows];
        for (int i = 0; i < num_rows; i++)
           m_inv[i] = new double[num_rows];


        //Compute 1/det(A)* (adj(A))
        double det_jac = compute_determinant(matrix, num_rows, num_cols);

        std::cout<<"determinant of jacobian"<<std::endl;
        std::cout<<det_jac<<std::endl;


        double **adj_jac = compute_adjugate(matrix, num_rows, num_cols);


//        std::cout<<"Adjugate of Jacobian: "<<std::endl;
//          for (int i = 0; i < num_rows; i++)
//          {
//              for (int j = 0; j < num_cols; j++)
//              {
//                  std::cout<<adj_jac[i][j]<<"  ";
//              }
//              std::cout<<std::endl;
//          }
//          std::cout<<std::endl;


        for (int i = 0 ; i < num_rows; i++)
        {
            for (int j = 0 ; j < num_rows; j++)
            {
                m_inv[i][j] = (1.0/det_jac) * adj_jac[i][j];
            }
        }

        return m_inv;

    }
    else
    {
        ROS_ERROR("Input is not a square matrix!");
        return 0;
    }

}


//Convert JntArray to std::Vector
vector<double> RobotController::JntArray_to_Vector(KDL::JntArray jnt_array)
{
    vector<double> vec(jnt_array.rows());

    for(int i = 0 ; i < jnt_array.rows() ; i++)
        vec[i]= jnt_array(i);

    return vec;
}


//Convert std::Vector to JntArray
KDL::JntArray RobotController::Vector_to_JntArray(vector<double> vec)
{
    KDL::JntArray jnt_array(vec.size());

    for(int i = 0 ; i < vec.size() ; i++)
        jnt_array(i)= vec[i];

    return jnt_array;
}


 //Compute Nakamura's Manipulability Measure
double RobotController::computeManipulabilityMeasure(Eigen::MatrixXf jacobian)
{

//    Eigen::MatrixXf jxjtrans = jacobian * jacobian.transpose();
//    double manipulability = sqrt(jxjtrans.determinant());

//    return manipulability;

    //Init manipulability measure
    double manipulability = 1.0;

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //cout << "Its singular values are:" << endl << svd.singularValues() << endl;
    //cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
    //cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

    //Singular values vector
    std::vector<double> e_values(svd.singularValues().size());

    for (int ev = 0; ev < e_values.size(); ev++)
    {
         e_values[ev] = svd.singularValues()[ev];
         //std::cout << e_values[ev]<<"  ";
         if(fabs(e_values[ev]) > 0.00001)
             manipulability *= fabs(e_values[ev]);
         else
         {
             //std::cout<<"Singular value smaller than 0.00001"<<endl;
         }
    }
    //std::cout<<endl;

    //Manipulability value check
    if (manipulability == 1.0 || manipulability < 0.00001) //a value manipulability = 1.0 indicates that all eigenvalues above are zero < 0.00001, i.e. robot close to singularity
        manipulability = 0.0001;  //Set the manipulability to a very low value -> to get high damping when all eigenvalues are zero

    //cout<<manipulability<<endl;

    return manipulability;
}


//Determinant of a Matrix
double RobotController::compute_determinant(double **matrix, int num_rows, int num_cols)
{
    //Resulting Determinant
    double det = 0.0;

    Eigen::MatrixXd mat_tmp(num_rows,num_cols);


    //Copy double pointer array into Eigen matrix
    for (int i = 0 ; i < num_rows; i++)
    {
        for (int j = 0 ; j < num_cols; j++)
        {
            mat_tmp(i,j) = matrix[i][j];
        }
    }


    //Compute Determinant of Matrix
    det = mat_tmp.determinant();

    return det;
}




double **RobotController::compute_adjugate(double **matrix, int num_rows, int num_cols)
{

    //Eigen Matrix storing the input matrix
    Eigen::MatrixXd helper_mat(num_rows,num_cols);


    //Copy double array into Eigen Matrix
    for (int i = 0; i < num_rows; i++)
    {
        for (int j = 0; j < num_cols; j++)
        {
            helper_mat(i,j) = matrix[i][j];
        }
    }

    //Compute Adjoint of matrix
    Eigen::MatrixXd mat_adjoint(num_rows,num_cols);
    mat_adjoint = helper_mat.adjoint();


    //Adjugate of the matrix
    double **m_adjugate;
    m_adjugate = new double*[num_rows];
    for (int i = 0; i < num_rows; i++)
       m_adjugate[i] = new double[num_cols];

    //Copy Eigen Adjoint Matrix into double pointer array
    for (int i = 0 ; i < num_rows; i++)
    {
        for (int j = 0 ; j < num_cols; j++)
        {
            m_adjugate[i][j] = mat_adjoint(i,j);
        }
    }

    return m_adjugate;

}


//Compute the Null Space of the jacobian_inverse
double **RobotController::compute_null_space_projection(Eigen::MatrixXf jacobian, double **jacobian_inverse)
{
    //Null-Space of the Jacobian Inverse
    double **null_proj;
    null_proj = new double*[jacobian.cols()];
    for (int i = 0; i < jacobian.cols(); i++)
       null_proj[i] = new double[jacobian.cols()];


    //Stores Jacobian Pseudoinverse
//    double **pinv_jac;
//    pinv_jac = new double*[jacobian.columns()];
//    for (int i = 0; i < jacobian.columns(); i++)
//       pinv_jac[i] = new double[jacobian.rows()];


//    //Compute J_pinv
//    pinv_jac = compute_J_pinv(jacobian);


    //Compute I - J_pinv*J
    double acc = 0.0;
    for (int i = 0 ; i < jacobian.cols(); i++)
    {
        for (int j = 0 ; j < jacobian.cols(); j++)
        {
            acc = 0.0;
            for (int col = 0 ; col < jacobian.rows(); col++)
            {
               acc = acc + (jacobian_inverse[i][col]* jacobian(col,j));
            }

            //Consider Identity matrix on diagonal elements
            if(i == j)
                acc = 1.0 - acc;
            else
                acc = 0.0 -acc;

            null_proj[i][j] = acc;
            //std::cout<<null_proj[i][j]<<"  ";
        }
        //std::cout<<std::endl;
    }
    //std::cout<<std::endl;


    //return the null space projector
    return null_proj;
}


//Compute the joint limit gradient (following Chaumette & Marchand "A redundancy based iterative approach for avoiding joint limits: Application to visual servoing")
std::vector<double> RobotController::computeJLgradient(KDL::JntArray config, std::vector<double> q_min, std::vector<double> q_max, double activation_param)
{
    //JL Gradient vector (to be returned by the function)
    std::vector<double> jl_gradient(config.rows());

    //Check activation_param value (must be in the range 0 < activation_param < 0.5)
    if (activation_param <= 0.0)
    {
       ROS_INFO("JL Gradient Activation Parameter clamped to 0.1");
       activation_param = 0.1;
    }
    if (activation_param >= 0.5)
    {
       ROS_INFO("JL Gradient Activation Parameter clamped to 0.49");
       activation_param = 0.49;
    }

    //Activation thresholds for gradient
    double q_min_activate, q_max_activate;


    for (int i = 0 ; i < config.rows(); i++)
    {
        //Compute joint range
        double joint_range = fabs(q_max[i] - q_min[i]);


        //std::cout<<"Current value for "<<i<<"-th joint is: "<<config(i)<<std::endl;

        //std::cout<<"Joint range for "<<i<<"-th joint is: "<<joint_range<<std::endl;

        //Activation thresholds
        q_min_activate = q_min[i] + activation_param*joint_range;
        q_max_activate = q_max[i] - activation_param*joint_range;

//        std::cout<<"q_min for "<<i<<"-th joint is: "<<q_min[i]<<std::endl;
//        std::cout<<"q_max for "<<i<<"-th joint is: "<<q_max[i]<<std::endl;
//        std::cout<<"q_min_activate for "<<i<<"-th joint is: "<<q_min_activate<<std::endl;
//        std::cout<<"q_max_activate for "<<i<<"-th joint is: "<<q_max_activate<<std::endl;

        //Compute Gradient component for joint i
        if (config(i) >  q_max_activate)
        {
            jl_gradient[i] =  (config(i) - q_max_activate) / joint_range;
        }
        else if (config(i) <  q_min_activate)
        {
            jl_gradient[i] =  (config(i) - q_min_activate) / joint_range;
        }
        else
        {
            jl_gradient[i] =  0.0;
        }

        //std::cout<<"Gradient for "<<i<<"-th joint is: "<<jl_gradient[i]<<std::endl;
    }
    //cout<<std::endl;


    //Return JL Gradient vector
    return jl_gradient;

}


//Function to select Control Strategy based on the Distance to a Singular configuration
int RobotController::control_strategy_selection(KDL::Jacobian current_jacobian)
{
//    //Convert KDL::Jacobian into double pointer array
//    double **curr_jac;
//    curr_jac = new double*[current_jacobian.rows()];
//    for (int i = 0; i < current_jacobian.rows(); i++)
//       curr_jac[i] = new double[current_jacobian.columns()];

//    double det = compute_determinant(curr_jac,current_jacobian.rows(),current_jacobian.columns());


    //--------------------- TODO !!!!!!!!!!!!!!!!!!!!------------------------------------
    //Compute Manipulability


    double det = 1.0;

    //Use Moore-Penrose Pseudoinverse Control Strategy when being far away from Singular Configuration
    if (det >= 0.4)
        return 1;
    //If determinant becomes small, i.e. we are approaching a singular configuration -> switch to Damped-least square Pseudoinverse control strategy
    else
        return 2;
}




//Functions to get Joint and Endeffector Trajectory generated by the controller
vector< map<string, double> > RobotController::getJointTrajectory()
{
    return configurations_;
}
vector< vector<double> > RobotController::getEETrajectory()
{
    return ee_poses_;
}




//Compute Control Points (Position of Spheres located alon the endeffector chain, subsequently used for collision avoidance)
vector<vector<double> > RobotController::compute_CA_ControlPoints(vector<int> joint_indices)
{
    //- Compute Position of spheres along kinematic chain
    vector< vector<double> > control_points;

    //Compute Segment FK (for endeffector)
    vector<double> current_ee_pose;

    for (int i = 0 ; i < joint_indices.size(); i++)
    {
        //Compute Segment FK for current segment
        current_ee_pose = RobotModel->compute_FK_Segment(kin_chain_,current_config_,joint_indices[i]);

        //Get Segment position
        vector <double> segment_pos;
        segment_pos.push_back(current_ee_pose[0]); //X pos
        segment_pos.push_back(current_ee_pose[1]); //Y pos
        segment_pos.push_back(current_ee_pose[2]); //Z pos
        segment_pos.push_back(ca_sphere_radius_); //radius of CA sphere

//        cout<<"X: "<<segment_pos[0]<<endl;
//        cout<<"Y: "<<segment_pos[1]<<endl;
//        cout<<"Z: "<<segment_pos[2]<<endl;

        //Store control points
        control_points.push_back(segment_pos); //segment control point /sphere
    }


    //Return control points
    return control_points;

}


//Compute Repulsive vector (for obstacle avoidance)
vector<double> RobotController::compute_EE_RepulsiveVector(vector<double> ee_position)
{
    //Repulsive vector to be returned
    vector<double> ee_rep_vec(dim_task_space_);

    //Init Artificial Potential Field elements 0,1,2 in cartesian space
    ee_rep_vec[0] = 0.0;
    ee_rep_vec[1] = 0.0;
    ee_rep_vec[2] = 0.0;
    //Artificial Potential Field imposes only a force in cartesian space (not in rotation space)
    ee_rep_vec[3] = 0.0;
    ee_rep_vec[4] = 0.0;
    ee_rep_vec[5] = 0.0;

    //Obstacle potential in cartesian space (Vr)
    vector<double> obs_potential(3);

    //Distance of endeffector to closest obstacle
    double min_obs_dist_norm = 10000.0;

    //Magnitude (repulsive vector) of closest Obstacle (i.e. the one highest obstacle potential magnitude)
    double max_pot_field_mag = 0.0;


    for(int i = 0 ; i < scene_obstacles_.size() ; i++)
    {

        //Distance from endeffector to i-th obstacle
        vector<double> cp_obs_dist = compute_Dist_to_Obs(ee_position, scene_obstacles_[i]);

        //Compute norm of obstacle distance (i.e ||cp_obs_dist||)
        double cp_obs_dist_norm = sqrt( (cp_obs_dist[0]*cp_obs_dist[0]) + (cp_obs_dist[1]*cp_obs_dist[1]) + (cp_obs_dist[2]*cp_obs_dist[2]));

        //Consider radius of obstacle spheres in distance measure
        cp_obs_dist_norm = cp_obs_dist_norm - scene_obstacles_[i][3]; // 3rd element in scene_obstacles_ is radius of sphere

        //If obstacle within endeffector surveillance region
        if (cp_obs_dist_norm < ca_sphere_radius_)
        {
            //Compute Magnitude of repulsive vector (i.e., v(P,O))
            double pot_field_mag = compute_Pot_Field_Magnitude(cp_obs_dist_norm);

            //Save maximum repulsive vector magnitude
            if(cp_obs_dist_norm < min_obs_dist_norm)
            {
                //Store maximum potential field magnitude
                max_pot_field_mag = pot_field_mag;
                //Set minimum obstacle distance norm to current obstacle distance norm
                min_obs_dist_norm = cp_obs_dist_norm;
            }

            //Compute repulsive vector for current obstacle ( mag * (cp_obs_dist / ||cp_obs_dist||) )
            obs_potential[0] = pot_field_mag * (cp_obs_dist[0]/cp_obs_dist_norm);
            obs_potential[1] = pot_field_mag * (cp_obs_dist[1]/cp_obs_dist_norm);
            obs_potential[2] = pot_field_mag * (cp_obs_dist[2]/cp_obs_dist_norm);

            //Compute repulsive vector for endeffector considering current obstacle (Vct)
            ee_rep_vec[0] = ee_rep_vec[0] + obs_potential[0];
            ee_rep_vec[1] = ee_rep_vec[1] + obs_potential[1];
            ee_rep_vec[2] = ee_rep_vec[2] + obs_potential[2];
        }
        else
        {
            //cout<<"No repulsive action for this object required"<<endl;
        }
    }


    //Compute norm of aggregated obstacle potentials
    double cp_all_obs_dist_norm = sqrt( (ee_rep_vec[0]*ee_rep_vec[0]) + (ee_rep_vec[1]*ee_rep_vec[1]) + (ee_rep_vec[2]*ee_rep_vec[2]));

    //Compute repulsive vector for endeffector considering all obstacles (if at least one object is within surveillance region)
    if (cp_all_obs_dist_norm != 0.0)
    {
        ee_rep_vec[0] = max_pot_field_mag * (ee_rep_vec[0]/cp_all_obs_dist_norm);
        ee_rep_vec[1] = max_pot_field_mag * (ee_rep_vec[1]/cp_all_obs_dist_norm);
        ee_rep_vec[2] = max_pot_field_mag * (ee_rep_vec[2]/cp_all_obs_dist_norm);
    }
    else
    {
        //cout<<"No repulsive action required, i.e. all obstacles outside surveillance region"<<endl;
    }

    return ee_rep_vec;
}


//Distance from control points to obstacle
vector<double> RobotController::compute_Dist_to_Obs(vector<double> pos_control_point, vector<double> obstacle_pos)
{
    //Cartesian distance between control point and obstacle
    vector<double> cp_obs_dist(3);

    //Compute distance between points
    if(obstacle_pos[4] == 0.0) //4th element of vector used to indicate that the obstacle is the base platform or another obstacle
    {
          cp_obs_dist[0] = pos_control_point[0] - obstacle_pos[0];
          cp_obs_dist[1] = pos_control_point[1] - obstacle_pos[1];
          cp_obs_dist[2] = pos_control_point[2] - obstacle_pos[2];
    }
    else
    {
        cp_obs_dist[0] = 0.0;
        cp_obs_dist[1] = 0.0;
        cp_obs_dist[2] = pos_control_point[2] - obstacle_pos[2]; //z distance used only when obstacle is the base platform
    }

    return cp_obs_dist;
}


//Compute Magnitude of repulsive vector
double RobotController::compute_Pot_Field_Magnitude(double cp_obs_dist_norm)
{

    double rep_vector_mag = 0.0;

    //Compute repulsive vector magnitude
    rep_vector_mag = max_mag_rep_vec_ / ( 1 + exp( ( cp_obs_dist_norm * ( 2 / ca_sphere_radius_ ) - 1) * ca_alpha_shape_fac_ ) );

    return rep_vector_mag;
}



//Compute risk of collision function (for robot body collision avoidance)
vector<vector<double> > RobotController::compute_risk_func_vector(vector<vector<double> > control_points_pos, Eigen::MatrixXf jacobian)
{

    //Vector used for joint motion scaling (to be returned)
    vector<vector<double> > joint_vel_bounds(num_joints_);

    //Init joint velocity bounds
    for (int i = 0 ; i < num_joints_ ; i++)
    {
        joint_vel_bounds[i].push_back(max_joint_velocity_); //max joint vel
        joint_vel_bounds[i].push_back(-max_joint_velocity_); //min joint vel
    }


    //Risk function vector (s)
    vector<double> risk_func_vec(num_joints_);

    //Init risk function vector
    for (int z = 0; z < num_joints_ ; z++)
        risk_func_vec[z] = 0.0;


    //Compute transpose of jacobian
    Eigen::MatrixXf jacobian_trans = jacobian.transpose();
    //cout<<jacobian_trans<<endl;


    //Compute vector generated by risk function for all control points except the endeffector (ee has index 0)
    for(int i = 1 ; i < control_points_pos.size() ; i++)
    {
        //Minumum values to be stored
        vector<double> cp_obs_dist_min ;
        double cp_obs_dist_norm_min = 10000.0;


        //----------- Start: Find the obstacle closest to the current control point --------
        for(int j = 0 ; j < scene_obstacles_.size() ; j++)
        {
           //Compute distance
           vector<double> cp_obs_dist = compute_Dist_to_Obs(control_points_pos[i],scene_obstacles_[j]);

           //Compute norm of obstacle distance (i.e ||cp_obs_dist||)
           double cp_obs_dist_norm = sqrt( (cp_obs_dist[0]*cp_obs_dist[0]) + (cp_obs_dist[1]*cp_obs_dist[1]) + (cp_obs_dist[2]*cp_obs_dist[2]));


           //Consider radius of obstacle spheres in distance measure
           cp_obs_dist_norm = cp_obs_dist_norm - scene_obstacles_[j][3]; // 3rd element in scene_obstacles_ is radius of sphere


           //Store minimum obstacle distance and its norm
           if (cp_obs_dist_norm < ca_sphere_radius_ && cp_obs_dist_norm < cp_obs_dist_norm_min)
           {
               cp_obs_dist_norm_min = cp_obs_dist_norm;
               cp_obs_dist_min = cp_obs_dist;

               //cout<<"Collision avoidance action for this CA needed!!!"<<endl;

           }
           else
           {
               //cout<<"No collision avoidance action for this CA needed!!!"<<endl;
           }

        }
       //----------- End: Find the obstacle closest to the current control point --------


       //----------- Start: Compute degree of influence for current cartesian consraint ------------
       // -> if an obstacle is within the control point surveillance region
       if (cp_obs_dist_norm_min < 10000.0)
       {
           //Compute risk of collision ( f(Dmin(C)) )
           double risk_func_val = 1.0 / ( 1.0 + exp( ( cp_obs_dist_norm_min * ( 2 / ca_sphere_radius_ ) - 1) * ca_alpha_shape_fac_ ) );

           //Generate vector from risk function value (joint_indices_for_CA_[i] = number of rows of partial jacobian transpose
           for (int n = 0 ; n < joint_indices_for_CA_[i] ; n++)
           {
                risk_func_vec[n] = jacobian_trans(n,0) * ((cp_obs_dist_min[0]/cp_obs_dist_norm_min) * risk_func_val) + jacobian_trans(n,1) * ((cp_obs_dist_min[1]/cp_obs_dist_norm_min) * risk_func_val) + jacobian_trans(n,2) * ((cp_obs_dist_min[2]/cp_obs_dist_norm_min) * risk_func_val);
                //cout<<"Risk: "<<risk_func_vec[n]<<endl;
           }

           //cout<<"Control Point: "<<i<<" slowed down"<<endl;

           //Compute joint velocity bound scaling
           double q_max_scaled = 1000.0;
           double q_min_scaled = 1000.0;
           //for (int k = 0 ; k < num_joints_ ; k++)
           for (int k = 0 ; k < joint_indices_for_CA_[i] ; k++)
           {
               //When the constraint is to close, all joint motions incompatible with the constraint are denied
               if (cp_obs_dist_norm_min <  (ca_sphere_radius_*0.6))
               {
                    joint_vel_bounds[k][0] = 0.0;
                    joint_vel_bounds[k][1] = 0.0;
               }
               else
               {

                    //Compute scaling for current cartesian constraint
                    if(0.0 <= risk_func_vec[k])
                         q_max_scaled = max_joint_velocity_ * (1.0 - risk_func_val);
                    else
                         q_min_scaled = -max_joint_velocity_ * (1.0 - risk_func_val);

                    //Determine minium joint velocity bounds across all cartesian constraints
                    //cout<<"Control Point: "<<endl;
                    //cout<<"Joint: "<<k<<endl;
                    if (q_max_scaled < joint_vel_bounds[k][0])
                    {
                        joint_vel_bounds[k][0] = q_max_scaled;
                        //cout<<"Max Joint Vel lowered"<<endl;
                    }
                    if (fabs(q_min_scaled) < fabs(joint_vel_bounds[k][1]))
                    {
                        joint_vel_bounds[k][1] = q_min_scaled;
                        //cout<<"Min Joint Vel lowered"<<endl;
                    }

                    //cout<<"Joint: "<<k<<endl;
                    //cout<<"Min dist norm: "<<cp_obs_dist_norm_min<<endl;
                    //cout<<"max: "<<joint_vel_bounds[k][0]<<endl;
                    //cout<<"min: "<<joint_vel_bounds[k][1]<<endl;
               }


           }


         }
       //----------- End: Compute degree of influence for current cartesian consraint ------------


   }//end control point iteration


//    for (int k = 0 ; k < num_joints_ ; k++)
//    {
//        cout<<"Joint"<<k<<"  max: "<<joint_vel_bounds[k][0]<<"  min:"<<joint_vel_bounds[k][1]<<endl;
//    }

   //Return joint velocity bounds
   return joint_vel_bounds;
}




//First Order Retraction (according to Paper "Task Constrained Motion Planning in Joint Space", Mike Stilman)
bool RobotController::run_config_retraction(vector<double> &retract_conf, KDL::Frame task_frame, KDL::Frame grasp_transform, vector<int> constraint_vector, vector<pair<double,double> > coordinate_dev, int max_iter)
{

    //Eigen Matrices storing the Jacobian for the chain (6 = dimension of task space)
    Eigen::MatrixXf eigen_current_task_jac(6,num_joints_);

    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_task_jac_pinv;


    // ----- Start: INITIALIZATION ---

    //Current iteration number
    int iter = 0;

    //Norm of current error
    vector<double> curr_task_error = compute_task_error(retract_conf, task_frame, grasp_transform, constraint_vector, coordinate_dev);

    //Compute current error norm
    double curr_error_norm = getVectorNorm(curr_task_error);

    //Integration step_width
    delta_t_ = 0.02;

    //Store initial configuration (before retraction)
    vector<double> retract_conf_initial = retract_conf;


    // ----- End: INITIALIZATION ---


    // ----- Start: First Order Retraction loop ---
    bool error_within_bounds = false;

    //Retract configuration until error_norm is below a certain threshold
    //while(p_error_threshold < curr_error_norm && iter < max_iter)
    while(error_within_bounds == false && iter < max_iter)
    {
        //Compute jacobian for current configuration of body chain
        eigen_current_task_jac = getTaskFrameJacobian(kin_chain_, retract_conf, task_frame);

        //cout<<"Task frame jacobian"<<endl;
        //cout<<eigen_current_task_jac<<endl;


        //Compute the Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
        current_task_jac_pinv = compute_J_vdls(eigen_current_task_jac, min_manip_treshold_, max_damping_factor_);


        //++++++ Compute joint velocity
        KDL::JntArray q_dot_out;
        q_dot_out = KDL::JntArray(num_joints_);
        for (int i = 0; i < num_joints_ ; i++)
        {
            //Init q_dot_out
            q_dot_out(i) = 0.0;
            for (int j = 0; j < 6 ; j++)
            {
                    q_dot_out(i) = q_dot_out(i) + current_task_jac_pinv[i][j] * curr_task_error[j];
            }
            //cout<<q_dot_out(i)<<" ";
        }
        //cout<<endl;


        //++++++ Update Joint Value
        //Current element index
        int curr_idx = 0;
        //Joint Limits check
        bool jl_respected = true;
        for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
        {
            if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
            {
                //Store new joint value in temporary variable
                double new_joint_val = retract_conf[curr_idx] - q_dot_out(curr_idx);// * delta_t_;

                //Check if joint is within joint limits
                if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, false))
                {
                    //Joint Limits are violated
                    jl_respected = false;

                    //Note: Keep fomer joint value, i.e. no update
                    //cout<<"Joint limits violated"<<endl;
                    //cout<<retract_conf[curr_idx]<<endl;

                }
                else
                {
                    //Joint Limits respected
                    jl_respected = true;

                    //Assign new joint value to current joint value
                    retract_conf[curr_idx] = new_joint_val;

                    //cout<<retract_conf[curr_idx]<<endl;
                }

                curr_idx++;
            }
        }
        //When at least one of the joints violates the joint limits
        if(!jl_respected)
             jl_ok_ = false;
         else
             jl_ok_ = true;


        //Update current task error
        curr_task_error = compute_task_error(retract_conf, task_frame, grasp_transform, constraint_vector, coordinate_dev);

        //Update task error norm
        //curr_error_norm = getVectorNorm(curr_task_error);
        error_within_bounds = is_error_within_bounds(curr_task_error);

        //cout<<curr_error_norm<<endl;

        //Increment iteration counter
        iter++;
    }




    // ----- End: First Order Retraction loop ---

    //Compute C-Space distance between initial config and projected sample
    //double projection_dist = compute_euclidean_config_distance(retract_conf_initial, retract_conf);
    //cout<<"Distance rand sample to projeted sample: "<<projection_dist<<endl;


    //Return projection failure if maximum number of iterations has been reached, otherwise return success
    if(iter == max_iter)
    {
        //cout<<"Retraction failed"<<endl;
        return false;
    }
    else
    {
        //cout<<"Retraction succeeded"<<endl;
        return true;
    }

}


vector<double> RobotController::compute_task_error(vector<double> retract_conf, KDL::Frame task_frame, KDL::Frame grasp_transform, vector<int> constraint_vector, vector<pair<double,double> > coordinate_dev)
{
    //Task error to be returned
    vector<double> task_error(6);

    //Invert task frame transform to get world frame expressed in task frame T^t_0
    KDL::Frame inv_task_frame_transform = task_frame.Inverse();


//    cout<<"x t_init: "<<task_frame.p.x()<<endl;
//    cout<<"y t_init: "<<task_frame.p.y()<<endl;
//    cout<<"z t_init: "<<task_frame.p.z()<<endl;

//    cout<<"x t_inv: "<<inv_task_frame_transform.p.x()<<endl;
//    cout<<"y t_inv: "<<inv_task_frame_transform.p.y()<<endl;
//    cout<<"z t_inv: "<<inv_task_frame_transform.p.z()<<endl;

//    double q_x,q_y,q_z,q_w;
//    task_frame.M.GetQuaternion(q_x,q_y,q_z,q_w);

//    vector<double> quat_vector(4);
//    quat_vector[0] = q_w;
//    quat_vector[1] = q_x;
//    quat_vector[2] = q_y;
//    quat_vector[3] = q_z;


//    vector<double> rpy_angles = three_axis_rot( -2.*(quat_vector[2]*quat_vector[3] - quat_vector[0]*quat_vector[1]), \
//                                        quat_vector[0]*quat_vector[0] - quat_vector[1]*quat_vector[1] - quat_vector[2]*quat_vector[2] + quat_vector[3]*quat_vector[3], \
//                                        2*(quat_vector[1]*quat_vector[3] + quat_vector[0]*quat_vector[2]), \
//                                       -2*(quat_vector[1]*quat_vector[2] - quat_vector[0]*quat_vector[3]), \
//                                        quat_vector[0]*quat_vector[0] + quat_vector[1]*quat_vector[1] - quat_vector[2]*quat_vector[2] - quat_vector[3]*quat_vector[3]);

//    cout<<"rotX: "<<rpy_angles[0]<<endl;
//    cout<<"rotY: "<<rpy_angles[1]<<endl;
//    cout<<"rotZ: "<<rpy_angles[2]<<endl;


    //Convert current config to KDL::JntArray
    KDL::JntArray curr_retract_conf = Vector_to_JntArray(retract_conf);

    //Compute current EE pose, i.e. T^0_e' = T^0_e * grasp_transform (transform between EE and task frame in initial configuration)
    // -> that means that the kinematics of the robot is extended by a grasp transform
    KDL::Frame curr_ee_transfom = RobotModel->compute_FK_frame(kin_chain_,curr_retract_conf) * grasp_transform;

//    cout<<"x b: "<<curr_ee_transfom.p.x()<<endl;
//    cout<<"y b: "<<curr_ee_transfom.p.y()<<endl;
//    cout<<"z b: "<<curr_ee_transfom.p.z()<<endl;


    //Compute transformation between task frame and current endeffector frame
    KDL::Frame task_frame_to_ee_transform = inv_task_frame_transform * curr_ee_transfom;


//    cout<<"x t: "<<task_frame_to_ee_transform.p.x()<<endl;
//    cout<<"y t: "<<task_frame_to_ee_transform.p.y()<<endl;
//    cout<<"z t: "<<task_frame_to_ee_transform.p.z()<<endl;

    // ++ Get task error + select only constraint components (others set to zero -> i.e. no error since they're unconstraint)
    vector<double> delta_vec(6);

    //--- Get translation
    delta_vec[0] = task_frame_to_ee_transform.p.x() * constraint_vector[0];
    delta_vec[1] = task_frame_to_ee_transform.p.y() * constraint_vector[1];
    delta_vec[2] = task_frame_to_ee_transform.p.z() * constraint_vector[2];

    //--- Get Rotation as roll,pitch,yaw
    //First get Rotation Quaternion
    double q_x,q_y,q_z,q_w;
    task_frame_to_ee_transform.M.GetQuaternion(q_x,q_y,q_z,q_w);

    //Reverse quaternion order to w,x,y,z (in order to use three_axis_rot function)
    vector<double> quat_vector(4);
    quat_vector[0] = q_w;
    quat_vector[1] = q_x;
    quat_vector[2] = q_y;
    quat_vector[3] = q_z;
    //Compute RPY angles
    vector<double> rpy_angles = three_axis_rot( -2.*(quat_vector[2]*quat_vector[3] - quat_vector[0]*quat_vector[1]), \
                                        quat_vector[0]*quat_vector[0] - quat_vector[1]*quat_vector[1] - quat_vector[2]*quat_vector[2] + quat_vector[3]*quat_vector[3], \
                                        2*(quat_vector[1]*quat_vector[3] + quat_vector[0]*quat_vector[2]), \
                                       -2*(quat_vector[1]*quat_vector[2] - quat_vector[0]*quat_vector[3]), \
                                        quat_vector[0]*quat_vector[0] + quat_vector[1]*quat_vector[1] - quat_vector[2]*quat_vector[2] - quat_vector[3]*quat_vector[3]);

    //Assign roll, pitch, yaw deviation from task frame to delta vector
    delta_vec[3] = rpy_angles[0];
    delta_vec[4] = rpy_angles[1];
    delta_vec[5] = rpy_angles[2];

    //Select error components according to "constraint_vector"
    delta_vec[3] = delta_vec[3] * constraint_vector[3];
    delta_vec[4] = delta_vec[4] * constraint_vector[4];
    delta_vec[5] = delta_vec[5] * constraint_vector[5];


    //cout<<"rotX: "<<rpy_angles[0]*(180/M_PI)<<endl;
    //cout<<"rotY: "<<rpy_angles[1]*(180/M_PI)<<endl;
    //cout<<"rotZ: "<<rpy_angles[2]*(180/M_PI)<<endl;

    //Check whether constraint coordinates are within their permitted deviation boundaries
    for (int i = 0; i < constraint_vector.size(); i++)
    {
        //Is the coordinate constraint?
        if(constraint_vector[i] == 1)
        {
            //Set error to zero if deviation is within tolerance otherwise keep the error
            //  coordinate_dev[i][0] = lower bound and  //  coordinate_dev[i][1] = upper bound
            task_error[i] = (delta_vec[i] < coordinate_dev[i].first || delta_vec[i] > coordinate_dev[i].second) ? delta_vec[i] : 0.0;
        }
        else
        {
            //Coordinate is unconstraints
            //task_error[i] = 0.0;
        }

        //cout<<task_error[i]<<" ";
    }
    //cout<<endl;


    return task_error;
}


//Three axis rotation
vector<double> RobotController::three_axis_rot(double r11, double r12, double r21, double r31, double r32)
{
    vector<double> rot(3);

    //find angles for rotations about X, Y, and Z axes
    rot[0] = atan2( r11, r12 ); //X
    rot[1] = asin( r21 );       //Y
    rot[2] = atan2( r31, r32 ); //Z

    return rot;
}


bool RobotController::is_error_within_bounds(vector<double> task_space_error)
{
    //Flag to be returned
    bool within_bounds = true;

    for (int i = 0; i < task_space_error.size() ; i++)
    {
        //Set flag to false
        if(fabs(task_space_error[i]) > 0.0001)
            within_bounds = false;
    }

    return within_bounds;
}



//Convert XYZ (Roll-Pitch-Yaw) Euler angles to Quaternion
vector<double> RobotController::convertEulertoQuat(double rotX, double rotY, double rotZ)
{
    double sx = sin(rotX/2);
    double sy = sin(rotY/2);
    double sz = sin(rotZ/2);
    double cx = cos(rotX/2);
    double cy = cos(rotY/2);
    double cz = cos(rotZ/2);

    vector<double> quat(4);
    //XYZ rotation sequence
    quat[0] =  sx * cy * cz + sy * sz * cx; //x
    quat[1] =  -sx * sz * cy + sy * cx * cz; //y
    quat[2] =  sx * sy * cz + sz * cx * cy; //z
    quat[3] =  -sx * sy * sz + cx * cy * cz; //w

    //ZYX rotation sequence
//    quat[0] =  sx * cy * cz - cx * sy * sz; //x
//    quat[1] =  cx * sy * cz + sx * cy * sz; //y
//    quat[2] =  cx * cy * sz - sx * sy * cz; //z
//    quat[3] =  cx * cy * cz + sx * sy * sz; //w

    return quat;

}

//Compute quaternion hamilton product
vector<double> RobotController::compute_QuatHProd(vector<double> q1, vector<double> q2)
{
    //q = (x,y,z,w)

    //Compute Hamilton product of quaternions 'q1' and 'q2'.
    vector<double> quat_prod(4);

//    quat_prod[0] = q1[4]*q2[3] + q1[3]*q2[4] + q1[2]*q2[1] - q1[1]*q2[2];  //x
//    quat_prod[1] = q1[4]*q2[2] - q1[3]*q2[1] + q1[2]*q2[4] + q1[1]*q2[3];  //y
//    quat_prod[2] = q1[4]*q2[1] + q1[3]*q2[2] - q1[2]*q2[3] + q1[1]*q2[4]; //z
//    quat_prod[3] = q1[4]*q2[4] - q1[3]*q2[3] - q1[2]*q2[2] - q1[1]*q2[1];  //w

    quat_prod[0] =  q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]; //x
    quat_prod[1] = -q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]; //y
    quat_prod[2] =  q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3] + q1[3] * q2[2]; //z
    quat_prod[3] = -q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] + q1[3] * q2[3]; //w

    //x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    //y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    //z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    //w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;


    return quat_prod;
}

//Compute inverse of quaternion
vector<double> RobotController::compute_QuadInv(vector<double> quat)
{
    vector<double> quat_inv(4);
    quat_inv[0] = -quat[0];
    quat_inv[1] = -quat[1];
    quat_inv[2] = -quat[2];
    quat_inv[3] = quat[3];

    return quat_inv;

}

//Compute norm of a vector
double RobotController::getVectorNorm(vector<double> vec)
{
    double sum_squares = 0.0;
    for(int i = 0 ; i < vec.size() ; i++)
    {
        sum_squares +=  vec[i] *  vec[i];
    }
    double vec_length = sqrt(sum_squares);

    return vec_length;
}


//Compute largest value stored in a vector
double RobotController::getLargestValueFromVector(vector<double> vec)
{
    double val = 0.0;

    for (int i = 0 ; i < vec.size() ; i++)
    {
        if(val<vec[i])
        {
           //Update largest value found
           val = vec[i];
        }
    }

    return val;
}


//Compute euclidean distance between two configurations
double RobotController::compute_euclidean_config_distance(vector<double> conf_start, vector<double> conf_end)
{
    //Euclidean distance to be returned
    double eucl_dist = 0.0;

    //Check if both vectors have the same size
    if(conf_start.size() != conf_end.size())
        ROS_ERROR("Different number of elements in conf_start and conf_end vector!");


    //Distance Vector between two configurations
    vector<double> distance_vec(conf_start.size());

    //Sum of the error square
    double sum_error_sq = 0.0;

//    //Current joint index
//    int j_idx = 0;
//    for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
//    {
//       if(kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
//       {
//           if(kin_chain_.getSegment(j).getJoint().getTypeName() != "TransAxis")
//           {
//               distance_vec[j_idx] = conf_end[j_idx] - conf_start[j_idx];
//               sum_error_sq += (distance_vec[j_idx] * distance_vec[j_idx]);
//           }

//           j_idx++;
//       }
//    }

    //Consider joint space difference
    for (int i = 0 ; i < conf_start.size() ; i++)
    {

        distance_vec[i] = conf_end[i] - conf_start[i];
        sum_error_sq += (distance_vec[i] * distance_vec[i]);
    }

    //Length of error vector
    eucl_dist = sqrt(sum_error_sq);

    //Return euclidean distance
    return eucl_dist;
}


Eigen::MatrixXf RobotController::getTaskFrameJacobian(KDL::Chain kin_chain, vector<double> retract_conf, KDL::Frame task_frame)
{
    //Task frame jacobian to be returned by the function (6 = dimension of task space)
    Eigen::MatrixXf eigen_task_frame_jac(6,num_joints_);

    //Eigen Matrices storing the Jacobian for the chain (6 = dimension of task space)
    Eigen::MatrixXf eigen_current_world_frame_jac(6,num_joints_);

    //Convert current config to KDL::JntArray
    KDL::JntArray curr_retract_conf = Vector_to_JntArray(retract_conf);

    //Get Jacobian w.r.t world frame, i.e. J^0
    eigen_current_world_frame_jac = getJacobian(kin_chain_,curr_retract_conf);


    //Invert task frame transform to get world frame expressed in task frame T^t_0
    KDL::Frame inv_task_frame_transform = task_frame.Inverse();
    //Get Rotation Matrix
    KDL::Rotation rot_world_frame_wrt_task_frame = inv_task_frame_transform.M;
    //Convert KDL Rotation Matrix to Eigen Matrix
    Eigen::MatrixXf rot_wf_wrt_tf(3,3);
    for(int i = 0 ; i < 3 ; i++)
    {
        for(int j = 0 ; j < 3 ; j++)
        {
            rot_wf_wrt_tf(i,j) = rot_world_frame_wrt_task_frame.operator ()(i,j);
        }
    }

    //Construct Eigen Matrix for transformation of jacobian into task frame
    Eigen::MatrixXf trans_jac_task_frame(6,6);
    trans_jac_task_frame.setZero(); //set all entries to zero
    trans_jac_task_frame.block(0,0,3,3) = rot_wf_wrt_tf; //Copy rotation matrix into upper left 3x3 block
    trans_jac_task_frame.block(3,3,3,3) = rot_wf_wrt_tf; //Copy rotation matrix into lower right 3x3 block


    //Compute linear relationship of instantaneous velocities "E(retract_config)" to time derivatives of position and orientation parameters
    double psi = atan2(rot_wf_wrt_tf(2,1),rot_wf_wrt_tf(2,2));
    double theta = -asin(rot_wf_wrt_tf(2,0));
    double phi = atan2(rot_wf_wrt_tf(1,0),rot_wf_wrt_tf(0,0));
    Eigen::MatrixXf angular_matrix(3,3);
    angular_matrix << cos(phi)/cos(theta) , sin(phi)/cos(theta), 0.0, -sin(phi) , cos(phi) , 0.0 , cos(phi)*sin(theta)/cos(theta) , sin(phi)*sin(theta)/cos(theta), 1.0;
    Eigen::MatrixXf identity(3,3);
    identity << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXf matrix_E(6,6);
    matrix_E.setZero();
    matrix_E.block(0,0,3,3) = identity; //Copy identity matrix into upper left 3x3 block
    matrix_E.block(3,3,3,3) = angular_matrix; //Copy angular matrix into lower right 3x3 block

//    cout<<"Matrix E"<<endl;
//    cout<<matrix_E<<endl;
//    cout<<"trans_jac_task_frame"<<endl;
//    cout<<trans_jac_task_frame<<endl;
//    cout<<"World frame jacobian"<<endl;
//    cout<<eigen_current_world_frame_jac<<endl;
//    cout<<"trans_jac_task_frame * World frame jacobian"<<endl;
//    cout<<trans_jac_task_frame * eigen_current_world_frame_jac<<endl;

    //Compute Task Jacobian
    eigen_task_frame_jac = matrix_E * (trans_jac_task_frame * eigen_current_world_frame_jac);


    return eigen_task_frame_jac;
}



//Dynamic Target Tracking controller
bool RobotController::run_J_vdls_Target_Tracking(bool show_motion)
{
    //Eigen Matrices storing the Jacobian for the chain
    Eigen::MatrixXf eigen_current_jac(dim_task_space_,num_joints_);
    //Pseudoinverse of the Jacobian (arm_chain)
    double **current_jac_pinv;
    //Null Space of the Jacobian (arm_chain)
    double **null_space_projector;
    //Value of the JL-Gradient function (to be used in the control loop)
    vector<double> jl_grad_func_val(num_joints_);


    //Variables for variable damping with DLS (based on manipulability analysis)
    min_manip_treshold_ = 0.03;

    //Set sleep duration between configuration
    sleep_duration_between_confs_ = 0.0;


    //Get EE pose
    vector<double> current_ee_pose = RobotModel->compute_FK(kin_chain_,current_config_);

    //Store current endeffector pose
    ee_poses_.push_back(current_ee_pose);


    //Update the error
    update_error_norm(current_ee_pose, current_goal_ee_pose_, false);

    if(error_norm_ < error_treshold_)
    {
       // ROS_INFO("Current target reached!");
    }


    //Compute jacobian for current configuration of body chain
    eigen_current_jac = getJacobian(kin_chain_,current_config_);

    //JL Gradient computation (used in methods using Null Space Projection Method)
    jl_grad_func_val = computeJLgradient(current_config_, q_min_, q_max_, jl_activation_param_);

    //Compute the Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
    current_jac_pinv = compute_J_vdls(eigen_current_jac, min_manip_treshold_, max_damping_factor_);

    //Compute the null space of the pseudoinvere
    null_space_projector = compute_null_space_projection(eigen_current_jac,current_jac_pinv);


    //++++++ Compute joint velocity
    for (int i = 0; i < num_joints_ ; i++)
    {
        //Init q_dot_out
        q_dot_out_(i) = 0.0;

        //Compute first term of control law -> J * error
        for (int j = 0; j < dim_task_space_ ; j++)
        {
                q_dot_out_(i) = q_dot_out_(i) + current_jac_pinv[i][j] * (current_goal_ee_vel_[j] + error_gain_ * error_[j]);
        }


        //Compute term of control law -> (I - J'J) * jl_gradient
        double q_dot_sec_task = 0.0;
        for (int s = 0; s < num_joints_ ; s++)
           q_dot_sec_task = q_dot_sec_task + (null_space_projector[i][s] * jl_grad_func_val[s]);

        //cout<<q_dot_sec_task<<" "<<endl;

        //Add second term to first joint velocoty term
        q_dot_out_(i) = q_dot_out_(i) + gain_sec_task_ * q_dot_sec_task;


        //Weight Joint velocity according to the PD strategy (joint weights are now considered in the jacobian pseudoinversion)
        q_dot_out_(i) = joint_weights_(i) *  q_dot_out_(i);

    }
    //cout<<endl;


    //++++++ Update Joint Value
    //cout<<"New Configuration: "<<endl;
    //Current element index
    int curr_idx = 0;
    //Joint Limits check
    bool jl_respected = true;
    for (int j = 0 ; j < kin_chain_.getNrOfSegments(); j++)
    {
        if (kin_chain_.getSegment(j).getJoint().getTypeName() != "None")
        {
            //Store new joint value in temporary variable
            double new_joint_val = current_config_(curr_idx) + q_dot_out_(curr_idx) * delta_t_;

            //Check if joint is within joint limits
            if(!RobotModel->checkJointLimits(kin_chain_.getSegment(j).getJoint().getName(),new_joint_val, true))
            {
                //Joint Limits are violated
                jl_respected = false;

                //Note: Keep fomer joint value, i.e. no update

                cout<<"Joint: "<<curr_idx<<" violates its joint limits!"<<endl;

                //Lower joint value bound violated
                //if (new_joint_val < q_min_[curr_idx])
                //    current_config_(curr_idx) = q_min_[curr_idx] + 0.05; //0.05 rad = 3 degree distance to joint range border
                //Upper joint value bound violated
                //if (q_max_[curr_idx] < new_joint_val)
                //    current_config_(curr_idx) = q_max_[curr_idx] - 0.05; //0.05 rad = 3 degree distance to joint range border
            }
            else
            {
                //Joint Limits respected
                jl_respected = true;

                //Assign new joint value to current joint value
                current_config_(curr_idx) = new_joint_val;
            }

            //cout<<"New value for joint: "<<kin_chain_.getSegment(j).getJoint().getName()<<" is: "<<current_config_(curr_idx)<<"   "<<endl;

            //Set Configuration to be shown by "show_ik_intermediate_config"-function
            nvalues_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

            //Set map entry
            conf_[kin_chain_.getSegment(j).getJoint().getName()] = current_config_(curr_idx);

            curr_idx++;
        }
    }
    //When at least one of the joints violates the joint limits
    if(!jl_respected)
         jl_ok_ = false;
     else
         jl_ok_ = true;


    //++++++ Show Motion
    if(show_motion == 1)
        show_ik_intermediate_config(sleep_duration_between_confs_);



    //Show endeffector trace
    //if(show_ee_trace == true)
    //    RobotModel->show_ee_trace(current_ee_pose,color_rgb_,max_ee_trace_points_);


    //"cout<<"Current error: "<<endl;
    //cout<<"X: "<<error_[0]<<endl;
    //cout<<"Y: "<<error_[1]<<endl;
    //cout<<"Z: "<<error_[2]<<endl;
    //cout<<"rotX: "<<error_[3]<<endl;
    //cout<<"rotY: "<<error_[4]<<endl;
    //cout<<"rotZ: "<<error_[5]<<endl;



    //Delete endeffector trace (for next ee trajectory)
    //RobotModel->delete_ee_trace();


    return true;
}



} //end of namespace
 
 
