/*
 * control_laws.h
 *
 *  Created on: March 30, 2015
 *      Author: Felix Burget
 */

#include <ros/ros.h>

#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <Eigen/SVD>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>  // for some reason there are problems with Eigen library if this header is not included

#include <kuka_motion_control/kdl_kuka_model.h>

#include <../gnuplot-iostream/gnuplot-iostream.h>


#ifndef CONTROL_LAWS_H
#define CONTROL_LAWS_H

using namespace std;



namespace kuka_motion_controller{

//Status flag to be returned by controller
// -> ADVANCED: Controller generated a motion until error change rate dropped below a threshold (goal has not been reached)
// -> REACHED: Controller generated a motion to the goal (final error below a given threshold)
// -> TRAPPED: Controller stuck in a certain EE pose, but error change rate is still above some threshold -> should not happen, modify/increase error change rate threshold
enum Status { ADVANCED, REACHED, TRAPPED};


class RobotController
{
    public:
    RobotController(string robot_desciption_param, string kinematic_chain, string ns_prefix = "");
    RobotController(boost::shared_ptr<kuka_motion_controller::KDLRobotModel> kdl_robot_model, string robot_desciption_param, string kinematic_chain, string ns_prefix= "");
    ~RobotController();

    //Compute the forward kinematics for a kinematic chain
    //std::vector<double> compute_FK(KDL::Chain kin_chain, KDL::JntArray chain_config);

    //Sets the start configuration and the cartesian endeffector goal based on the first and last entry of the joint trajectory file
    void init(char* start_conf_filename, char* ee_traj_filename, char* ee_vel_traj_filename);

    //Set Planning Scene Service Name
    void setPlanningSceneServiceName(string service_name);

    //Get random EE pose
    vector<double> getRandEEPose();

    //Set Random Start Configuration
    void setRandomStartConf();

    //Get Random Configuration (by uniform sampling in the joint range)
    KDL::JntArray getRandomConf(double env_size_x, double env_size_y);
    KDL::JntArray getRandomConf(vector<double> env_size_x, vector<double> env_size_y);

    //Get Random Configuration (by gaussian sampling around a mean configuration vector)
    KDL::JntArray getRandomConf(vector<double> mean_config, double std_dev, double env_size_x, double env_size_y);
    KDL::JntArray getRandomConf(vector<double> mean_config, double std_dev, vector<double> env_size_x, vector<double> env_size_y);


    //Set a specific Start Configuration as map
    void setStartConf(map<string, double> start_config);

    //Set a specific Start Configuration as double vector
    void setStartConf(vector<double> start_config);

    //Set Variable Constraint Vector (constraint variables and don't care variables)
    // -> Considered in error computation of control loop
    void setVariableConstraints(vector<int> var_constraint_vec, vector<pair<double, double> > var_coordinate_dev);

    //Read first config of the motion segments joint trajectory (of the subject)
    map<string, double>  readStartConfig(char* joint_traj_filename);

    //Read joint trajectory (of the human subject and motion segment or the control based generated trajectory)
    map<string, vector<double> > readJointTrajectory(char* joint_traj_filename);

    //Read endeffector trajectory of the motion segments (of the human subject)
    void readEEposTrajectory(char* ee_traj_filename);

    //Read endeffector velocity trajectory of the motion segments (of the human subject)
    void readEEvelTrajectory(char* ee_vel_traj_filename);


    //Get the Jacobian for a given chain and configuration
    Eigen::MatrixXf getJacobian(KDL::Chain kin_chain, KDL::JntArray config);

    //Compute the pseudoinverse of the jacobian matrix
    double **compute_J_pinv(Eigen::MatrixXf jacobian);

    //Compute the Damped-least squares jacobian matrix with constant damping factor
    double **compute_J_dls(Eigen::MatrixXf jacobian, float damping_factor);

    //Compute the Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
    double **compute_J_vdls(Eigen::MatrixXf jacobian, float min_manip_treshold, float max_damping_factor);

    //Compute the Weighted Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
    double **compute_J_wvdls(Eigen::MatrixXf jacobian, float min_manip_treshold, float max_damping_factor, std::vector<double> previous_jl_grad_func_val, std::vector<double> jl_grad_func_val);

    //Compute the Weighted Least-Norm jacobian matrix
    double **compute_J_wln(Eigen::MatrixXf jacobian, std::vector<double> previous_jl_grad_func_val, std::vector<double> jl_grad_func_val);

    //Compute the transpose of the jacobian matrix
    double **compute_J_trans(Eigen::MatrixXf jacobian, std::vector<double> error);

    //Compute Nakamura's Manipulability Measure
    double computeManipulabilityMeasure(Eigen::MatrixXf jacobian);

    //Compute the Null Space of the jacobian
    double **compute_null_space_projection(Eigen::MatrixXf jacobian, double **jacobian_inverse);

    //Compute the joint limit gradient (following Chaumette & Marchand "A redundancy based iterative approach for avoiding joint limits: Application to visual servoing")
    std::vector<double> computeJLgradient(KDL::JntArray config, std::vector<double> q_min, std::vector<double> q_max, double activation_param);

    //Compute the Inverse of a Matrix
    double **compute_inverse(double **matrix, int num_rows, int num_cols);

    //Determinant of a Matrix
    double compute_determinant(double **matrix, int num_rows, int num_cols);

    //Compute Adjugate of a Matrix
    double** compute_adjugate(double **matrix, int num_rows, int num_cols);

    //Convert JntArray to std::Vector
    vector<double> JntArray_to_Vector(KDL::JntArray jnt_array);

    //Convert std::Vector to JntArray
    KDL::JntArray Vector_to_JntArray(vector<double> vec);

    //Compute norm of a vector
    double getVectorNorm(vector<double> vec);

    //Compute largest value stored in a vector
    double getLargestValueFromVector(vector<double> vec);

    //Three axis rotation
    vector<double> three_axis_rot(double r11, double r12, double r21, double r31, double r32);

    //Compute euclidean distance between two configurations
    double compute_euclidean_config_distance(vector<double> conf_start, vector<double> conf_end);

    //Convert  ZYX (Yaw-Pitch-Roll) Euler angles to Quaternion
    vector<double> convertEulertoQuat(double rotX, double rotY, double rotZ);

    //Compute quaternion hamilton product
    vector<double> compute_QuatHProd(vector<double> q1, vector<double> q2);

    //Compute inverse of quaternion
    vector<double> compute_QuadInv(vector<double> quat);

    //Function to select the best control strategy for the current configuration
    int control_strategy_selection(KDL::Jacobian current_jacobian);

    //--------------- Control Approaches ------------

    //Set endeffector goal pose with file path to store trajectory data
    void set_EE_goal_pose(vector<double> ee_pose, char *traj_file_name);

    //Set endeffector goal pose
    void set_EE_goal_pose(vector<double> ee_pose);

    //Set Motor Control Strategy
    void set_motion_strategy(int strategy_index);

    //Functions for joint weight adapting based on PD strategy
    //void adapt_joint_weights(int joint_num);
    //void adapt_joint_weights();

    //Show intermediate configuration of IK loop
    void show_ik_intermediate_config(double sleep_duration);

    //Set of different control approaches (using the current joint weights)
    bool run_J_pinv_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data);
    bool run_J_dls_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data);
    bool run_J_vdls_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data);
    Status run_VDLS_Control_Connector(int max_iter, vector<vector<double> > &joint_traj, vector<vector<double> > &ee_traj, bool show_motion, bool show_ee_trace);
    bool run_J_wvdls_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data);
    bool run_J_wln_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data);
    bool run_J_trans_Controller(int max_iter, int show_motion, bool show_ee_trace, bool store_traj_data);

    //Dynamic Target Tracking controller
    bool run_J_vdls_Target_Tracking(bool show_motion);

    //First Order Retraction
    bool run_config_retraction(vector<double> &retract_conf, KDL::Frame task_frame, KDL::Frame grasp_transform, vector<int> constraint_vector, vector<pair<double,double> > coordinate_dev, int max_iter);
    vector<double> compute_task_error(vector<double> retract_conf, KDL::Frame task_frame, KDL::Frame grasp_transform, vector<int> constraint_vector, vector<pair<double,double> > coordinate_dev);
    //Error within bounds check (with bounds fixed or given as input)
    bool is_error_within_bounds(vector<double> task_space_error);
    Eigen::MatrixXf getTaskFrameJacobian(KDL::Chain kin_chain, vector<double> retract_conf, KDL::Frame task_frame);

    //Learn human joint trajectory by adapting the joint weights used in the controller
    bool learn_joint_trajectory(int control_strategy, int max_ik_iter, int max_learning_iter, double mean_joint_traj_error_threshold, int show_motion, bool show_ee_trace, bool store_traj_data);


    //Update the error and its norm(pos+orientation)
    void update_error_norm(vector<double> curr_ee_pose, vector<double> des_ee_pose, bool error_clamping);
    //Update the error vector(pos+orientation)
    void update_error_vec(vector<double> curr_ee_pose, vector<double> des_ee_pose);


    //Update Joint weights based on error between human and control joint trajectory (used for learning)
    bool update_joint_weights(int j_num, double mean_joint_traj_error_threshold);

    //Clamping the magnitude of the error
    void clamp_error_magnitude(double error_norm_upper_bound);

    //Write the final/learned joint weights to file
    void writeJointWeights(KDL::JntArray joint_weights, char* file_path);

    //Load the learned joint weights from file
    bool load_joint_weights(char* file_path);


    //Functions to get Joint and Endeffector Trajectory generated by the controller
    vector< map<string, double> > getJointTrajectory();
    vector< vector<double> > getEETrajectory();


    //--------------- Collision Avoidance Computation ------------

    //Select control points at the following joints
    vector<int> joint_indices_for_CA_;

    //Compute Control Points (Position of Spheres located alon the endeffector chain, subsequently used for collision avoidance)
    vector<vector<double> > compute_CA_ControlPoints(vector<int> joint_indices);

    //Compute Repulsive vector (for obstacle avoidance)
    vector<double> compute_EE_RepulsiveVector(vector<double> ee_position);

    //Distance from control points to obstacle
    vector<double> compute_Dist_to_Obs(vector<double> pos_control_point,vector<double> obstacle_pos);

    //Compute Magnitude of repulsive vector
    double compute_Pot_Field_Magnitude(double cp_obs_dist_norm);

    //Add an obstacle to be considered for collision avoidance
    void add_Collision_Obstacle(vector<double> object_data);

    //Add base platform as obstacle to be considered for collision avoidance
    void add_Base_Platform_Obstacle();


    //Compute risk of collision function vector(for robot body collision avoidance)
    vector<vector<double> > compute_risk_func_vector(vector<vector<double> > control_points_pos, Eigen::MatrixXf jacobian);



    private:


    //Node Handle
    ros::NodeHandle nh;

    // Path to package (to be read from param server)
    std::string package_path_;

    //Name of Planning Scene Service;
    string m_planning_scene_service;

    //Generate KDL tree from Kuka urdf
    boost::shared_ptr<KDLRobotModel> RobotModel;
    //motion_strategies::KDLRobotModel RobotModel;

    //PD Motion Strategy
    int pd_motion_strategy_;

    //Name of the kinematic chain
    string kinematic_goup_;

    //Data for a custom kinematic chain
    KDL::Chain kin_chain_;
    vector<string> joint_names_;
    int num_joints_;                         //Num Joints of the chain
    vector<double> q_min_, q_max_, opt_pos_; //Joint range data


    //Current target pose for endeffector
    vector<double> current_goal_ee_pose_;

    //Current target velocity for endeffector
    vector<double> current_goal_ee_vel_;


    //File containing start Configuration of Robot
    char* file_path_start_config_;
    map<string, vector<double> > human_joint_trajectory_;
    int num_configs_; //number of configurations in the human joint trajectory = number of control based generated configs

    //Cartesian Pose Trajectory to be tracked by the endeffector
    char* file_path_ee_pos_trajectory_;
    double **ee_ref_pos_trajectory_;
    int num_ee_poses_;
    bool traj_tracking_enabled_;

    //Cartesian Velocity Trajectory to be tracked by the endeffector
    char* file_path_ee_vel_trajectory_;
    double **ee_ref_vel_trajectory_;
    int num_ee_vels_;



    //Current chain configuration
    KDL::JntArray current_config_;

    //Dimension of task space
    int dim_task_space_;

    //Control Parameters (for all control approaches)
    KDL::JntArray joint_weights_;   //Joint weights (depend on PD Strategy)
    double min_change_rate_;        //Threshold (i.e. minimal joint value change rate) used for joint activation or switching (for PD_MOTION_STRATEGY)
    int index_last_joint_activated_; //Index (in config vector) of last joint activated
    double delta_t_;                //Time Step (time to apply the computed velocity q_dot)
    float gain_sec_task_;           //Choose gain for secondary task (here joint limit avoidance)
    bool jl_ok_;                    //Flag indicating whether joint values are within their admissible range
    double error_gain_;             //Gain for the error (often referred to as K in the literature)

    double jl_activation_param_; //Activation parameter (for shaping JL Gradient function used in Null Space Projection Methods, must be in the range 0 < activation_param < 0.5)
    float const_lambda_fac_;  //Constant Damping factor for damped-least squares method
    float min_manip_treshold_; //Variables for variable damping with DLS (based on manipulability analysis)
    float max_damping_factor_;
    double error_treshold_; //Treshold for error


    //Threshold (i.e. minimal joint value change rate) used for joint activation or switching (for PD_MOTION_STRATEGY = 2 or 3)
    //double min_change_rate =   0.008;//0.0175;//0.0349; //in rad per time step  0.0349 = 2 degrees

    //Index (in config vector) of last joint activated
    //int index_last_joint_activated = 0;


    //Error and its norm
    vector<double> error_;      //cartesian error of ee pose
    double error_norm_;         //Error norm

    //Permitted devation from target frame
    //Permitted deviation for constrained coordinates
    // -> specifies an lower and upper deviation boundary
    vector<pair<double,double> > coordinate_dev_;

    //Variables Constraint Vector (defines constraint variables of pose vector)
    vector<int> var_constraint_vec_;


    //Current joint velocity
    KDL::JntArray q_dot_out_;

    //Random number generator to generate a random initial configuration within joint limits
    random_numbers::RandomNumberGenerator  random_number_generator_;


    //Map containing joint names and values. Used for visualization in Planning Scene
    //boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;

    ros::Publisher scene_pub_;
    map<string, double> nvalues_;
    double sleep_duration_between_confs_; //Sleep duration between showing configs

    //Maximum number of points for the endeffector trajectory trace
    int max_ee_trace_points_;


    //Vectors storing joint trajectory data to be written to a file
    vector< map<string, double> > configurations_;
    map<string, double> conf_;
    map<string, double> w_joints_;
    char* file_path_joint_trajectory_;


    //Vectors storing endeffector trajectory data to be written to a file
    vector< vector<double> > ee_poses_;
    vector<double> color_rgb_; //color of ee trace (if shown)
    char* file_path_endeffector_trajectory_;

    //Marker for obstacle representation
    ros::Publisher obstacle_pub_;

    // ----------------------- Learning joint weights --------------------
    //Flag indicating when joint weight gradient changes direction (in that case the joint weight update rate is lowered)
    int joint_weight_gradient_direction_; //0->joint_weight_increased, 1->joint_weight_decreased, 2->init

    //Maximum value for joint weight change
    vector<double> max_joint_weight_modification_;

    //Store mean trajectory error using the joint weights of the previous iteration
    double previous_mean_joint_trajectory_error_;

    //Path for the file storing the final/learned joint weights
    char* file_path_joint_weights_;



    //----------------------- Collision Avoidance --------------------
    //Radius of spheres used as safety margin around control points and to compute potential field
    double ca_sphere_radius_;
    //Shape factor to influence behavior of repulsive vector magnitude
    double ca_alpha_shape_fac_;
    //Maximum magnitude of repulsive vector
    double max_mag_rep_vec_;

    //Data of obstacles in the scene (pos + size)
    vector< vector<double> > scene_obstacles_;

    //Obstacle Marker array
    visualization_msgs::Marker obstacle_array_;

    //Flag indicating whether collision avoidance is active
    bool collision_avoidance_active_;

    //Marker for table representation
    ros::Publisher base_pub_;

    //Parameter used for Joint velocity bounds shaping for robot body collision avoidance
    double max_joint_velocity_;


    //----------------------- For plotting with Gnuplot --------------------
    //Gnuplot gnu_plot_;

};

}//end of namespace

#endif // CONTROL_LAWS_H
 
