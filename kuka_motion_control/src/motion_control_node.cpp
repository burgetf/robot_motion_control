/*
 *motion_control_node.cpp
 *
 *  Created on: March 30, 2015
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <kuka_motion_control//kdl_kuka_model.h>
#include <kuka_motion_control/control_laws.h>


using namespace std;

int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "motion_control_node");


    //Node Handle
    ros::NodeHandle nh;


    //Set default values
    int CONTROL_STRATEGY = 1;
    int MOTION_STRATEGY = 0;
    int SHOW_MOTION = 0;
    bool SHOW_EE_TRACE = 0;
    bool STORE_TRAJ_DATA = 0;
    string TRAJ_NAME = "test_trajectory";
    string CHAIN = "kuka_complete_arm";


    if(argc > 1) {
      stringstream s(argv[1]);
      s >> TRAJ_NAME;
    }

    if(argc > 2) {
      stringstream s(argv[2]);
      s >> CONTROL_STRATEGY;
    }

    if(argc > 3) {
      stringstream s(argv[3]);
      s >> MOTION_STRATEGY;
    }

    if(argc > 4) {
      stringstream s(argv[4]);
      s >> SHOW_MOTION;
    }

    if(argc > 5) {
      stringstream s(argv[5]);
      s >> SHOW_EE_TRACE;
    }

    if(argc > 6) {
      stringstream s(argv[6]);
      s >> STORE_TRAJ_DATA;
    }



    // START: FILES TO READ
    cout<<endl;

    cout<<"------------------------------- SETUP ------------------------------------"<<endl;


    //Subject name
    cout << "Trajectory Name: "<<TRAJ_NAME<<endl;
    cout<<endl;

    //Read package path from parameter server
    //std::string package_path;
    //nh.param("package_path", package_path, std::string("/home/burgetf/catkin_ws/src/robot_motion_control/kuka_motion_control"));

    //Get package path of "kuka_motion_control"
    std::string package_path;
    package_path = ros::package::getPath("kuka_motion_control");


    //Set path to the file containing the robot start configuration
    string folder_path = package_path + "/trajectories/start_configurations/";
    const char* joint_trajectory_file_1 = folder_path.c_str();
    const char* joint_trajectory_file_2 = "_start_conf.txt";
    string tmp = joint_trajectory_file_1 + TRAJ_NAME + joint_trajectory_file_2;
    char* start_conf_file = new char[tmp.size() + 1];
    copy(tmp.begin(), tmp.end(), start_conf_file);
    start_conf_file[tmp.size()] = '\0'; // don't forget the terminating 0
    cout<<"Path to start configuration file:"<<endl<<start_conf_file<<endl;

    //Set path to the file containing the ee POSITION trajectory
    folder_path = package_path + "/trajectories/ee_pos_trajectories/";
    const char* ee_trajectory_file_1 = folder_path.c_str();
    const char* ee_trajectory_file_2 = "_ee_trajectory.txt";
    string tmp2 = ee_trajectory_file_1 + TRAJ_NAME + ee_trajectory_file_2;
    char* ee_traj_file = new char[tmp2.size() + 1];
    copy(tmp2.begin(), tmp2.end(), ee_traj_file);
    ee_traj_file[tmp2.size()] = '\0'; // don't forget the terminating 0
    cout<<"Path to endeffector pose trajectory file:"<<endl<<ee_traj_file<<endl;

    //Set path to the file containing the ee VELOCITY trajectory
    folder_path = package_path + "/trajectories/ee_vel_trajectories/";
    const char* ee_vel_trajectory_file_1 = folder_path.c_str();
    const char* ee_vel_trajectory_file_2 = "_ee_vel_trajectory.txt";
    string tmp3 = ee_vel_trajectory_file_1 + TRAJ_NAME + ee_vel_trajectory_file_2;
    char* ee_vel_traj_file = new char[tmp3.size() + 1];
    copy(tmp3.begin(), tmp3.end(), ee_vel_traj_file);
    ee_vel_traj_file[tmp3.size()] = '\0'; // don't forget the terminating 0
    cout<<"Path to endeffector velocity trajectory file:"<<endl<<ee_vel_traj_file<<endl;

    cout<<endl;
    // END: FILES TO READ




    // START: CONTROL SETUP
    cout<<endl;

    switch (CONTROL_STRATEGY)
    {
        case 0: cout << "Control Approach: Jacobian Pseudoinverse Method "<<endl;
            break;
        case 1: cout << "Control Approach: Jacobian Damped Least Squares Method "<<endl;
            break;
        case 2: cout << "Control Approach: Jacobian Variable Damped Least Squares Method"<<endl;
            break;
        case 3: cout << "Control Approach: Jacobian Weighted Variable Damped Least Squares Method"<<endl;
            break;
        case 4: cout << "Control Approach: Jacobian Weighted Least-Norm Method"<<endl;
            break;
        case 5: cout << "Control Approach: Jacobian Transpose Method"<<endl;
            break;
        //default: cout << "Default Control Approach: Jacobian Pseudoinverse Method "<<endl;
        //         CONTROL_STRATEGY = 0;
        //         break;
    }

    switch (MOTION_STRATEGY)
    {
        case 0: cout << "Motion Strategy: All Joints weighted equally"<<endl;
            break;
        case 1: cout << "Motion Strategy: Joint weighted by factor 0.5 along the chain"<<endl;
            break;
        case 2: cout << "Motion Strategy: Arm motion preference (lower weights for spine joints)"<<endl;
            break;
        case 3: cout << "Motion Strategy: Spine motion preference (lower weights for arm joints)"<<endl;
            break;
        case 4: cout << "Motion Strategy: Turn of one joint"<<endl;
            break;
        case 5: cout << "Motion Strategy: Use learned joint weights if available"<<endl;
            break;
        default: cout << "Default Motion Strategy: All Joints weighted equally or use learned joints if available"<<endl;
                 MOTION_STRATEGY = 0;
                 break;
    }


    switch (SHOW_MOTION)
    {
        case 0: cout << "Show motion: deactivated "<<endl;
            break;
        case 1: cout << "Show motion: activated"<<endl;
            break;
        default: cout << "Default Show motion: deactivated"<<endl;
                 SHOW_MOTION = 0;
                 break;
    }


    switch (SHOW_EE_TRACE)
    {
        case 0: cout << "Show endeffector trace: deactivated" <<endl;
            break;
        case 1: cout << "Show endeffector trace: activated"<<endl;
            break;
        default: cout << "Default Show endeffector trace: deactivated"<<endl;
                 SHOW_EE_TRACE = 0;
                 break;
    }


    switch (STORE_TRAJ_DATA)
    {
        case 0: cout << "Store trajectory data: deactivated" <<endl;
            break;
        case 1: cout << "Store trajectory data: activated"<<endl;
            break;
        default: cout << "Default Store trajectory data: deactivated"<<endl;
                 STORE_TRAJ_DATA = 0;
                 break;
    }

    cout<<"--------------------------------------------------------------------------"<<endl<<endl;

    // END: CONTROL SETUP





    //Create RobotController Object for Motion Control of the robot
    kuka_motion_controller::RobotController RobotControl("robot_description", CHAIN);


    //++++++++++++ OPTION 1: Load start and goal configuration (goal ee pose) from file ++++++++++
    //Sets the start configuration and the cartesian endeffector goal based on the first and last entry of the joint trajectory file
    RobotControl.init(start_conf_file, ee_traj_file, ee_vel_traj_file);




    //++++++++++++ OPTION 2: Random start configuration and desired goal ee pose selected manually++++++++++
    //Set a goal position and orientation w.r.t base frame of your chain
//    vector<double> goal_ee_pose(7); //(x,y,z, quatX, quatY, quatZ, quatW)
//    goal_ee_pose[0] = -0.108012;
//    goal_ee_pose[1] = 0.59835;
//    goal_ee_pose[2] = 0.215184;
//    goal_ee_pose[3] = 1000.0;
//    goal_ee_pose[4] = 1000.0;
//    goal_ee_pose[5] = 1000.0;
//    goal_ee_pose[6] = 1000.0;
//    RobotControl.set_EE_goal_pose(goal_ee_pose,ee_traj_file);


    //++++++++++++ OPTION 3: Set start configuration and desired goal ee pose selected manually++++++++++
    //Set a start configuration for the chain
//    vector<double> start_conf(7); //(x,y,z, quatX, quatY, quatZ, quatW)
//    start_conf[0] = 0.3;
//    start_conf[1] = 0.77;
//    start_conf[2] = -1.45;
//    start_conf[3] = -0.80;
//    start_conf[4] = -0.74;
//    start_conf[5] = -0.61;
//    start_conf[6] = 0.74;
//    RobotControl.setStartConf(start_conf);

//    //Set a goal pose for the endeffector
//    vector<double> goal_ee_pose(7); //(x,y,z, quatX, quatY, quatZ, quatW)
//    goal_ee_pose[0] = -0.108012;
//    goal_ee_pose[1] = 0.59835;
//    goal_ee_pose[2] = 0.215184;
//    goal_ee_pose[3] = 1000.0;
//    goal_ee_pose[4] = 1000.0;
//    goal_ee_pose[5] = 1000.0;
//    goal_ee_pose[6] = 1000.0;
//    RobotControl.set_EE_goal_pose(goal_ee_pose);



     //+++++++ CONTROL PARAMETER SETTINGS
     //Maximum IK iterations admissible
     int max_iter = 5000;

     if (SHOW_MOTION == 0 && SHOW_EE_TRACE == 0)
        max_iter = 8000;


     //----- Add collision avoidance obstacles

     //Add base platform as obstacle
     RobotControl.add_Base_Platform_Obstacle();

     //Add other obstacles
     vector<double> obstacles;
     obstacles.push_back(-0.34919);
     obstacles.push_back(0.372848);
     obstacles.push_back(0.442304); //0.482304
     obstacles.push_back(0.12); //Radius of obstacle
     RobotControl.add_Collision_Obstacle(obstacles);
     vector<double> obstacles2;
     obstacles2.push_back(0.34919);
     obstacles2.push_back(0.372848);
     obstacles2.push_back(0.442304); //0.482304
     obstacles2.push_back(0.12); //Radius of obstacle
     RobotControl.add_Collision_Obstacle(obstacles2);


     //Set Motion Strategy
     RobotControl.set_motion_strategy(MOTION_STRATEGY);


     //+++++++ RUN CONTROLLER
     //For the control you can use of the following control schemes
     cout<<"Entering loop:"<<endl;

     //Pseudoinverse (unstable in the neighbourhood of singularities)  Null Space Optimization (for joint limit avoidance)
     if (CONTROL_STRATEGY == 0)
     {
          RobotControl.run_J_pinv_Controller(max_iter,SHOW_MOTION,SHOW_EE_TRACE,STORE_TRAJ_DATA);
     }

     //Damped-least squares Pseudoinverse (stable in the neighbourhood of singularities)  Null Space Optimization (for joint limit avoidance)
     if (CONTROL_STRATEGY == 1)
     {
          RobotControl.run_J_dls_Controller(max_iter,SHOW_MOTION,SHOW_EE_TRACE,STORE_TRAJ_DATA);
     }

     //Compute the Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
     if (CONTROL_STRATEGY == 2)
     {
          RobotControl.run_J_vdls_Controller(max_iter,SHOW_MOTION,SHOW_EE_TRACE,STORE_TRAJ_DATA);

     }

     //Compute the Weighted Damped-least squares jacobian matrix with variable damping factor (depending on Nakamura's Manipulability measure)
     if (CONTROL_STRATEGY == 3)
     {
          RobotControl.run_J_wvdls_Controller(max_iter,SHOW_MOTION,SHOW_EE_TRACE,STORE_TRAJ_DATA);

     }

     //Weighted Least-Norm Pseudoinverse (avoids joint limits, but unstable in the neighbourhood of singularities)
     if (CONTROL_STRATEGY == 4)
     {
          RobotControl.run_J_wln_Controller(max_iter,SHOW_MOTION,SHOW_EE_TRACE,STORE_TRAJ_DATA);

     }

     //Jacobian Transpose (no joint limit avoidance, unstable in the neighbourhood of singularities)
     if (CONTROL_STRATEGY == 5)
     {
          RobotControl.run_J_trans_Controller(max_iter,SHOW_MOTION,SHOW_EE_TRACE,STORE_TRAJ_DATA);
     }




  ros::spin();

  return 0;
}

