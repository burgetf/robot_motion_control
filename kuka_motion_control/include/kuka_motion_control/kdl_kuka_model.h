/*
 * kdl_kuka_model.h
 *
 *  Created on: March 30, 2015
 *      Author: Felix Burget
 */


#include <ros/ros.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>


//#include <kdl/chainiksolverpos_nr.hpp>
//#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/chainiksolvervel_pinv_nso.hpp>
//#include <kdl/chainjnttojacsolver.hpp>

//#include <kdl/utilities/svd_eigen_HH.hpp>
//#include <Eigen/SVD>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <srdfdom/model.h>

#ifndef KDL_KUKA_MODEL_H_
#define KDL_KUKA_MODEL_H_

using namespace std;

namespace kuka_motion_controller{


//Structure representing joint limits
struct JointLimits{
    vector<string> joint_names;
    vector<double> q_min;
    vector<double> q_max;
};


class KDLRobotModel
{
  public:
  KDLRobotModel(string robot_desciption_param, string planning_scene_topic, string ee_trajectory_topic, string chain);
  ~KDLRobotModel();


  //Get the Arm Kinematic Chains
  KDL::Chain getCompleteArmChain();

  //Get joint names of the kinematic chain
  vector<string> getJointNames();

  //Get number of joints of the kinematic chain
  int getNumJoints();           //total number
  int getNumRevoluteJoints();   //number of revolute joints
  int getNumPrismaticJoints();  //number of prismatic joints


  //Get Base Link Frame Name
  string getBaseLinkName(){return base_link_name_;}

  //Compute the forward kinematics for a kinematic chain (returning the pose as a vector)
  std::vector<double> compute_FK(KDL::Chain kin_chain, KDL::JntArray chain_config);

  //Compute the forward kinematics for a kinematic chain (returning the pose as a frame)
  KDL::Frame compute_FK_frame(KDL::Chain kin_chain, KDL::JntArray chain_config);

  //Compute the forward kinematics for a kinematic chain (for a specific segment)
  std::vector<double> compute_FK_Segment(KDL::Chain kin_chain, KDL::JntArray chain_config, int segment);

  //Build a custom chain from a root to a tip link of the robot
  bool getCustomChain(const string &chain_root, const string &chain_tip, KDL::Chain &custom_chain);

  //Add a virtual tool center point (TCP) to the robots endeffector
  void addTCP(string name, KDL::Frame ee_tcp_transform, KDL::Chain &kin_chain);

  //Add a virtual base joint to a kinematic chain
  void addVirtualBaseJoint(KDL::Chain &kin_chain, KDL::Joint base_joint, double q_min, double q_max, KDL::Frame transform);

  //Provides the min, max (Joint limits) and center value for all joint contained in a given kinematic chain
  void getJointRangeData(KDL::Chain kin_chain, vector<double>& q_min, vector<double>& q_max , vector<double>& q_center);

  //Check if a given joint respects it's value range
  bool checkJointLimits(string joint_name, double q_val, bool show_output);

  //Get the minimal distance of a joint value to  the joint range limits
  double getDistanceToJointLimits(string joint_name, double q_val);

  //Show configuration of kinematic chain(s) in planning scene
  bool showConfig(vector<KDL::Chain> kin_chains, vector<KDL::JntArray> config_chains, double sleep_duration);

  //Show endeffector trace in planning scene (RViz) with limited number of trace points
  void show_ee_trace(vector<double> curr_pose_vec, vector<double> color_rgb, int max_traj_points);

  //Show endeffector trace in planning scene (RViz) with unlimited number of trace points
  void show_ee_trace(vector<double> curr_pose_vec, vector<double> color_rgb);

  //Delete endeffector trace points, i.e. empty the array of points
  void delete_ee_trace();

  //Show control points used for collision avoidance
  void show_CA_Control_Points(vector< vector<double> > ca_control_points);

  //Write an arbitrary sequence of vectors to a file (whatever it is)
  void writeTrajectoryToFile(vector< vector<double> > configurations, char *motion_file);

  //Write a sequence of configurations to a file
  void writeJointTrajectoryToFile(vector< map<string, double> > configurations, map<string, double> joint_weights, char *motion_file);

  //Write a sequence of joint velocities to a file
  void writeJointVelTrajectoryToFile(vector<vector<double> > joint_velos, char *motion_file);

  //Write joint acceleration trajectory
  void writeJointAccTrajectoryToFile(vector<vector<double> > joint_accs, char *motion_file);

  //Write joint jerk trajectory
  void writeJointJerkTrajectoryToFile(vector<vector<double> > joint_jerks, char *motion_file);

  //Write a sequence of endeffector poses to a file
  void writeEndeffectorTrajectoryToFile(vector<vector<double> > ee_poses, char *motion_file);

  //Write a sequence of endeffector velocities to a file
  void writeEndeffectorVelTrajectoryToFile(vector<vector<double> > ee_velos, char *motion_file);

  //Write a sequence of endeffector accelerations to a file
  void writeEndeffectorAccTrajectoryToFile(vector<vector<double> > ee_accs, char *motion_file);

  //Write a sequence of endeffector jerks to a file
  void writeEndeffectorJerkTrajectoryToFile(vector<vector<double> > ee_jerks, char *motion_file);





  private:
  //Node Handle
  ros::NodeHandle nh_;

  //KDL Tree build from parameter/urdf file
  KDL::Tree kdl_tree_;

  //Stores the values of the robot_description parameter
  string robot_desc_string_;

  //KDL Limb Chains
  //KDL::Chain LeftArmChain_;
  KDL::Chain complete_arm_;

  //Base Link Name used as reference frame for ee trajectory
  string base_link_name_;

  //Number of joints
  int num_joints_;      //in total
  int num_joints_revolute_;  //in rotational
  int num_joints_prismatic_; //in prismatic


  //Planning Scene Monitor
  boost::shared_ptr<planning_scene::PlanningScene> p_s_;

  //Planning Scene Publisher
  ros::Publisher scene_pub_;

  //Marker Publisher (to visualize endeffector trajectory)
  ros::Publisher ee_traj_pub_;

  //Marker Publisher (to visualize kinematic chain control points used for CA)
  ros::Publisher ca_control_points_pub_;

  //Map storing the joint names and corresponding limits
  JointLimits joint_limits_;

  //Get the joint limits (used by getJointRangeData-function)
  void getJointLimits(string joint_name, double &q_min, double &q_max);

  //Line Strip marker vector storing endeffector positions
  visualization_msgs::Marker ee_traj_;

  //Line Strip marker vector storing endeffector positions
  visualization_msgs::Marker ca_control_points_;

};




}//end of namespace


#endif //KDL_KUKA_MODEL_H_
 
 
