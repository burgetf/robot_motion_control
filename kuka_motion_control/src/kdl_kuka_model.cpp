

#include <kuka_motion_control/kdl_kuka_model.h>
#include <tf/transform_broadcaster.h>


using namespace std;

namespace kuka_motion_controller{

//Constructor
KDLRobotModel::KDLRobotModel(string robot_desciption_param, string planning_scene_topic, string ee_trajectory_topic, string chain)
{

    //-------------------- Build KDL tree from parameter server param (here robot_description) ----------------------------
    nh_.param(robot_desciption_param, robot_desc_string_, string("robot_description"));

    //Set robot description parameter
    //robot_desc_string_ = robot_desciption_param;
    //cout<<robot_desc_string_<<endl;

    if (!kdl_parser::treeFromString(robot_desc_string_, kdl_tree_)){
      ROS_ERROR("Failed to construct kdl tree");
      //return false;
    }
    else
    {
        ROS_INFO("KDL tree build successfully");
    }


    //-------------------- Get base and tip links for the Kinematic chains from SRDF ----------------------------
    //Create planning scene
    robot_model_loader::RobotModelLoader robot_model_loader(robot_desciption_param);
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    p_s_ = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(kinematic_model));


    //Get SRDF data for robot
    const boost::shared_ptr< const srdf::Model > &srdf_robot = p_s_->getRobotModel()->getSRDF();
    

    //Get the groups specified in the urdf
    //cout<<"srdf getName: "<<srdf_robot->getName() <<endl;
    const vector< srdf::Model::Group> &srdf_group = srdf_robot->getGroups();

    //Create Robot State
    //robot_state_ = boost::shared_ptr<robot_state::RobotState>(new robot_state::RobotState(psm_->getRobotModel()));

    //Scene publisher
    scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 10);

    //Init joint limits map
    //Get the urdf from the robot model loader
    const boost::shared_ptr<urdf::ModelInterface> robot_urdf = robot_model_loader.getURDF();

    //Init number of joints
    num_joints_ = 0;       //in total
    num_joints_revolute_ = 0;   //in rotational
    num_joints_prismatic_ = 0; //in prismatic

    
    //Get the tip link 
    boost::shared_ptr<const urdf::Link> link;
    //Joint from the urdf
    boost::shared_ptr<const urdf::Joint> joint;


    //Find the base and tip link of the kinematic chain
    for (int i = 0; i<srdf_group.size(); i++)
    {

        //!!!!!!!!!!!!!!!!!!!!TODO: Get the total number of joint for all groups contained in the srdf!!!!!!!!!
        // ->then resize joint_limits_ only once and set the values as done in the code below

        if (srdf_group[i].name_ == chain)
        {
            cout<<"Name of Kinematic Group: "<<srdf_group[i].name_<<endl;
            cout<<"base link Name: "<<srdf_group[i].chains_[0].first<<endl;
            cout<<"tip link Name: "<<srdf_group[i].chains_[0].second<<endl<<endl;

            //Store base link name
            base_link_name_ = srdf_group[i].chains_[0].first;

            //Reset num joints
            num_joints_ = 0;

            //Get the chain from the KDL tree
            kdl_tree_.getChain(srdf_group[i].chains_[0].first,srdf_group[i].chains_[0].second, complete_arm_);

            //Get the tip link
            link = robot_urdf->getLink(srdf_group[i].chains_[0].second);

            while (link && link->name != srdf_group[i].chains_[0].first)
            {
                joint = robot_urdf->getJoint(link->parent_joint->name);
                if (!joint)
                {
                    ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
                    //return false;
                }
                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                    //ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
                    num_joints_++;
                }
                link = robot_urdf->getLink(link->getParent()->name);
            }

            //Set the size of the vectors containing the joint names and joint limits
            joint_limits_.joint_names.resize(num_joints_);
            joint_limits_.q_min.resize(num_joints_);
            joint_limits_.q_max.resize(num_joints_);


            //Get the tip link
            link = robot_urdf->getLink(srdf_group[i].chains_[0].second);


            //Get the joint limits
            unsigned int n = 0;
            while (link && n < num_joints_)
               {
                   joint = robot_urdf->getJoint(link->parent_joint->name);
                   if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
                   {
                       //ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

                       float lower, upper;
                       int hasLimits;
                       if ( joint->type != urdf::Joint::CONTINUOUS )
                       {
                           lower = joint->limits->lower;
                           upper = joint->limits->upper;
                           hasLimits = 1;

                           if(joint->type == urdf::Joint::PRISMATIC)
                               num_joints_prismatic_++;
                           if(joint->type == urdf::Joint::REVOLUTE)
                               num_joints_revolute_++;
                       }
                       else
                       {
                           lower = -M_PI;
                           upper = M_PI;
                           hasLimits = 0;

                           //Note: Continous joints are counted as revolute joints
                           num_joints_revolute_++;
                       }
                       int index = num_joints_ - n -1;

                       joint_limits_.q_min[index] = lower;
                       joint_limits_.q_max[index] = upper;
                       joint_limits_.joint_names[index] = joint->name;
                       n++;

                       //cout<<"Joint name: "<<joint->name<<endl;
                       //cout<<"Joint min val: "<<lower<<endl;
                       //cout<<"Joint max val: "<<upper<<endl;

                  }
                   link = robot_urdf->getLink(link->getParent()->name);
               }

            //Exit SRDF Iteration
            break;

        }


//        if (srdf_group[i].name_ == "kuka_complete_arm" && chain == "kuka_complete_arm")
//        {
//            cout<<"Name of Kuka Arm Group: "<<srdf_group[i].name_<<endl;
//            cout<<"base link Name: "<<srdf_group[i].chains_[0].first<<endl;
//            cout<<"tip link Name: "<<srdf_group[i].chains_[0].second<<endl<<endl;

//            //Store base link name
//            base_link_name_ = srdf_group[i].chains_[0].first;

//            //Reset num joints
//            num_joints_ = 0;

//            //Get the chain from the KDL tree
//            kdl_tree_.getChain(srdf_group[i].chains_[0].first,srdf_group[i].chains_[0].second, complete_arm_);

//            //Get the tip link
//            link = robot_urdf->getLink(srdf_group[i].chains_[0].second);

//            while (link && link->name != srdf_group[i].chains_[0].first)
//            {
//                joint = robot_urdf->getJoint(link->parent_joint->name);
//                if (!joint)
//                {
//                    ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
//                    //return false;
//                }
//                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
//                    //ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
//                    num_joints_++;
//                }
//                link = robot_urdf->getLink(link->getParent()->name);
//            }

//            //Set the size of the vectors containing the joint names and joint limits
//            joint_limits_.joint_names.resize(num_joints_);
//            joint_limits_.q_min.resize(num_joints_);
//            joint_limits_.q_max.resize(num_joints_);


//            //Get the tip link
//            link = robot_urdf->getLink(srdf_group[i].chains_[0].second);


//            //Get the joint limits
//            unsigned int n = 0;
//            while (link && n < num_joints_)
//               {
//                   joint = robot_urdf->getJoint(link->parent_joint->name);
//                   if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
//                   {
//                       //ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

//                       float lower, upper;
//                       int hasLimits;
//                       if ( joint->type != urdf::Joint::CONTINUOUS )
//                       {
//                           lower = joint->limits->lower;
//                           upper = joint->limits->upper;
//                           hasLimits = 1;

//                           if(joint->type == urdf::Joint::PRISMATIC)
//                               num_joints_prismatic_++;
//                           if(joint->type == urdf::Joint::REVOLUTE)
//                               num_joints_revolute_++;
//                       }
//                       else
//                       {
//                           lower = -M_PI;
//                           upper = M_PI;
//                           hasLimits = 0;

//                           //Note: Continous joints are counted as revolute joints
//                           num_joints_revolute_++;
//                       }
//                       int index = num_joints_ - n -1;

//                       joint_limits_.q_min[index] = lower;
//                       joint_limits_.q_max[index] = upper;
//                       joint_limits_.joint_names[index] = joint->name;
//                       n++;

//                       //cout<<"Joint name: "<<joint->name<<endl;
//                       //cout<<"Joint min val: "<<lower<<endl;
//                       //cout<<"Joint max val: "<<upper<<endl;

//                  }
//                   link = robot_urdf->getLink(link->getParent()->name);
//               }

//            //Exit SRDF Iteration
//            break;

//        }


//        if (srdf_group[i].name_ == "omnirob_lbr_sdh" && chain == "omnirob_lbr_sdh")
//        {
//            cout<<"Name of Kuka Robot Group: "<<srdf_group[i].name_<<endl;
//            cout<<"base link Name: "<<srdf_group[i].chains_[0].first<<endl;
//            cout<<"tip link Name: "<<srdf_group[i].chains_[0].second<<endl<<endl;

//            //Store base link name
//            base_link_name_ = srdf_group[i].chains_[0].first;

//            //Reset num joints
//            num_joints_ = 0;

//            //Get the chain from the KDL tree
//            kdl_tree_.getChain(srdf_group[i].chains_[0].first,srdf_group[i].chains_[0].second, complete_arm_);

//            //Get the tip link
//            link = robot_urdf->getLink(srdf_group[i].chains_[0].second);

//            while (link && link->name != srdf_group[i].chains_[0].first)
//            {
//                joint = robot_urdf->getJoint(link->parent_joint->name);
//                if (!joint)
//                {
//                    ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
//                    //return false;
//                }
//                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
//                    //ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
//                    num_joints_++;
//                }
//                link = robot_urdf->getLink(link->getParent()->name);
//            }

//            //Set the size of the vectors containing the joint names and joint limits
//            joint_limits_.joint_names.resize(num_joints_);
//            joint_limits_.q_min.resize(num_joints_);
//            joint_limits_.q_max.resize(num_joints_);


//            //Get the tip link
//            link = robot_urdf->getLink(srdf_group[i].chains_[0].second);


//            //Get the joint limits
//            unsigned int n = 0;
//            while (link && n < num_joints_)
//               {
//                   joint = robot_urdf->getJoint(link->parent_joint->name);
//                   if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
//                   {
//                       //ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

//                       float lower, upper;
//                       int hasLimits;
//                       if ( joint->type != urdf::Joint::CONTINUOUS )
//                       {
//                           lower = joint->limits->lower;
//                           upper = joint->limits->upper;
//                           hasLimits = 1;

//                           if(joint->type == urdf::Joint::PRISMATIC)
//                               num_joints_prismatic_++;
//                           if(joint->type == urdf::Joint::REVOLUTE)
//                               num_joints_revolute_++;

//                       }
//                       else
//                       {
//                           lower = -M_PI;
//                           upper = M_PI;
//                           hasLimits = 0;

//                           //Note: Continous joints are counted as revolute joints
//                           num_joints_revolute_++;
//                       }
//                       int index = num_joints_ - n -1;

//                       joint_limits_.q_min[index] = lower;
//                       joint_limits_.q_max[index] = upper;
//                       joint_limits_.joint_names[index] = joint->name;
//                       n++;

//                       //cout<<"Joint name: "<<joint->name<<endl;
//                       //cout<<"Joint min val: "<<lower<<endl;
//                       //cout<<"Joint max val: "<<upper<<endl;

//                  }
//                   link = robot_urdf->getLink(link->getParent()->name);
//               }

//            //Exit SRDF Iteration
//            break;

//        }


//        if (srdf_group[i].name_ == "robotino_robot" && chain == "robotino_robot")
//        {
//            cout<<"Name of Robotino Robot Group: "<<srdf_group[i].name_<<endl;
//            cout<<"base link Name: "<<srdf_group[i].chains_[0].first<<endl;
//            cout<<"tip link Name: "<<srdf_group[i].chains_[0].second<<endl<<endl;

//            //Store base link name
//            base_link_name_ = srdf_group[i].chains_[0].first;

//            //Reset num joints
//            num_joints_ = 0;

//            //Get the chain from the KDL tree
//            kdl_tree_.getChain(srdf_group[i].chains_[0].first,srdf_group[i].chains_[0].second, complete_arm_);

//            //Get the tip link
//            link = robot_urdf->getLink(srdf_group[i].chains_[0].second);

//            while (link && link->name != srdf_group[i].chains_[0].first)
//            {
//                joint = robot_urdf->getJoint(link->parent_joint->name);
//                if (!joint)
//                {
//                    ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
//                    //return false;
//                }
//                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
//                    //ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
//                    num_joints_++;
//                }
//                link = robot_urdf->getLink(link->getParent()->name);
//            }

//            //Set the size of the vectors containing the joint names and joint limits
//            joint_limits_.joint_names.resize(num_joints_);
//            joint_limits_.q_min.resize(num_joints_);
//            joint_limits_.q_max.resize(num_joints_);


//            //Get the tip link
//            link = robot_urdf->getLink(srdf_group[i].chains_[0].second);


//            //Get the joint limits
//            unsigned int n = 0;
//            while (link && n < num_joints_)
//               {
//                   joint = robot_urdf->getJoint(link->parent_joint->name);
//                   if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
//                   {
//                       //ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

//                       float lower, upper;
//                       int hasLimits;
//                       if ( joint->type != urdf::Joint::CONTINUOUS )
//                       {
//                           lower = joint->limits->lower;
//                           upper = joint->limits->upper;
//                           hasLimits = 1;

//                           if(joint->type == urdf::Joint::PRISMATIC)
//                               num_joints_prismatic_++;
//                           if(joint->type == urdf::Joint::REVOLUTE)
//                               num_joints_revolute_++;

//                       }
//                       else
//                       {
//                           lower = -M_PI;
//                           upper = M_PI;
//                           hasLimits = 0;

//                           //Note: Continous joints are counted as revolute joints
//                           num_joints_revolute_++;
//                       }
//                       int index = num_joints_ - n -1;

//                       joint_limits_.q_min[index] = lower;
//                       joint_limits_.q_max[index] = upper;
//                       joint_limits_.joint_names[index] = joint->name;
//                       n++;

//                       //cout<<"Joint name: "<<joint->name<<endl;
//                       //cout<<"Joint min val: "<<lower<<endl;
//                       //cout<<"Joint max val: "<<upper<<endl;

//                  }
//                   link = robot_urdf->getLink(link->getParent()->name);
//               }

//            //Exit SRDF Iteration
//            break;

//        }

    }   // end of srdf iteration

    //-------------------- Endeffector trajectory drawing setup ----------------------------
    //Marker Publisher (to visualize endeffector trajectory)
    ee_traj_pub_ = nh_.advertise<visualization_msgs::Marker>(ee_trajectory_topic, 10);

    ee_traj_.header.frame_id = base_link_name_;
    ee_traj_.header.stamp = ros::Time::now();
    ee_traj_.ns = "line_strip";
    ee_traj_.action = visualization_msgs::Marker::ADD;
    ee_traj_.pose.orientation.w = 1.0;
    ee_traj_.id = 1;
    //Set Marker Type
    ee_traj_.type = visualization_msgs::Marker::LINE_STRIP;
    //LINE_STRIP markers use only the x component of scale, for the line width
    ee_traj_.scale.x = 0.03;


    //-------------------- Control points drawing setup ----------------------------
    //Marker Publisher (to visualize kinematic chain control points used for CA)
    ca_control_points_pub_ = nh_.advertise<visualization_msgs::Marker>("ca_control_points", 10);

    ca_control_points_.header.frame_id = base_link_name_;
    ca_control_points_.header.stamp = ros::Time::now();
    ca_control_points_.ns = "control_point_spheres";
    ca_control_points_.action = visualization_msgs::Marker::ADD;
    ca_control_points_.pose.orientation.w = 1.0;
    ca_control_points_.id = 1;
    //Set Marker Type
    ca_control_points_.type = visualization_msgs::Marker::SPHERE_LIST;

}//end of constructor

//Destructor
KDLRobotModel::~KDLRobotModel()
{
    //Nothing to do yet
}




//Get the Arm Kinematic Chain
KDL::Chain KDLRobotModel::getCompleteArmChain()
{
    return complete_arm_;
}


//Get joint names of the kinematic chain
vector<string> KDLRobotModel::getJointNames()
{
    return joint_limits_.joint_names;
}


//Get number of joints of the kinematic chain
int KDLRobotModel::getNumJoints()
{
    return num_joints_;
}
int KDLRobotModel::getNumRevoluteJoints()
{
    return num_joints_revolute_;
}
int KDLRobotModel::getNumPrismaticJoints()
{
    return num_joints_prismatic_;
}



//Compute the forward kinematics for a kinematic chain
std::vector<double> KDLRobotModel::compute_FK(KDL::Chain kin_chain, KDL::JntArray chain_config)
{
    //Forward Kinematic Solver for kinematic chain
    KDL::ChainFkSolverPos_recursive fk_solver_body_chain(kin_chain);


    //Stores the FK result: Current Endeffector pose in cartesian space expressd as transformation matrix-> expressed w.r.t base frame
    KDL::Frame ee_cart_pose;

    //Stores the EE position and orientation, expressed as a vector (x,y,z, Xrot, Yrot, Zrot)
    //std::vector<double> ee_pose(6);
    std::vector<double> ee_pose(7);

    //Solve forward kinematics for body chain
    int success = fk_solver_body_chain.JntToCart(chain_config, ee_cart_pose);

    //Get EE position
    ee_pose[0] =  ee_cart_pose.p.x();
    ee_pose[1] =  ee_cart_pose.p.y();
    ee_pose[2] =  ee_cart_pose.p.z();
    //Get EE orientation (ZYX-euler angles)
    //ee_cart_pose.M.GetEulerZYX(ee_pose[5], ee_pose[4], ee_pose[3]); //GetEulerZYX gets the euler ZYX parameters of a rotation : First rotate around Z with alfa, then around the new Y with beta, then around new X with gamma.

    //Get the endeffector orientation as a quaternion (q = x,y,z,w)
    ee_cart_pose.M.GetQuaternion(ee_pose[3], ee_pose[4], ee_pose[5], ee_pose[6]);

    return ee_pose;
}



//Compute the forward kinematics for a kinematic chain (for a specific segment)
std::vector<double> KDLRobotModel::compute_FK_Segment(KDL::Chain kin_chain, KDL::JntArray chain_config, int segment)
{
    //Forward Kinematic Solver for kinematic chain
    KDL::ChainFkSolverPos_recursive fk_solver_body_chain(kin_chain);

    //Stores the FK result: Current Endeffector pose in cartesian space expressd as transformation matrix-> expressed w.r.t base frame
    KDL::Frame ee_cart_pose;

    //Stores the EE position and orientation, expressed as a vector (x,y,z, Xrot, Yrot, Zrot)
    //std::vector<double> ee_pose(6);
    std::vector<double> ee_pose(7);

    //Solve forward kinematics for body chain
    int success = fk_solver_body_chain.JntToCart(chain_config, ee_cart_pose,segment);

    //Get EE position
    ee_pose[0] =  ee_cart_pose.p.x();
    ee_pose[1] =  ee_cart_pose.p.y();
    ee_pose[2] =  ee_cart_pose.p.z();
    //Get EE orientation (ZYX-euler angles)
    //ee_cart_pose.M.GetEulerZYX(ee_pose[5], ee_pose[4], ee_pose[3]); //GetEulerZYX gets the euler ZYX parameters of a rotation : First rotate around Z with alfa, then around the new Y with beta, then around new X with gamma.

    //Get the endeffector orientation as a quaternion
    ee_cart_pose.M.GetQuaternion(ee_pose[3], ee_pose[4], ee_pose[5], ee_pose[6]);

    return ee_pose;
}



//Compute the forward kinematics for a kinematic chain (returning the pose as a frame)
KDL::Frame KDLRobotModel::compute_FK_frame(KDL::Chain kin_chain, KDL::JntArray chain_config)
{
    //Forward Kinematic Solver for kinematic chain
    KDL::ChainFkSolverPos_recursive fk_solver_body_chain(kin_chain);

    //Stores the FK result: Current Endeffector pose in cartesian space expressd as transformation matrix-> expressed w.r.t base frame
    KDL::Frame ee_cart_pose;

    //Solve forward kinematics for body chain
    int success = fk_solver_body_chain.JntToCart(chain_config, ee_cart_pose);

//    if(success>=0)
//    {
//        for(int i = 0 ; i < 4 ; i++)
//        {
//            for(int j = 0 ; j < 4 ; j++)
//            {
//                std::cout << ee_cart_pose.operator ()(i,j) <<" ";
//            }
//            std::cout <<endl;
//        }
//            printf("%s \n","Succes, thanks KDL!");
//    }
//    else
//    {
//            printf("%s \n","Error: could not calculate forward kinematics :(");
//    }

//    int z;
//    cin>>z;

    return ee_cart_pose;

}



//Function to build a custom kinematic chain from the robot given a root and tip
bool KDLRobotModel::getCustomChain(const string &chain_root, const string &chain_tip, KDL::Chain &custom_chain)
{
      //Flag indicating an error (when chain_root or chain_tip does not exist)
      bool error = false;

      //Get all segments of the KDL::tree
      KDL::SegmentMap seg_map = kdl_tree_.getSegments();
      //Check whether chain_root is a member of the robot
      KDL::SegmentMap::const_iterator it = seg_map.find(chain_root);
      if (it == seg_map.end())
       {
          ROS_ERROR("The specified chain_root is not a link of the robot!!!");
          error = true;
       }
      //Check whether chain_tip is a member of the robot
      it = seg_map.find(chain_tip);
      if (it == seg_map.end())
       {
          ROS_ERROR("The specified chain_tip is not a link of the robot!!!");
          error = true;
       }

      if (error == false)
      {
          //Chains from common parent to respective chain tip
          KDL::Chain forward_chain_tip;
          KDL::Chain forward_chain_root;

          //Get Name of the KDL tree root segment
          string tree_root_name = kdl_tree_.getRootSegment()->second.segment.getName();
          //cout<<"Tree root name: "<<tree_root_name<<endl;

          //Get forward chain from root segment to chain_tip
          kdl_tree_.getChain(tree_root_name, chain_tip, forward_chain_tip);
          //cout<<forward_chain_tip.getNrOfSegments()<<endl;
          //cout<<forward_chain_tip.getSegment(0).getName()<<endl;

          //Get forward chain from common_parent to chain_root
          kdl_tree_.getChain(tree_root_name, chain_root, forward_chain_root);


          //Find common parent of chain_root and chain_tip
          string common_parent;
          string curr_segment_name_chain_root;
          string curr_segment_name_chain_tip;
          bool common_parent_found = false;

          //Iterate backwards through the segments of the root_chain
          for (int i = forward_chain_root.getNrOfSegments()-1 ; i >= 0 ; i--)
          {
              curr_segment_name_chain_root = forward_chain_root.getSegment(i).getName();
              //cout<<"Current segment name root chain: "<<curr_segment_name_chain_root<<endl;

              //Look for the segment in the forward_chain_tip kinematic chain
              for (int j = forward_chain_tip.getNrOfSegments()-1 ; j >= 0 ; j--)
              {
                  curr_segment_name_chain_tip = forward_chain_tip.getSegment(j).getName();
                  //cout<<"Current segment name tip chain: "<<curr_segment_name_chain_tip<<endl;

                  //Common segment of root and tip chain found
                  if (curr_segment_name_chain_root == curr_segment_name_chain_tip)
                  {
                      common_parent = curr_segment_name_chain_root;
                      common_parent_found = true;
                      cout<<"Common parent of chains: "<<common_parent<<endl;

                      //If the chain_root is on the same kinematic chain as the chain_tip (no kinematic inversion required)
                      if (common_parent == chain_root)
                      {
                          //Simply build the chain with the getChain function
                          kdl_tree_.getChain(chain_root,chain_tip,custom_chain);
                      }
                      //If the chain_tip is on the same kinematic chain as the chain_root (kinematic inversion required)
                      else if (common_parent == chain_tip)
                      {
                          //Go backwards from endeffector (now root of our new chain to the common parent of the two chains)  <-  Not sure whether correct!!!!
                          //Start from penultimate segment (as the getChain function does)
                          for (int s = forward_chain_root.getNrOfSegments()-1; s >= 0; s--)
                          {
                             KDL::Frame curr_frame(forward_chain_root.getSegment(s).getFrameToTip().Inverse());
                             KDL::Rotation curr_rot(curr_frame.M);
                             KDL::Vector curr_p(curr_frame.p);

    //                         KDL::Frame orig_frame(forward_chain_root.getSegment(s).getFrameToTip());
    //                         KDL::Rotation orig_rot(orig_frame.M);
    //                         KDL::Vector orig_p(orig_frame.p);

    //                         cout<<"------------------ ORIGINAL ----------------------"<<endl;
    //                         cout<<"X trans: "<<orig_p.x()<<endl;
    //                         cout<<"Y trans: "<<orig_p.y()<<endl;
    //                         cout<<"Z trans: "<<orig_p.z()<<endl;

    //                         double zr1_check,yr_check,zr2_check;
    //                         orig_rot.GetEulerZYZ(zr1_check,yr_check,zr2_check);
    //                         cout<<"ZR1 rot: "<<zr1_check<<endl;
    //                         cout<<"Y rot: "<<yr_check<<endl;
    //                         cout<<"ZR2 rot: "<<zr2_check<<endl;

    //                         cout<<"------------------ INVERTED ----------------------"<<endl;
    //                         cout<<"X trans: "<<curr_p.x()<<endl;
    //                         cout<<"Y trans: "<<curr_p.y()<<endl;
    //                         cout<<"Z trans: "<<curr_p.z()<<endl;

    //                         curr_rot.GetEulerZYZ(zr1_check,yr_check,zr2_check);
    //                         cout<<"ZR1 rot: "<<zr1_check<<endl;
    //                         cout<<"Y rot: "<<yr_check<<endl;
    //                         cout<<"ZR2 rot: "<<zr2_check<<endl;


                             string new_segment_name = forward_chain_root.getSegment(s).getName();
                             new_segment_name.append("_inv");
                             //Re-create chain with inverse order of segments
                             custom_chain.addSegment(KDL::Segment(new_segment_name,forward_chain_root.getSegment(s).getJoint(),KDL::Frame(curr_rot,curr_p)));

                             //Stop when chain_tip link has been added to the custom chain
                             if (forward_chain_root.getSegment(s).getName() == chain_tip)
                                  break;
                          }
                      }
                       //If the chain_root and chain_tip are on different kinematic chains (kinematic inversion + appending forward chain required)
                      else
                      {
                          //Go backwards from endeffector (now root of our new chain to the common parent of the two chains)  <-  Not sure whether correct!!!!
                          //Start from penultimate segment (as the getChain function does)
                          for (int s = forward_chain_root.getNrOfSegments()-1; s >= 0; s--)
                          {
                             KDL::Frame curr_frame(forward_chain_root.getSegment(s).getFrameToTip().Inverse());
                             KDL::Rotation curr_rot(curr_frame.M);
                             KDL::Vector curr_p(curr_frame.p);

    //                         KDL::Frame orig_frame(forward_chain_root.getSegment(s).getFrameToTip());
    //                         KDL::Rotation orig_rot(orig_frame.M);
    //                         KDL::Vector orig_p(orig_frame.p);

    //                         cout<<"------------------ ORIGINAL ----------------------"<<endl;
    //                         cout<<"X trans: "<<orig_p.x()<<endl;
    //                         cout<<"Y trans: "<<orig_p.y()<<endl;
    //                         cout<<"Z trans: "<<orig_p.z()<<endl;

    //                         double zr1_check,yr_check,zr2_check;
    //                         orig_rot.GetEulerZYZ(zr1_check,yr_check,zr2_check);
    //                         cout<<"ZR1 rot: "<<zr1_check<<endl;
    //                         cout<<"Y rot: "<<yr_check<<endl;
    //                         cout<<"ZR2 rot: "<<zr2_check<<endl;

    //                         cout<<"------------------ INVERTED ----------------------"<<endl;
    //                         cout<<"X trans: "<<curr_p.x()<<endl;
    //                         cout<<"Y trans: "<<curr_p.y()<<endl;
    //                         cout<<"Z trans: "<<curr_p.z()<<endl;

    //                         curr_rot.GetEulerZYZ(zr1_check,yr_check,zr2_check);
    //                         cout<<"ZR1 rot: "<<zr1_check<<endl;
    //                         cout<<"Y rot: "<<yr_check<<endl;
    //                         cout<<"ZR2 rot: "<<zr2_check<<endl;


                             string new_segment_name = forward_chain_root.getSegment(s).getName();
                             new_segment_name.append("_inv");

                             //Don't add the common_parent, otherwise it will be in the new chain twice
                             if(forward_chain_root.getSegment(s).getName() != common_parent)
                             {
                                 //Re-create chain with inverse order of segments
                                 custom_chain.addSegment(KDL::Segment(new_segment_name,forward_chain_root.getSegment(s).getJoint(),KDL::Frame(curr_rot,curr_p)));
                             }

                          }

                          //Append the forward chain to the chain_tip to the custom_chain
                          custom_chain.addChain(forward_chain_tip);
                      }


                      //Leave inner loop
                      break;
                  }
              }

              //Quit searching when common parent of the two chains has been found or the two Kin.Chains have the same endeffector (this is an error)
              if(common_parent_found == true)
                 break;
          }

          if(common_parent_found == false)
              ROS_ERROR("The root and tip link specified have no common parent link!!!");

      } //end of error query


      //Return true when custom chain has been build successfully (no error)
      if (error == false)
          return true;
      else
          return false;

}


//Add a Tool Center Point (TCP) to a kinematic chain (ee_tcp_transform specifies the transformation between the endeffector and the tcp)
void KDLRobotModel::addTCP(string name, KDL::Frame ee_tcp_transform, KDL::Chain &kin_chain)
{
    //Add the TCP to the Kinematic Chain
    kin_chain.addSegment(KDL::Segment(name ,KDL::Joint(name,KDL::Joint::None),KDL::Frame(ee_tcp_transform.M,ee_tcp_transform.p)));
}



//Add a virtual base joint to a kinematic chain
void KDLRobotModel::addVirtualBaseJoint(KDL::Chain &kin_chain, KDL::Joint base_joint, double q_min, double q_max, KDL::Frame transform)
{
    //Add Joint Limits Information to struct
    joint_limits_.joint_names.push_back(base_joint.getName());
    joint_limits_.q_min.push_back(q_min);
    joint_limits_.q_max.push_back(q_max);

    //Create a name for the new segment
    string seg_name = "virtual_" + base_joint.getName() + "_link";

    //Create Segment
    KDL::Segment base_segment = KDL::Segment(seg_name,base_joint,transform);

    //Create KDL::Chain
    KDL::Chain kin_chain_new;

    //Add Segment to chain
    kin_chain_new.addSegment(base_segment);

    //Add original chain to base segment
    kin_chain_new.addChain(kin_chain);

    //Assign new kinematic chain to input chain
    kin_chain = kin_chain_new;
}


void KDLRobotModel::getJointRangeData(KDL::Chain kin_chain, vector<double>& q_min, vector<double>& q_max , vector<double>& q_center)
{
    //cout<<"Load Joint range information..."<<endl;

     //Joint Range Center
    int joint_num = 0;
    double tmp_limit = 0.0;

    //For the body chain (right arm to right leg)
    for (int k = 0 ; k < kin_chain.getNrOfSegments(); k++)
      {
          if(kin_chain.getSegment(k).getJoint().getTypeName() != "None")
          {
            //Search for "_inv" extension within segment name (indicating that this segment has been inverted in the custom chain build process)
            unsigned found_inv = kin_chain.getSegment(k).getName().find_last_of("_");

            //Inversion of joint range required
            if(kin_chain.getSegment(k).getName().substr(found_inv+1) == "inv")
            {
                getJointLimits(kin_chain.getSegment(k).getJoint().getName(),q_min[joint_num],q_max[joint_num]);
                tmp_limit = q_min[joint_num];
                q_min[joint_num] =  -q_max[joint_num];
                q_max[joint_num] = -tmp_limit;
                q_center[joint_num] = (q_max[joint_num]+q_min[joint_num])/2;
                //cout<<"Joint range for joint "<<kin_chain.getSegment(k).getJoint().getName()<<": "<<"min: "<<q_min[joint_num]<<" "<<"max: "<<q_max[joint_num]<<endl;
                joint_num++;
            }
            //No inversion of joint range required
            if(kin_chain.getSegment(k).getName().substr(found_inv+1) != "inv")
            {
                getJointLimits(kin_chain.getSegment(k).getJoint().getName(),q_min[joint_num],q_max[joint_num]);
                q_center[joint_num] = (q_max[joint_num]+q_min[joint_num])/2;
                //cout<<"Joint range for joint "<<kin_chain.getSegment(k).getJoint().getName()<<": "<<"min: "<<q_min[joint_num]<<" "<<"max: "<<q_max[joint_num]<<endl;
                joint_num++;
            }

          }
      }
}



//Get the Limits for a given Joint
void KDLRobotModel::getJointLimits(string joint_name, double &q_min, double &q_max)
{
    //Init q_min and q_max
    q_min = 0.0;
    q_max = 0.0;

    for (int i = 0 ; i < joint_limits_.joint_names.size(); i++)
    {
        if (joint_limits_.joint_names[i] == joint_name)
        {
           q_min = joint_limits_.q_min[i];
           q_max = joint_limits_.q_max[i];
        }
    }


    if (q_min == 0.0 && q_max == 0.0)
        ROS_ERROR("Joint not found");

}



bool KDLRobotModel::checkJointLimits(string joint_name, double q_val, bool show_output)
{
    //Init q_min and q_max
    double q_min = 0.0;
    double q_max = 0.0;

    //Get the joint limits for the joint
    getJointLimits(joint_name,q_min,q_max);

    //Check if joint value is within the joint range
    if (q_val < q_min || q_val > q_max)
    {
        if(show_output)
        {
            //cout<<"Joint "<<joint_name<<" violates joint limit"<<endl;
            //cout<<"Current val: "<<q_val<<" lower joint limit: "<<q_min<<" upper joint limit: "<<q_max<<endl;
        }
        //joint violates joint limit -> return false
        return false;
    }
    else
        return true;  //joint has a value within joint limit

}


//Get the minimal distance of a joint value to  the joint range limits
double KDLRobotModel::getDistanceToJointLimits(string joint_name, double q_val)
{
    //Joint Limits
    double q_min = 0.0, q_max = 0.0;

    //Get the joint limits for the joint "joint_name"
    getJointLimits(joint_name, q_min, q_max);

    double dist_to_upper_jl = q_max - q_val;
    double dist_to_lower_jl = -(q_min - q_val);

    //Check if joint is within joint limits
    if(!checkJointLimits(joint_name,q_val,false))
        return -1.0;
    else if (dist_to_upper_jl < dist_to_lower_jl) //if joint value is closer to upper limit
        return dist_to_upper_jl;
    else                                          //if joint value is closer to lower limit
        return dist_to_lower_jl;
}


//Publish the configuration of arbitrary kinematic chains onto the Planning Scene
bool KDLRobotModel::showConfig(vector<KDL::Chain> kin_chains, vector<KDL::JntArray> config_chains, double sleep_duration = 0.02)
{

//  cout<<"Num chains: "<<kin_chains.size()<<endl;
//  cout<<"Num config vectors: "<<config_chains.size()<<endl;
//  cout<<"Length of config vector 1 is:"<<config_chains[0].rows()<<endl;
//  cout<<"Length of config vector 2 is:"<<config_chains[1].rows()<<endl;
//  cout<<"Length of config vector 3 is:"<<config_chains[2].rows()<<endl;
//  cout<<"Length of config vector 4 is:"<<config_chains[3].rows()<<endl;

//  cout<<"Length of config in total:"<<config_chains[0].rows() + config_chains[1].rows() +config_chains[2].rows() + config_chains[3].rows()<<endl;

  //Map containing joint names and values
  map<string, double> nvalues;

  //Check if there's a configuration vector for each chain
  if (kin_chains.size() != config_chains.size())
  {
      ROS_ERROR("The number of kinematic chains does not match the number of configuration vectors");
      return false;
  }

  //Total number of joints (for all kinematic chains in the input vector)
  int num_joints_total = 0;

  for (int i = 0; i < kin_chains.size(); i++)
  {
      //Check if there's a value for each joint of the respective kinematic chain
      if (kin_chains[i].getNrOfJoints() != config_chains[i].rows())
      {
          ROS_ERROR("The number of joints of this chain does not match the number of entries in the configuration vectors");
          return false;
      }
      else
      {
          num_joints_total += kin_chains[i].getNrOfJoints();
      }
  }


  //Configuration to show in the planning scene
  KDL::JntArray show_config = KDL::JntArray(num_joints_total);

  //Stack configurations in a single vector
  int joint_pos_offset = 0;
  for (int i = 0; i < kin_chains.size(); i++)
  {
      if(i > 0)
      joint_pos_offset += kin_chains[i-1].getNrOfJoints();

      //Check if there's a value for each joint of the respective kinematic chain
      for (int j = 0; j < kin_chains[i].getNrOfJoints(); j++)
      {
          show_config(j+joint_pos_offset) = config_chains[i](j);
          //cout<<show_config(j+joint_pos_offset)<<endl;

      }
  }


  //cout<<endl;


  //Initialize Index of current configuration element
  int current_config_element = 0;
  int start_config_element = 0;


  //Invert the angles of joints which are part of an inverted chain (only required when custom chains are build involving chain inversion)
  for (int i = 0; i < kin_chains.size(); i++)
  {
      //The the start index for the configuration vector (storing the configuration of all chains in a single vector)
      int start_index_next_chain = 0;
      if(i > 0)
      {
          //cout<<"Curr conf: "<<current_config_element<<endl;
          //cout<<"chain dofs: "<<kin_chains[i-1].getNrOfJoints()<<endl;
          for (int chains = i ; chains > 0 ; chains--)
                start_index_next_chain = start_index_next_chain + kin_chains[chains-1].getNrOfJoints();

          current_config_element = start_index_next_chain;
          start_config_element = current_config_element;
      }

      for (int o = 0 ; o < kin_chains[i].getNrOfSegments(); o++)
      {
        //Search for "_inv" extension within segment name (indicating that this segment has been inverted in the custom chain build process)
        unsigned found_inv = kin_chains[i].getSegment(o).getName().find_last_of("_");

        //Search for "_inv" extension within segment name (indicating that this segment has been inverted in the custom chain build process)
        unsigned found_virtual = kin_chains[i].getSegment(o).getName().find_first_of("_");

        //Invert the value of this angle (to be coherent with the kinematic of the robot model, which does not know about the inversion of the kinematic chain)
        // Segment needs to have "inv" ending in it's name and corresponding joint must be a rotational one (not fixed)
        if(kin_chains[i].getSegment(o).getName().substr(found_inv+1) == "inv" && kin_chains[i].getSegment(o).getJoint().getTypeName() != "None")
        {
           //invert angle value to get correct rotation of joint in robot model
           show_config(current_config_element) = -show_config(current_config_element);

           //Store name and value of joint in a map (later used to set the robot state)
           nvalues[kin_chains[i].getSegment(o).getJoint().getName()] = show_config(current_config_element);

           //Check if joint is within joint limits
           checkJointLimits(kin_chains[i].getSegment(o).getJoint().getName(),show_config(current_config_element),false);

           //Show minimal distance to joint limit
           //double min_dist = getDistanceToJointLimits(kin_chains[i].getSegment(o).getJoint().getName(), show_config(current_config_element));
           //cout<<"Min. Dist. to JL: "<<min_dist<<endl;


           //cout<<"Angle value for joint: "<<kin_chains[i].getSegment(o).getJoint().getName()<<" inverted"<<", New Value is : " <<show_config(current_config_element)<<endl;
           current_config_element++;
        }

        //Configuration element index must be incremented when a rotational joint without the "inv" ending is encountered
        if(kin_chains[i].getSegment(o).getName().substr(found_inv+1) != "inv" && kin_chains[i].getSegment(o).getName().substr(0,found_virtual) != "virtual" && kin_chains[i].getSegment(o).getJoint().getTypeName() != "None")
        {
           //Store name and value of joint in a map (later used to set the robot state)
           nvalues[kin_chains[i].getSegment(o).getJoint().getName()] = show_config(current_config_element);

           //Check if joint is within joint limits
           checkJointLimits(kin_chains[i].getSegment(o).getJoint().getName(),show_config(current_config_element),false);

           //Show minimal distance to joint limit
           //double min_dist = getDistanceToJointLimits(kin_chains[i].getSegment(o).getJoint().getName(), show_config(current_config_element));
           //cout<<"Min. Dist. to JL: "<<min_dist<<endl;


           //cout<<"Angle value for joint: "<<kin_chains[i].getSegment(o).getJoint().getName()<<" is : " <<show_config(current_config_element)<<endl;

           current_config_element++;
        }

        //If the current segment is a virtual base segment it is not part of the robot configuration, thus not included in the map published to the planning scene
        if (kin_chains[i].getSegment(o).getName().substr(0,found_virtual) == "virtual")
        {
            //Check if joint is within joint limits
            checkJointLimits(kin_chains[i].getSegment(o).getJoint().getName(),show_config(current_config_element),false);


            //Just increment the configuration index
            current_config_element++;
        }
      }

  }


//  //Add some fixed values
//  nvalues["HeadYaw"] = 0.0;
//  nvalues["HeadPitch"] = 0.0;

//  nvalues["LShoulderPitch"] = 0.0; 	//Left shoulder joint front and back (Y) 	-119.5 to 119.5 	-2.0857 to 2.0857
//  nvalues["LShoulderRoll"] = 0.0; 	//Left shoulder joint right and left (Z) 	   -18 to 76 	    -0.3142 to 1.3265
//  nvalues["LElbowYaw"] = 0.0; 	    //Left shoulder joint twist (X) 	        -119.5 to 119.5 	-2.0857 to 2.0857
//  nvalues["LElbowRoll"] = -0.5; 	    //Left elbow joint (Z) 	                 -88.5 to -2 	    -1.5446 to -0.0349
//  nvalues["LWristYaw"] = 0.0; 	    //Left wrist joint (X) 	                    -104.5 to 104.5 	-1.8238 to 1.8238


  //------------- Publish configuration onto Planning Scene ------------------------
  //RobotState is the current configuration of the robot
  robot_state::RobotState robot_state(p_s_->getRobotModel());
  //Set all joint values to the default values
  robot_state.setToDefaultValues();

  //Set current robot state
  robot_state.setVariablePositions(nvalues);

  //Apply robot state to planning scene
  p_s_->setCurrentState(robot_state);

  //Publish state on planning scene
  moveit_msgs::PlanningScene psmsg;
  p_s_->getPlanningSceneMsg(psmsg);
  //psmsg.robot_model_root = "r_sole";
  scene_pub_.publish(psmsg);
  //sleep(1);
  ros::Duration(sleep_duration).sleep();

  //Return true when everything succeeds
  return true;

}



//Show endeffector trace in planning scene (RViz) with limited number of trace points
void KDLRobotModel::show_ee_trace(vector<double> curr_pose_vec, vector<double> color_rgb, int max_traj_points)
{
   // Line strip is green
   ee_traj_.color.r = color_rgb[0];
   ee_traj_.color.g = color_rgb[1];
   ee_traj_.color.b = color_rgb[2];
   ee_traj_.color.a = 1.0;

   //Set current endeffector position
   geometry_msgs::Point p;
   p.x = curr_pose_vec[0];
   p.y = curr_pose_vec[1];
   p.z = curr_pose_vec[2];

//   cout<<"EE trace point:"<<endl;
//   cout<<curr_pose_vec[0]<<endl;
//   cout<<curr_pose_vec[1]<<endl;
//   cout<<curr_pose_vec[2]<<endl;

   //Enter current ee position into marker vector
   ee_traj_.points.push_back(p);

   //Limit number of trajectory points shown to max_traj_points
   if(max_traj_points < ee_traj_.points.size())
   {
       // erase the first element
       ee_traj_.points.erase (ee_traj_.points.begin());
   }

   ee_traj_pub_.publish(ee_traj_);

}


//Show endeffector trace in planning scene (RViz) with unlimited number of trace points
void KDLRobotModel::show_ee_trace(vector<double> curr_pose_vec, vector<double> color_rgb)
{
   // Line strip is green
   ee_traj_.color.r = color_rgb[0];
   ee_traj_.color.g = color_rgb[1];
   ee_traj_.color.b = color_rgb[2];
   ee_traj_.color.a = 1.0;

   //Set current endeffector position
   geometry_msgs::Point p;
   p.x = curr_pose_vec[0];
   p.y = curr_pose_vec[1];
   p.z = curr_pose_vec[2];

   //Enter current ee position into marker vector
   ee_traj_.points.push_back(p);

   ee_traj_pub_.publish(ee_traj_);

}


//Delete endeffector trace in planning scene (RViz)
void KDLRobotModel::delete_ee_trace()
{
    ee_traj_.points.clear();
}


//Show control points used for collision avoidance
void KDLRobotModel::show_CA_Control_Points(vector< vector<double> > ca_control_points)
{
    //SPHERE_LIST markers use the scale for the radius (radius given by )
    if (ca_control_points.size() > 0)
    {
        //Set diameter of control point sphere
        ca_control_points_.scale.x = 2 * ca_control_points[0][3]; //2 * radius
        ca_control_points_.scale.y = 2 * ca_control_points[0][3];
        ca_control_points_.scale.z = 2 * ca_control_points[0][3];

        // Color of control point spheres
        ca_control_points_.color.r = 0.0;
        ca_control_points_.color.g = 0.0;
        ca_control_points_.color.b = 1.0;
        ca_control_points_.color.a = 0.5;


        for(int i = 0 ; i < ca_control_points.size(); i++)
        {
            //Set current endeffector position
            geometry_msgs::Point p;
            p.x = ca_control_points[i][0];
            p.y = ca_control_points[i][1];
            p.z = ca_control_points[i][2];

            //Enter current ee position into marker vector
            ca_control_points_.points.push_back(p);
        }

         ca_control_points_pub_.publish(ca_control_points_);

         //Clear control point list after publishing
         for(int i = 0 ; i < ca_control_points.size(); i++)
            ca_control_points_.points.pop_back();
    }
    else
    {
        ROS_ERROR("No collision avoidance control points available!!!");
    }


}



void KDLRobotModel::writeJointTrajectoryToFile(vector< map<string, double> > configurations, map<string, double>  joint_weights, char *motion_file)
{

    //-------------------- Write solution path into file
    //Remove the current solution file
    if( remove( motion_file ) != 0 )
    {
        cout<< "Error deleting file (writeJointTrajectoryToFile)" <<endl;
    }

    //Files to store configurations of motion
    ofstream joint_val_path;
    //Open a file which will store the sequence of traversed configurations
    joint_val_path.open(motion_file); // opens the file
    if( !joint_val_path)
      { // file couldn't be opened
            cerr << "Error: solution path file could not be opened" << endl;
            exit(1);
      }


    //Write joint names into File Header
    int line_break_counter = 0;

    for (map<string,double>::const_iterator it = joint_weights.begin(); it != joint_weights.end(); ++it)
    {
        joint_val_path << it->first << " ";
        //cout << it->first << " ";
        line_break_counter++;

        //Break Line
        if (line_break_counter == 7)
                joint_val_path << endl;
    }
    //Set cursor to next line (for configurations line)
    joint_val_path << endl;
    //cout<<endl;


    //Write joint weights into File Header
    for (map<string,double>::const_iterator it = joint_weights.begin(); it != joint_weights.end(); ++it)
    {
        joint_val_path << it->second << " ";
        //cout << it->second << " ";
    }
    //Set cursor to next line (for joint names header line)
    joint_val_path << endl;
    //cout<<endl;


    //Write the motion, i.e. the sequence of configs into a file
    for (unsigned int i = 0 ; i < configurations.size() ; i++)
    {
        //cout <<"Configuration number: "<< i<<endl;
        for (map<string,double>::const_iterator iter = configurations.at(i).begin(); iter != configurations.at(i).end(); ++iter)
        {
            joint_val_path << iter->second << " ";
            //cout << iter->second << " ";
        }

        //Set cursor to next line (for next configuration)
        joint_val_path << endl;
        //cout<<endl;

    }


    //Close the file
    joint_val_path.close();
}



//Write an arbitrary trajectory to file
void KDLRobotModel::writeTrajectoryToFile(vector< vector<double> > configurations, char *motion_file)
{

    //-------------------- Write solution path into file
    //Remove the current solution file
    if( remove( motion_file ) != 0 )
    {
        cout<< "Error deleting file (writeTrajectoryToFile)" <<endl;
    }

    //Files to store configurations of motion
    ofstream joint_val_path;
    //Open a file which will store the sequence of traversed configurations
    joint_val_path.open(motion_file); // opens the file
    if( !joint_val_path)
      { // file couldn't be opened
            cerr << "Error: solution path file could not be opened" << endl;
            exit(1);
      }


    //Write the motion, i.e. the sequence of configs into a file
    for (unsigned int i = 0 ; i < configurations.size() ; i++)
    {
        for (unsigned int j = 0 ; j < configurations[i].size() ; j++)
        {
            //cout <<"Configuration number: "<< i<<endl;
            joint_val_path << configurations[i][j] << " ";

        }

        //Set cursor to next line (for next configuration)
        joint_val_path << endl;
        //cout<<endl;

    }

    //Close the file
    joint_val_path.close();
}




//Write a sequence of joint velocities to a file
void KDLRobotModel::writeJointVelTrajectoryToFile(vector<vector<double> > joint_velos, char *motion_file)
{
    //-------------------- Write solution path into file
    //Remove the current solution file
    if( std::remove( motion_file ) != 0 )
    {
        std::cout<< "Error deleting file (writeJointVelTrajectoryToFile)" <<std::endl;
    }

    //File for endeffector trajectory
    std::ofstream joint_vel_path;
    //Open a file
    joint_vel_path.open(motion_file); // opens the file
    if( !joint_vel_path)
      { // file couldn't be opened
            std::cerr << "Error: joint velocity trajectory file could not be opened" << std::endl;
            exit(1);
      }

    //Write Header
    joint_vel_path << "Velocities for Joints from base_link to wrist_link"<< std::endl;

    //Write the endeffector trajectory into the file
    for (unsigned int i = 0 ; i < joint_velos.size() ; i++)
    {
        for (int j = 0 ; j<joint_velos[i].size() ; j++)
            joint_vel_path << joint_velos[i][j] <<" ";

        //Set cursor to next line (for next configuration)
        joint_vel_path << std::endl;

    }

    //Close the file
    joint_vel_path.close();
    //-------------------------------------------------------------


}



//Write joint acceleration trajectory
void KDLRobotModel::writeJointAccTrajectoryToFile(vector<vector<double> > joint_accs, char *motion_file)
{
    //-------------------- Write solution path into file
    //Remove the current solution file
    if( std::remove( motion_file ) != 0 )
    {
        std::cout<< "Error deleting file (writeJointAccTrajectoryToFile)" <<std::endl;
    }

    //File for endeffector trajectory
    std::ofstream joint_acc_path;
    //Open a file
    joint_acc_path.open(motion_file); // opens the file
    if( !joint_acc_path)
      { // file couldn't be opened
            std::cerr << "Error: joint acceleration trajectory file could not be opened" << std::endl;
            exit(1);
      }

    //Write Header
    joint_acc_path << "Accelerations for Joints from base_link to wrist_link"<< std::endl;

    //Write the endeffector trajectory into the file
    for (unsigned int i = 0 ; i < joint_accs.size() ; i++)
    {
        for (int j = 0 ; j<joint_accs[i].size() ; j++)
            joint_acc_path << joint_accs[i][j] <<" ";

        //Set cursor to next line (for next configuration)
        joint_acc_path << std::endl;

    }

    //Close the file
    joint_acc_path.close();
    //-------------------------------------------------------------
}


//Write joint jerk trajectory
void KDLRobotModel::writeJointJerkTrajectoryToFile(vector<vector<double> > joint_jerks, char *motion_file)
{
    //-------------------- Write solution path into file
    //Remove the current solution file
    if( std::remove( motion_file ) != 0 )
    {
        std::cout<< "Error deleting file (writeJointJerkTrajectoryToFile)" <<std::endl;
    }

    //File for endeffector trajectory
    std::ofstream joint_jerk_path;
    //Open a file
    joint_jerk_path.open(motion_file); // opens the file
    if( !joint_jerk_path)
      { // file couldn't be opened
            std::cerr << "Error: joint jerk trajectory file could not be opened" << std::endl;
            exit(1);
      }

    //Write Header
    joint_jerk_path << "Jerk for Joints from base_link to wrist_link"<< std::endl;

    //Write the endeffector trajectory into the file
    for (unsigned int i = 0 ; i < joint_jerks.size() ; i++)
    {
        for (int j = 0 ; j<joint_jerks[i].size() ; j++)
            joint_jerk_path << joint_jerks[i][j] <<" ";

        //Set cursor to next line (for next configuration)
        joint_jerk_path << std::endl;

    }

    //Close the file
    joint_jerk_path.close();
    //-------------------------------------------------------------
}



//Write a sequence of configurations to a file
void KDLRobotModel::writeEndeffectorTrajectoryToFile(vector<vector<double> > ee_poses, char *motion_file)
{
    //-------------------- Write solution path into file
    //Remove the current solution file
    if( std::remove( motion_file ) != 0 )
    {
        std::cout<< "Error deleting file (writeEndeffectorTrajectoryToFile)" <<std::endl;
    }

    //File for endeffector trajectory
    std::ofstream endeffector_path;
    //Open a file
    endeffector_path.open(motion_file); // opens the file
    if( !endeffector_path)
      { // file couldn't be opened
            std::cerr << "Error: endeffector trajectory file could not be opened" << std::endl;
            exit(1);
      }

    //Write Header
    endeffector_path << "X   Y   Z   quatX   quatY   quatZ   W"<< std::endl;

    //Write the endeffector trajectory into the file
    for (unsigned int i = 0 ; i < ee_poses.size() ; i++)
    {
        for (int j = 0 ; j<ee_poses[i].size() ; j++)
            endeffector_path << ee_poses[i][j] <<" ";

        //Set cursor to next line (for next configuration)
        endeffector_path << std::endl;

    }

    //Close the file
    endeffector_path.close();
    //-------------------------------------------------------------
}



//Write a sequence of endeffector velocities to a file
void KDLRobotModel::writeEndeffectorVelTrajectoryToFile(vector<vector<double> > ee_velos, char *motion_file)
{
    //-------------------- Write solution path into file
    //Remove the current solution file
    if( std::remove( motion_file ) != 0 )
    {
        std::cout<< "Error deleting file (writeEndeffectorVelTrajectoryToFile)" <<std::endl;
    }

    //File for endeffector trajectory
    std::ofstream endeffector_vel_path;
    //Open a file
    endeffector_vel_path.open(motion_file); // opens the file
    if( !endeffector_vel_path)
      { // file couldn't be opened
            std::cerr << "Error: endeffector velocity trajectory file could not be opened" << std::endl;
            exit(1);
      }

    //Write Header
    endeffector_vel_path << "Xdot   Ydot   Zdot   rotXdot   rotYdot   rotZdot"<< std::endl;

    //Write the endeffector trajectory into the file
    for (unsigned int i = 0 ; i < ee_velos.size() ; i++)
    {
        for (int j = 0 ; j<ee_velos[i].size() ; j++)
            endeffector_vel_path << ee_velos[i][j] <<" ";

        //Set cursor to next line (for next configuration)
        endeffector_vel_path << std::endl;

    }

    //Close the file
    endeffector_vel_path.close();
    //------------------------------------------------------------
}




//Write a sequence of endeffector accelerations to a file
void KDLRobotModel::writeEndeffectorAccTrajectoryToFile(vector<vector<double> > ee_accs, char *motion_file)
{
    //-------------------- Write solution path into file
    //Remove the current solution file
    if( std::remove( motion_file ) != 0 )
    {
        std::cout<< "Error deleting file (writeEndeffectorAccTrajectoryToFile)" <<std::endl;
    }

    //File for endeffector trajectory
    std::ofstream endeffector_acc_path;
    //Open a file
    endeffector_acc_path.open(motion_file); // opens the file
    if( !endeffector_acc_path)
      { // file couldn't be opened
            std::cerr << "Error: endeffector acceleration trajectory file could not be opened" << std::endl;
            exit(1);
      }

    //Write Header
    endeffector_acc_path << "Xacc   Yacc   Zacc   rotXacc   rotYacc   rotZacc"<< std::endl;

    //Write the endeffector trajectory into the file
    for (unsigned int i = 0 ; i < ee_accs.size() ; i++)
    {
        for (int j = 0 ; j<ee_accs[i].size() ; j++)
            endeffector_acc_path << ee_accs[i][j] <<" ";

        //Set cursor to next line (for next configuration)
        endeffector_acc_path << std::endl;

    }

    //Close the file
    endeffector_acc_path.close();
    //------------------------------------------------------------
}


//Write a sequence of endeffector jerks to a file
void KDLRobotModel::writeEndeffectorJerkTrajectoryToFile(vector<vector<double> > ee_jerks, char *motion_file)
{
    //-------------------- Write solution path into file
    //Remove the current solution file
    if( std::remove( motion_file ) != 0 )
    {
        std::cout<< "Error deleting file (writeEndeffectorJerkTrajectoryToFile)" <<std::endl;
    }

    //File for endeffector trajectory
    std::ofstream endeffector_jerk_path;
    //Open a file
    endeffector_jerk_path.open(motion_file); // opens the file
    if( !endeffector_jerk_path)
      { // file couldn't be opened
            std::cerr << "Error: endeffector jerk trajectory file could not be opened" << std::endl;
            exit(1);
      }

    //Write Header
    endeffector_jerk_path << "Xjerk   Yjerk   Zjerk   rotXjerk   rotYjerk   rotZjerk"<< std::endl;

    //Write the endeffector trajectory into the file
    for (unsigned int i = 0 ; i < ee_jerks.size() ; i++)
    {
        for (int j = 0 ; j<ee_jerks[i].size() ; j++)
            endeffector_jerk_path << ee_jerks[i][j] <<" ";

        //Set cursor to next line (for next configuration)
        endeffector_jerk_path << std::endl;

    }

    //Close the file
    endeffector_jerk_path.close();
    //------------------------------------------------------------

}

} //end of namespace

 
 
