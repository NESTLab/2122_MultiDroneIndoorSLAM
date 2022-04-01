#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatus.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>


class MoveBaseInterface
{
  public:
    MoveBaseInterface(ros::NodeHandle nh, bool testing=false);
    ~MoveBaseInterface(){};

    /**
     * @brief Get the Distance Prediction object
     *        Requests plan from start to end and return 
     *        the distance of the calculated plan
     * 
     * @param start_pose 
     * @param end_pose 
     * @return float distance of the calculated trajectory
     */
    float getDistancePrediction(geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose);

    /**
     * @brief Get the Distance Prediction object
     *        Requests plan from start to end and return 
     *        the distance of the calculated plan
     * 
     * @param start_pose 
     * @param end_pose 
     * @return float distance of the calculated trajectory
     */
    float getDistancePrediction(geometry_msgs::Point &start_point, geometry_msgs::Point &end_point);

    /**
     * @brief Get the Distance Prediction object
     *        Requests plan from start to end and return 
     *        the distance of the calculated plan
     * 
     * @param start_pose 
     * @param end_pose 
     * @return float distance of the calculated trajectory
     */
    float getDistancePrediction(geometry_msgs::Point &goal);

    /**
     * @brief Get the Time Prediction For Travel object
     * 
     * @param goal 
     * @return float 
     */
    float getTimePredictionForTravel(geometry_msgs::Point &goal);

    /**
     * @brief Get the Time Prediction For Travel object
     * 
     * @param goal 
     * @return float 
     */
    float getTimePredictionForTravel(geometry_msgs::Point &start_point, geometry_msgs::Point &end_point);

    /**
     * @brief Starts navigation and takes the robot to the location
     * 
     * @param goal_pose 
     * @return true     Request accepted 
     * @return false    Request rejected
     */
    bool goToPose(geometry_msgs::PoseStamped &goal_pose, bool wait_for_result = false);
    
    /**
     * @brief Starts navigation and takes the robot to the location
     * 
     * @param goal_pose 
     * @return true     Request accepted 
     * @return false    Request rejected
     */
    bool goToPoint(geometry_msgs::Point &goal, bool wait_for_result = false);
    
    /**
     * @brief Stops all the existing trajectories
     * 
     * @return true 
     * @return false 
     */
    bool stopRobot();

    /**
     * @brief Get the Robot Current Pose object
     * 
     * @return geometry_msgs::PoseStamped    Robot's current pose
     */
    geometry_msgs::PoseStamped getRobotCurrentPose();

    /**
     * @brief Get the Robot Current Pose object
     * 
     * @return geometry_msgs::PoseStamped    Robot's current pose
     */
    geometry_msgs::PoseStamped getOtherRobotCurrentPose(const std::string& robot_name);


    //////////// TO-DO ////////////
    /**
     * @brief In case the robot is stuck, this function should work as the recovery method
     *        Drive back a meter, turn PI to the direction of the next path
     *        Return after recovery complete
     *        
     * @return true 
     * @return false 
     */
    bool recoverRobot();

    bool navigationSucceeded();

    bool navigationDone();

  private:
    ros::NodeHandle nh_;
    // typedef for move base
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseTypedef;
    MoveBaseTypedef *move_base_client_;

    ros::ServiceClient move_base_planning_client_;
    ros::Publisher debug_generated_path_pub;
    ros::Subscriber testing_switch_trigger_sub;
    tf::TransformListener tf_listener_;
    
    move_base_msgs::MoveBaseGoal move_base_action_goal_;

    std::string namespace_prefix = ros::this_node::getNamespace();
    std::string logging_prefix_ = "[ "+ namespace_prefix +" | mdis_state_machine | move_base_interface ] ";
    std::string map_frame;
    std::string robot_frame;
    bool testing_mode;

    const int MAX_ATTEMPTS = 5;
    const float ROBOT_SPEED = (14/68.4); // Experimentally derived

    float calculatePathLength(const nav_msgs::Path& path);
    void testSwitchTrigCB(std_msgs::Empty msg);
    static bool testing_switch_trigger;

    geometry_msgs::PoseStamped transformTf2msg(const tf::StampedTransform& transform);
};