#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <chrono>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Command mapping for Joy messages
enum TrolleyCommand
{
  MOVE_FORWARD = 0,
  MOVE_BACKWARD = 1,
  MOVE_UP = 2,
  MOVE_DOWN = 3,
  OIL_CHOKE = 4,
  SET_BEEPER = 5,
  SET_TIMEOUT = 6,
  RESET_POSITION = 7,
  RESET_HEIGHT = 8,
  SET_VELOCITY = 9,
  MOVE_TO = 10,
  LIFT_TO = 11,
  STOP_ALL = 12,
  SET_WAKE_INTERVAL = 13,
  SET_CMD_SPACING = 14,
  REQUEST_UPDATE = 15,
  TRY_RECONNECT = 16,
  CONFIG_INFO = 17
};

enum TrolleyStatus
{
  NOT_CONNECTED = 0,
  STOPPED = 1,
  READY = 2,
  FORWARD = 3,
  BACKWARD = 4,
  UPWARD = 5,
  DOWNWARD = 6,
  MOVING_TO = 7,
  LIFTING_TO = 8,
  GOAL_REACHED = 9,
  CONNECTED = 10,
};

// Published and subscribed topic names
static const char* POSITION = "position";
static const char* HEIGHT = "height";
static const char* STATUS = "status";
static const char* ODOMETRY = "odom";
static const char* COMMANDS = "trolley_commands";
static const char* RAIL_FRONT = "on_rail_front";
static const char* RAIL_REAR = "on_rail_rear";

struct TrolleyState
{
  double position = 0.0; // in mm
  double height = 0.0; // in mm

  int horizontal_move_dir = 0;
  int vertical_move_dir = 0;

  std::optional<double> position_goal = std::nullopt;
  std::optional<double> height_goal = std::nullopt;

  TrolleyStatus status = READY;

  void changePos(int dir, std::optional<double> goal = std::nullopt)
  {
    position_goal = goal;
    horizontal_move_dir = dir;
  }

  void changeHeight(int dir, std::optional<double> goal = std::nullopt)
  {
    height_goal = goal;
    vertical_move_dir = dir;
  }

  std_msgs::String getStatus()
  {
    std_msgs::String msg;
    switch(status)
    {
      case NOT_CONNECTED: msg.data = "Not connected"; break;
      case STOPPED: msg.data = "Stopped"; break;
      case READY: msg.data = "Ready to operate"; break;
      case FORWARD: msg.data = "Moving Forward"; break;
      case BACKWARD: msg.data = "Moving Backwards"; break;
      case UPWARD: msg.data = "Going Up"; break;
      case DOWNWARD: msg.data = "Going Down"; break;
      case MOVING_TO: msg.data = "Moving to"; break;
      case LIFTING_TO: msg.data = "Lifting to"; break;
      case GOAL_REACHED: msg.data = "Goal reached"; break;
      case CONNECTED: msg.data = "Connected"; break;
    }
    return msg;
  }
};

class TrolleySimulation
{
private:
  bool is_initialized = false;

  TrolleyState state;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  ros::Publisher odom_pub;
  ros::Publisher status_pub;
  ros::Publisher rail_front_pub;
  ros::Publisher rail_rear_pub;

  tf2_ros::StaticTransformBroadcaster static_br;
  tf2_ros::TransformBroadcaster odom_br;
  tf2_ros::TransformBroadcaster pose_br;

  ros::Subscriber cmd_sub;

  boost::thread trolley_move_thread;
  boost::thread pub_status_thread;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface;

  // ROS parameters
  double update_time_interval; // TIME_INTERVAL
  std::string planning_group;
  double position_min; // in mm
  double position_max; // in mm
  //double position speed; // not implemented. const double POSITION_SPEED = 1000.0;
  double height_min; // in mm
  double height_max; // in mm
  //double height_speed; // not implemented. const double HEIGHT_SPEED = 1000.0;
  double moveit_goal_tolerance;
  double moveit_goal_joint_tolerance;
  bool publish_tf;
  bool publish_odom;
  bool print_position_height;
  std::string move_joint;
  std::string lift_joint;
  bool demo_mode;

public:
  TrolleySimulation() :
    tfBuffer(ros::Duration(30)),
    tfListener(tfBuffer),
    trolley_move_thread(&TrolleySimulation::trolleyMoveThread, this),
    pub_status_thread(&TrolleySimulation::pubStatusThread, this)
  {
    ros::NodeHandle nh, priv_nh("~");

    priv_nh.param<double>("update_time_interval", update_time_interval, 0.1);
    priv_nh.param<std::string>("planning_group", planning_group, "linear");
    priv_nh.param<double>("position_min", position_min, 0.0);     // in mm
    priv_nh.param<double>("position_max", position_max, 4000.0); // in mm. this value is copied from the robot model.
    priv_nh.param<double>("height_min", height_min, 0.0);         // in mm
    priv_nh.param<double>("height_max", height_max, 2200.0);      // in mm. this value is copied from the robot model.
    priv_nh.param<double>("moveit_goal_tolerance", moveit_goal_tolerance, 0.05);      // in meters
    priv_nh.param<double>("moveit_goal_joint_tolerance", moveit_goal_joint_tolerance, 0.05);      // in meters
    priv_nh.param<bool>("publish_tf", publish_tf, false);
    priv_nh.param<bool>("publish_odom", publish_odom, false);
    priv_nh.param<bool>("print_position_height", print_position_height, true);
    priv_nh.param<std::string>("move_joint", move_joint, "platform_move_joint");
    priv_nh.param<std::string>("lift_joint", lift_joint, "platform_lift_joint");
    priv_nh.param<bool>("demo_mode", demo_mode, false);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    status_pub = nh.advertise<std_msgs::String>("status", 10, true);
    rail_front_pub = nh.advertise<std_msgs::Bool>("on_rail_front", 10);
    rail_rear_pub = nh.advertise<std_msgs::Bool>("on_rail_rear", 10);

    cmd_sub = nh.subscribe("trolley_commands", 1, &TrolleySimulation::commandCb, this);

    move_group_interface.reset(new moveit::planning_interface::MoveGroupInterface(planning_group));
    move_group_interface->setGoalTolerance(moveit_goal_tolerance);
    move_group_interface->setGoalJointTolerance(moveit_goal_joint_tolerance);

    broadcastOdomFrame();
    status_pub.publish(state.getStatus());
    is_initialized = true;
  }

  void commandCb(const sensor_msgs::Joy::ConstPtr &command)
  {
    if (!is_initialized)
    {
      ROS_ERROR("Fake trolley node is not initialized. The command is not executed!");
      return;
    }

    auto contains = [](const auto &vec, const auto &el)
    {
      return std::find(vec.begin(), vec.end(), el) != vec.end();
    };

    if (contains(command->buttons, MOVE_FORWARD)) // 0
    {
      if (command->axes.size() == 0 ||  command->axes[0] == 0.f)
      {
        if (command->axes.size() == 0) ROS_ERROR("Axes is empty. Regarding the axis value as 0 which stops the trolley. This may create a problem with the real driver.");
        state.changePos(0);
        move_group_interface->stop(); // TODO: need to stop? maybe?
      }
      else
      {
        state.changePos(1);
        move_group_interface->setJointValueTarget(move_joint, position_max);
        move_group_interface->asyncMove();
        state.status = FORWARD;
      }
    }
    if (contains(command->buttons, MOVE_BACKWARD)) // 1
    {
      if (command->axes.size() == 0 ||  command->axes[0] == 0.f)
      {
        if (command->axes.size() == 0) ROS_ERROR("Axes is empty. Regarding the axis value as 0 which stops the trolley. This may create a problem with the real driver. ");
        state.changePos(0);
        move_group_interface->stop(); // TODO: need to stop? maybe?
      }
      else
      {
        state.changePos(-1);
        move_group_interface->setJointValueTarget(move_joint, position_min);
        move_group_interface->asyncMove();
        state.status = BACKWARD;
      }
    }
    if (contains(command->buttons, MOVE_UP)) // 2
    {
      if (command->axes.size() == 0 || command->axes[0] == 0.f)
      {
        if (command->axes.size() == 0) ROS_ERROR("Axes is empty. This may create a problem with the real driver!");
        state.changeHeight(0);
        move_group_interface->stop(); // TODO: need to stop? maybe?
      }
      else
      {
        state.changeHeight(1);
        move_group_interface->setJointValueTarget(lift_joint, height_max);
        move_group_interface->asyncMove();
        state.status = UPWARD;
      }
    }
    if (contains(command->buttons, MOVE_DOWN)) // 3
    {
      if (command->axes.size() == 0 || command->axes[0] == 0.f)
      {
        if (command->axes.size() == 0) ROS_ERROR("Axes is empty. This may create a problem with the real driver!");
        state.changeHeight(0);
        move_group_interface->stop(); // TODO: need to stop? maybe?
      }
      else
      {
        state.changeHeight(-1);
        move_group_interface->setJointValueTarget(lift_joint, height_min);
        move_group_interface->asyncMove();
        state.status = DOWNWARD;
      }
    }
    if (contains(command->buttons, OIL_CHOKE)) // 4
    {
      ROS_WARN("Not implemented command: OIL_CHOKE");
    }
    if (contains(command->buttons, SET_BEEPER)) // 5
    {
      ROS_WARN("Not implemented command: SET_BEEPER");
    }
    if (contains(command->buttons, SET_TIMEOUT)) // 6
    {
      ROS_WARN("Not implemented command: SET_TIMEOUT");
    }
    if (contains(command->buttons, RESET_POSITION)) // 7
    {
      ROS_WARN("Not implemented command: RESET_POSITION");
    }
    if (contains(command->buttons, RESET_HEIGHT)) // 8
    {
      ROS_WARN("Not implemented command: RESET_HEIGHT");
    }
    if (contains(command->buttons, SET_VELOCITY)) // 9
    {
      ROS_WARN("Not implemented command: SET_VELOCITY");
    }
    if (contains(command->buttons, MOVE_TO)) // 10
    {
      if (command->axes.size() <= 0)
      {
        ROS_ERROR("MOVE_TO command is called without axis! Axis should include position value in mm.");
      }
      else
      {
        double pos = (double)command->axes[0];
        if (pos < position_min || pos > position_max) ROS_ERROR("Given position value %lf is outside of the boundaries! Executing the command nevertheless. (min: %lf, max: %lf)", pos, position_min, position_max);
        state.changePos(pos > state.position ? 1 : -1, pos);
        move_group_interface->setJointValueTarget(move_joint, pos / 1000.0);
        move_group_interface->asyncMove();
        state.status = MOVING_TO;
      }
    }
    if (contains(command->buttons, LIFT_TO)) // 11
    {
      if (command->axes.size() <= 0)
      {
        ROS_ERROR("LIFT_TO command is called without axis! Axis should include height value in mm.");
      }
      else
      {
        double height = (double)command->axes[0];
        if (height < height_min || height > height_max) ROS_ERROR("Given height value %lf is outside of the boundaries! Executing the command nevertheless. (min: %lf, max: %lf)", height, height_min, height_max);
        state.changeHeight(height > state.height ? 1 : -1, height);
        move_group_interface->setJointValueTarget(lift_joint, height / 1000.0);
        move_group_interface->asyncMove();
        state.status = LIFTING_TO;
      }
    }
    if (contains(command->buttons, STOP_ALL)) // 12
    {
      move_group_interface->stop();
      state.status = STOPPED
    }
    if (contains(command->buttons, SET_WAKE_INTERVAL)) // 13
    {
      ROS_WARN("Not implemented command: SET_WAKE_INTERVAL");
    }
    if (contains(command->buttons, SET_CMD_SPACING)) // 14
    {
      ROS_WARN("Not implemented command: SET_CMD_SPACING");
    }
    if (contains(command->buttons, REQUEST_UPDATE)) // 15
    {
      ROS_WARN("Not implemented command: REQUEST_UPDATE");
    }
    if (contains(command->buttons, TRY_RECONNECT)) // 16
    {
      ROS_WARN("Not implemented command: TRY_RECONNECT");
    }
    if (contains(command->buttons, CONFIG_INFO)) // 17
    {
      ROS_WARN("Not implemented command: CONFIG_INFO");
    }
  }

  void trolleyMoveThread()
  {
    while (!is_initialized) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (ros::Rate rate(1.0/update_time_interval); ros::ok(); rate.sleep())
    {
      // Assign random joint targets in the demo mode!
      if (demo_mode)
      {
        move_group_interface->setRandomTarget();
        move_group_interface->move();
      }

    }
  }

  void pubStatusThread()
  {
    while (!is_initialized) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (ros::Rate rate(1.0/update_time_interval); ros::ok(); rate.sleep())
    {
      auto joint_values = move_group_interface->getCurrentJointValues();
      auto joint_names = move_group_interface->getJointNames();
      for (int i=0; i<joint_values.size(); i++)
      {
        if (joint_names[i].compare(move_joint) == 0)
        {
          state.position = joint_values[i] * 1000.0;
        }
        else if  (joint_names[i].compare(lift_joint) == 0)
        {
          state.height = joint_values[i] * 1000.0;
        }
      }

      if (print_position_height)
      {
        ROS_INFO_STREAM_THROTTLE(1.0, "Trolley Position: " << state.position << "mm Trolley Height: " << state.height << "mm");
      }

      status_pub.publish(state.getStatus());
      broadcastOdometry();
    }
  }

  void broadcastOdomFrame()
  {
    if (publish_tf)
    {
      geometry_msgs::TransformStamped t_odom;
      t_odom.header.stamp = ros::Time::now();
      t_odom.transform.rotation.w = 1;
      t_odom.header.frame_id = "map";
      t_odom.child_frame_id = "odom";
      static_br.sendTransform(t_odom);
    }
  }

  void broadcastOdometry()
  {
    ros::Time current_time = ros::Time::now();
    geometry_msgs::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = state.position / 1000.0;
    t.transform.rotation.w = 1;

    if (publish_tf)
    {
      odom_br.sendTransform(t);
    }

    if (publish_odom)
    {
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = current_time;
      odom_msg.header.frame_id = t.header.frame_id;
      odom_msg.child_frame_id = t.child_frame_id;
      odom_msg.pose.pose.position.x = t.transform.translation.x;
      odom_msg.pose.pose.orientation.w = t.transform.rotation.w;
      odom_pub.publish(odom_msg);
    }

  }
};
