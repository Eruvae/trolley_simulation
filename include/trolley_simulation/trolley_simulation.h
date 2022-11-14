#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

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
  const double POSITION_MIN = 0.0;
  const double POSITION_MAX = 20000.0;
  const double POSITION_SPEED = 1000.0;

  const double HEIGHT_MIN = 0.0;
  const double HEIGHT_MAX = 80000.0;
  const double HEIGHT_SPEED = 1000.0;

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

  void checkPosMinMax()
  {
    bool reached = false;
    if ((reached = (position >= POSITION_MAX))) position = POSITION_MAX;
    else if ((reached = (position <= POSITION_MIN))) position = POSITION_MIN;
    if (reached) changePos(0);
  }

  void checkHeightMinMax()
  {
    bool reached = false;
    if ((reached = (height >= HEIGHT_MAX))) height = HEIGHT_MAX;
    else if ((reached = (height <= HEIGHT_MIN))) height = HEIGHT_MIN;
    if (reached) changeHeight(0);
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
  const double TIME_INTERVAL = 0.1;

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

  const std::string PLANNING_GROUP = "linear"; // TODO: parameterize
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

public:
  TrolleySimulation() :
    tfBuffer(ros::Duration(30)),
    tfListener(tfBuffer),
    trolley_move_thread(&TrolleySimulation::trolleyMoveThread, this),
    move_group_interface(PLANNING_GROUP)
  {
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    status_pub = nh.advertise<std_msgs::String>("status", 10, true);
    rail_front_pub = nh.advertise<std_msgs::Bool>("on_rail_front", 10);
    rail_rear_pub = nh.advertise<std_msgs::Bool>("on_rail_rear", 10);

    cmd_sub = nh.subscribe("trolley_commands", 1, &TrolleySimulation::commandCb, this);

    status_pub.publish(state.getStatus());

    //ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    //std::copy(move_group_interface.getJointModelGroupNames().begin(),
    //          move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    //broadcastOdomFrame();
    is_initialized = true;
  }
/*
  if TrolleyCommands.MOVE_FORWARD in command.buttons:
    self.trolley.moveForward(False if command.axes[0] == 0.0 else True)

  if TrolleyCommands.MOVE_BACKWARD in command.buttons:
    self.trolley.moveBackward(False if command.axes[0] == 0.0 else True)

  if TrolleyCommands.MOVE_UP in command.buttons:
    self.trolley.moveUp(False if command.axes[0] == 0.0 else True)

  if TrolleyCommands.MOVE_DOWN in command.buttons:
    self.trolley.moveDown(False if command.axes[0] == 0.0 else True)

  if TrolleyCommands.OIL_CHOKE in command.buttons:
    self.trolley.oilChoke(False if command.axes[0] == 0.0 else True)

  if TrolleyCommands.SET_BEEPER in command.buttons:
    self.trolley.setBeeper(False if command.axes[0] == 0.0 else True)

  if TrolleyCommands.SET_TIMEOUT in command.buttons:
    self.trolley.setTimeout(int(command.axes[0]))

  if TrolleyCommands.RESET_POSITION in command.buttons:
    self.trolley.resetPosition()

  if TrolleyCommands.RESET_HEIGHT in command.buttons:
    self.trolley.resetHeight()

  if TrolleyCommands.SET_VELOCITY in command.buttons:
    self.trolley.setVelocity(int(command.axes[0]))

  if TrolleyCommands.MOVE_TO in command.buttons:
    self.trolley.moveTo(int(command.axes[0]))

  if TrolleyCommands.LIFT_TO in command.buttons:
    self.trolley.liftTo(int(command.axes[0]))

  if TrolleyCommands.STOP_ALL in command.buttons:
    self.trolley.stopAll()

  if TrolleyCommands.SET_WAKE_INTERVAL in command.buttons:
    self.trolley.setWakeInterval(int(command.axes[0]))

  if TrolleyCommands.SET_CMD_SPACING in command.buttons:
    self.trolley.setCmdSpacing(int(command.axes[0]))

  if TrolleyCommands.REQUEST_UPDATE in command.buttons:
    self.trolley.requestPositionUpdate()
    sleep(0.1)
    self.trolley.requestHeightUpdate()
    sleep(0.1)
    self.trolley.requestSensorsUpdate()
    self.trolley.isReady()

    self.status_pub.publish(self.trolley.getStatus())
    print('update published')

  if TrolleyCommands.TRY_RECONNECT in command.buttons:
    self.trolley.tryReconnect()

  if TrolleyCommands.CONFIG_INFO in command.buttons:
    self.trolley.configInfo(False if command.axes[0] == 0.0 else True)
  */
  void commandCb(const sensor_msgs::Joy::ConstPtr &command)
  {
    auto contains = [](const auto &vec, const auto &el)
    {
      return std::find(vec.begin(), vec.end(), el) != vec.end();
    };

    if (contains(command->buttons, MOVE_FORWARD)) // 0
    {
      if (command->axes[0] == 0.f)
        state.changePos(0);
      else
        state.changePos(1);
    }
    if (contains(command->buttons, MOVE_BACKWARD)) // 1
    {
      if (command->axes[0] == 0.f)
        state.changePos(0);
      else
        state.changePos(-1);
    }
    if (contains(command->buttons, MOVE_UP)) // 2
    {
      if (command->axes[0] == 0.f)
        state.changeHeight(0);
      else
        state.changeHeight(1);
    }
    if (contains(command->buttons, MOVE_DOWN)) // 3
    {
      if (command->axes[0] == 0.f)
        state.changeHeight(0);
      else
        state.changeHeight(-1);
    }
    if (contains(command->buttons, OIL_CHOKE)) // 4
    {

    }
    if (contains(command->buttons, SET_BEEPER)) // 5
    {

    }
    if (contains(command->buttons, SET_TIMEOUT)) // 6
    {

    }
    if (contains(command->buttons, RESET_POSITION)) // 7
    {

    }
    if (contains(command->buttons, RESET_HEIGHT)) // 8
    {

    }
    if (contains(command->buttons, SET_VELOCITY)) // 9
    {

    }
    if (contains(command->buttons, MOVE_TO)) // 10
    {
      double pos = (double)command->axes[0];
      state.changePos(pos > state.position ? 1 : -1, pos);
    }
    if (contains(command->buttons, LIFT_TO)) // 11
    {
      double height = (double)command->axes[0];
      state.changeHeight(height > state.height ? 1 : -1, height);
    }
    if (contains(command->buttons, STOP_ALL)) // 12
    {

    }
    if (contains(command->buttons, SET_WAKE_INTERVAL)) // 13
    {

    }
    if (contains(command->buttons, SET_CMD_SPACING)) // 14
    {

    }
    if (contains(command->buttons, REQUEST_UPDATE)) // 15
    {

    }
    if (contains(command->buttons, TRY_RECONNECT)) // 16
    {

    }
    if (contains(command->buttons, CONFIG_INFO)) // 17
    {

    }
  }

  void trolleyMoveThread()
  {
    for (ros::Rate r(1.0/TIME_INTERVAL); ros::ok(); r.sleep())
    {
      if (!is_initialized) continue;

      if (state.position_goal)
      {
        double dist = *state.position_goal- state.position;
        if (std::abs(dist) < TIME_INTERVAL * state.POSITION_SPEED)
        {
          state.position = *state.position_goal;
          state.changePos(0);
        }
        else
        {
          state.position += std::copysign(TIME_INTERVAL * state.POSITION_SPEED, dist);
        }
        state.checkPosMinMax();
      }
      else if (state.horizontal_move_dir != 0)
      {
        state.position += state.horizontal_move_dir * TIME_INTERVAL * state.POSITION_SPEED;
        state.checkPosMinMax();
      }

      if (state.height_goal)
      {
        double dist = *state.height_goal- state.height;
        if (std::abs(dist) < TIME_INTERVAL * state.HEIGHT_SPEED)
        {
          state.height = *state.height_goal;
          state.changeHeight(0);
        }
        else
        {
          state.height += std::copysign(TIME_INTERVAL * state.HEIGHT_SPEED, dist);
        }
        state.checkHeightMinMax();
      }
      else if (state.vertical_move_dir != 0)
      {
        state.height += state.vertical_move_dir * TIME_INTERVAL * state.HEIGHT_SPEED;
        state.checkHeightMinMax();
      }

      // DEBUG
      move_group_interface.setJointValueTarget("platform_move_joint", 1.0);
      move_group_interface.setJointValueTarget("platform_lift_joint", 0.5);
      move_group_interface.asyncMove();

      auto joint_values = move_group_interface.getCurrentJointValues();
      for (int i=0; i<joint_values.size(); i++)
      {
        ROS_FATAL("joint-%d: %f", i, joint_values[i]);
      }

      ROS_INFO_STREAM("Current pose: " << state.position << "; Height: " << state.height);
    }
  }

  void pubStatusThread()
  {
    /*rate = rospy.Rate(10) # 10hz
    # initial status and pose publishing
    self.status_pub.publish(self.trolley.getStatus())
    self.updateHeight()
    self.broadcastOdometry()
    if(self.tracker_enable):
      self.broadcastTrackerPose()

    while not rospy.is_shutdown():
      self.updateRailSensor()
      # Publish height transform and status when they change
      if self.trolley.newHeight():
        self.updateHeight()
      if self.trolley.newStatus():
        self.status_pub.publish(self.trolley.getStatus())
      # pub rail sensor status
      self.rail_front_pub.publish(self.trolley.getFrontSensor())
      self.rail_rear_pub.publish(self.trolley.getRearSensor())
      # Broadcast odometry
      self.broadcastOdometry()
      if(self.tracker_enable):
        self.broadcastTrackerPose()

      if self.launch_record:
        self.launch_record = False
        self.start_recording()

      if not self.recording and self.rail_sensors['came_off']:
        self.rail_sensors['came_off'] = False

      if self.recording and self.auto_stop_record and self.rail_sensors['came_off']:
        self.stop_recording()

      rate.sleep()

    self.stop_recording()*/
    for (ros::Rate rate(10); ros::ok(); rate.sleep())
    {
      status_pub.publish(state.getStatus());
      //broadcastOdometry();
    }
  }

  void broadcastOdomFrame()
  {
    geometry_msgs::TransformStamped t_odom;
    t_odom.header.stamp = ros::Time::now();
    t_odom.transform.rotation.w = 1;
    t_odom.header.frame_id = "map";
    t_odom.child_frame_id = "odom";
    static_br.sendTransform(t_odom);
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
    odom_br.sendTransform(t);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = t.header.frame_id;
    odom_msg.child_frame_id = t.child_frame_id;
    odom_msg.pose.pose.position.x = t.transform.translation.x;
    odom_msg.pose.pose.orientation.w = t.transform.rotation.w;
    odom_pub.publish(odom_msg);
  }
};
