#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

// Command mapping for Joy messages
enum TrolleyCommands
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

// Published and subscribed topic names
static const char* POSITION = "position";
static const char* HEIGHT = "height";
static const char* STATUS = "status";
static const char* ODOMETRY = "odom";
static const char* COMMANDS = "trolley_commands";
static const char* RAIL_FRONT = "on_rail_front";
static const char* RAIL_REAR = "on_rail_rear";

class TrolleySimulation
{
private:
  const double POSITION_MIN = 0.0;
  const double POSITION_MAX = 20.0;
  const double POSITION_SPEED = 1.0;

  const double HEIGHT_MIN = 0.0;
  const double HEIGHT_MAX = 8.0;
  const double HEIGHT_SPEED = 1.0;

  double position;
  double height;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  ros::Publisher odom_pub;
  ros::Publisher status_pub;
  ros::Publisher rail_front_pub;
  ros::Publisher rail_rear_pub;

  ros::Subscriber cmd_sub;

public:
  TrolleySimulation() :
    tfBuffer(ros::Duration(30)),
    tfListener(tfBuffer)
  {
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    status_pub = nh.advertise<std_msgs::String>("status", 10);
    rail_front_pub = nh.advertise<std_msgs::Bool>("on_rail_front", 10);
    rail_rear_pub = nh.advertise<std_msgs::Bool>("on_rail_rear", 10);

    cmd_sub = nh.subscribe("trolley_commands", 1, &TrolleySimulation::commandCb, this);
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

    if (contains(command->buttons, MOVE_FORWARD))
    {

    }
    if (contains(command->buttons, MOVE_BACKWARD))
    {

    }
    if (contains(command->buttons, MOVE_UP))
    {

    }
    if (contains(command->buttons, MOVE_DOWN))
    {

    }
    if (contains(command->buttons, OIL_CHOKE))
    {

    }
    if (contains(command->buttons, SET_BEEPER))
    {

    }
    if (contains(command->buttons, SET_TIMEOUT))
    {

    }
    if (contains(command->buttons, RESET_POSITION))
    {

    }
    if (contains(command->buttons, RESET_HEIGHT))
    {

    }
    if (contains(command->buttons, SET_VELOCITY))
    {

    }
    if (contains(command->buttons, MOVE_TO))
    {

    }
    if (contains(command->buttons, LIFT_TO))
    {

    }
    if (contains(command->buttons, STOP_ALL))
    {

    }
  }

  void trolleyMoveThread()
  {

  }

};
