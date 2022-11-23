#include "trolley_simulation/trolley_simulation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trollomatic");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  double update_time_interval = nhp.param<double>("update_time_interval", 0.1);
  bool demo_mode = nhp.param<bool>("demo_mode", false);

  TrolleySimulation sim;
  for (ros::Rate rate(1.0/update_time_interval); ros::ok(); rate.sleep())
  {
    if (demo_mode)
      sim.trolleyMove();

    sim.pubStatus();
  }
}
