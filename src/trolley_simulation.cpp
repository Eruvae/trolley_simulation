#include "trolley_simulation/trolley_simulation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trollomatic");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  TrolleySimulation sim;
  sim.pubStatusThread();
}
