#include "robot_gui/robot_gui_class.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  
  RobotGUI robot_gui;
  robot_gui.run();
  
  return 0;
}