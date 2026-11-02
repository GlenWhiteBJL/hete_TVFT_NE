cmake_minimum_required(VERSION 3.0.2)
project(ne_tvft_experiment)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs sensor_msgs nav_msgs mavros_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/agent_node.py scripts/supervisor_node.py scripts/recorder_node.py
  scripts/analyze_experiment.py scripts/offline_demo.py scripts/mock_fleet.py scripts/preflight.py
  scripts/smoke_mock.py
  scripts/delay_sweep.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
