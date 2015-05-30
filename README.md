Software_IGVC [![Build Status](http://jenkins.chicagoedt.org/job/Software_IGVC_Upstream/badge/icon)](http://jenkins.chicagoedt.org/job/Software_IGVC_Upstream/)
=============

Run the Real Robot
------------
**1)** The configuration command below will:
  - start all the driver nodes (roboteq, imu, sicktim, gps)
  - start the localization nodes (2x ekf_localization_node, navsat_transform_node)
  - start the navigation stack (move_base)


---> `roslaunch scipio_2dnav start.launch`


**2)** Once configuration is launch, launch the state machine to send gps goals:
  - It will wait 5 seconds after start, before sending the first goal.

---> `roslaunch state_machine start.launch`



Run the Simulation
------------
**1)** The simulator launch command below will:
  - Completely setup everything you need to simply just send goals to the robot
  - Depending on your computer speed, it may take up to 15 seconds to load gazebo

---> `roslaunch scipio_simulation gazebo.launch`

**2)** Once you see the IGVC course in Gazebo:
  - Start state machine to send goals

---> `roslaunch state_machine start.launch`
