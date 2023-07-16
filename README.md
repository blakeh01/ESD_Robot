# ESD_Robot

TODO doc: (might be slightly out of date) 
  https://docs.google.com/document/d/1vrp94aHgXG1E4lBfs8wTVxMaC26XkYNUi0QIz4ZkVTM/edit?usp=sharing

General TODOs:
  * Scanning system has a way to communicate with the laser scanner but no way to plot 3D data. Need to add code to talk to steppers to sweep along the object and rotate it to scan all sides.
  * Fix inverse kinematics in pybullet to iterate through goal command until a tolerance position is reached.
  * Probing algorithm: how to handle complex objects? sides vs. arcs in creating trajectories
  * Trajectories and motion planning: need to parse an entire probe plan before even executing due to the nature of G-code.
  * Force sensor: (code written to read forces) need to take measurements and create an algorithm to detect a sudden shock from a collision.
  * Probe grounding: need code to back off robot and ground. As all movement will have to be pre-processed, will need to also be preprocessed and inserted with probing code.
