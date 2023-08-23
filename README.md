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

Some general docs:
  * IMPORTANT PATHS:
      * deprecated - random classes that might have some valuable info on algorithms/code ive already attempted.
      * gui - classes associated with GUIs. UI files are included such that you can edit them with pyqt5's designer.
      * robot - robot-associated classes (rbt controllers, constants)
      * sim - classes used in creating and setting up the pybullet simulation. Also contains command control for running tests on the robot/simulation.
      * testcases - some random classes that are valuable in the final product (ie. a class to communicate with the laser scanner)
      * util - random utilty classes
      * Controller.py - important class that ties together the robot and the simulation
      * main.py - RUN THIS CLASS TO RUN PROGRAM... IT CREATES GUI AND UPDATE TIMER FOR THE ENTIRE PROGRAM. 
