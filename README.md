# ESD_Robot

A PyBullet-based simulator using custom hardware to scan/probe objects.

Some general docs:
  * IMPORTANT PATHS:
      * gui - classes associated with GUIs. UI files are included such that you can edit them with pyqt5's designer.
      * robot - robot-associated classes (rbt controllers, constants)
      * sim - classes used in creating and setting up the pybullet simulation. Also contains command control for running tests on the robot/simulation.
      * util - random utilty classes
      * Controller.py - important class that ties together the robot and the simulation
      * main.py - RUN THIS CLASS TO RUN PROGRAM... IT CREATES GUI AND UPDATE TIMER FOR THE ENTIRE PROGRAM. 
