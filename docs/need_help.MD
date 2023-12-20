# Helpful Tips for ESDC Machine

## COM Ports are invalid.
Check ports.py and the devices in the device manager are matching. An easy way to check the COM port is to
open device manager, and then unplug/replug in one system from the ESDC machine, waiting to see which COM disappears.
Reconnect the device, and take note and change default ports.py as needed. (src/robot/ports.py)

## Stepper motors are not homed.
If the stepper motors are not properly homed, restart the software as this forces a homing sequence. The stepper
motors require some cycles to warm up before proper homing is achieved.

If any axis is not properly homed, a manual adjustment can be made in the home position, as long as this is done
AFTER the homing sequence after opening the software.

## Robotic arm is not moving as expected
Check if the robotic arm has flashing red servo motors. If there are flashing red servo motors, they have run into 
an over-torque condition. Shutdown the software, unplug the robotic arm from the side of the ESDC.

## The point cloud is unexpected.
Due to the nature of point cloud generation through pybullet's raycasting system, some point cloud generations will
not be appropriate for a scan. Editing the settings for the point cloud generator will help in creating a more uniform
point cloud.

## Collision error.
If the simulation is displaying COLLISION ERROR, this means that the robot has detected a collision, whether if thats
from a simulated movement, or the torque detectors on the servos. The robot will need to be homed and the software
reset.



# Misc tips.

* Arduino code for the feather0 is stored in external/servo_code/ and is flashing using the Arduino software.
* GUIs were designed in PyQT5's designer, this is found in the lib folder (file search designer.exe), all raw UI files are in src/gui/ui_files/
* Postprocessing script is housed in src/sim/scan_algo/PostProcessor.py, please note that the coordinate system exported from probing is rectangular, and a conversion to polar might be necessary
* The simulation is scaled 2x for floating point errors. This can be changed in src/sim/sim_constants.py
* All simulated models are in data/sim/ are in the URDF file format. This is extensively documented on various forms.
* main.py is the main file that should be run. The PyCharm instance left on the computer should have a launcher labeled "Launch Software"
* This software is built for Python 3.10 as this is the last open3d version easily accessible.