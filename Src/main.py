'''

    maingui.py creates the update thread and GUI for the program.
    Run this file to start the program...

'''
import random
import sys
import os
import time

import pybullet_planning as pp
import pybullet as p

import Src.sim.ObjectVisualizer
import win32gui
from PyQt5 import QtGui
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QWizard
)
from Src.Controller import Controller
from Src.gui.dialogs.dialog_robot_info import DialogRobotInfo
from Src.gui.dialogs.dialog_normal_generator import DialogNormalGenerator
from Src.gui.dialogs.dialog_probe_profile import DialogProbeProfile
from Src.gui.dialogs.dialog_charge_object import DialogChargeObject
from Src.gui.object_wizard import Ui_ObjectWizard
from Src.gui.main_window import Ui_MainWindow

import pyqtgraph as pg
import numpy as np

DATA_DIR = os.path.join(os.path.abspath('../'), "Data", "sim")

URDF_RBT = os.path.join(DATA_DIR, "urdf", "rx200pantex.urdf")
URDF_PLAT_NO_OBJ = os.path.join(DATA_DIR, "urdf", "actuated_platform_no_obj.urdf")
URDF_PLAT = os.path.join(DATA_DIR, "urdf", "actuated_platform.urdf")
URDF_OBJ = os.path.join(DATA_DIR, "urdf", "object.urdf")

class UpdateThread(QThread):
    update_frame = pyqtSignal()

    def __init__(self, controller_instance):
        QThread.__init__(self)
        self.is_running = False
        self.controller_instance = controller_instance

    def run(self):
        self.is_running = True

        while self.is_running:
            self.controller_instance.send_update()
            self.update_frame.emit()

    def stop(self):
        self.is_running = False
        self.quit()


class MainWindow(QMainWindow, Ui_MainWindow):

    def __init__(self, obj_wiz_data, parent=None):
        super().__init__(parent)

        self.setupUi(self)
        self.setWindowTitle("Controller")

        # Update our GUI to keep up with the controller flags
        self._gui_timer = QTimer(self)
        self._gui_timer.timeout.connect(self.update_gui)

        # New controller that is stored within our main window.
        print("[MAIN] Initializing Controller...")
        self.controller = Controller(self, obj_wiz_data)

        # Object visualizer
        # print("[MAIN] Creating Open3D Visualizer...")
        # self.o3d_visualizer = Src.sim.ObjectVisualizer.ObjectVisualizer()

        self.update_thread = UpdateThread(self.controller)
        self.update_thread.update_frame.connect(self.update)
        self.update_thread.start()  # start controller thread
        self._gui_timer.start(32)  # start GUI update thread ~ 60 FPS

        # Program Signals:
        self.btn_shutdown_program.clicked.connect(self.stop_program)  # shutdown button
        self.btn_edit_consts.clicked.connect(self.edit_constants)  # set probe pos dialog TODO
        self.btn_rbt_info.clicked.connect(self.dialog_rbt_info)  # robot 'info' button
        self.btn_normal_generator.clicked.connect(self.dialog_normal_generator)
        self.btn_probe_setup.clicked.connect(self.dialog_probe_setup)
        self.btn_start_probing.clicked.connect(self.begin_probe_flow)

        # Set update rate to given value.
        self.lbl_updaterate.setText(str(round(1 / self.controller.update_rate)) + " /s")

        # Move windows (o3d) to proper tabs
        # hwnd = win32gui.FindWindowEx(0, 0, None, "Open3D")
        # self.window = QtGui.QWindow.fromWinId(hwnd)
        # self.windowcontainer = self.createWindowContainer(self.window, self.widget_open3d)
        # self.windowcontainer.setMinimumSize(1221, 761)

        # Moves pybullet to the GUI... however it disables all controls from the user
        # todo figure out a fix^
        self.embed_pysim()

        print("[MAIN] Initialized Program! Ready for action...")

    def update_gui(self):
        # Update simulation isRunning label
        if self.controller.simulation_instance.can_run:
            self.lbl_sim_status.setStyleSheet("background-color: lightgreen")
            self.lbl_sim_status.setText("RUNNING")
        else:
            self.lbl_sim_status.setStyleSheet("background-color: red")
            self.lbl_sim_status.setText("OFF")

        # Update collision label
        if self.controller.obj_distance <= 0.003:
            self.lbl_distance_rbt_obj.setStyleSheet("background-color: red")
            self.controller.simulation_instance.col_flag = True
        elif 0 < self.controller.obj_distance < 0.01:
            self.lbl_distance_rbt_obj.setStyleSheet("background-color: orange")
        else:
            self.lbl_distance_rbt_obj.setStyleSheet("background-color: lightgreen")
        self.lbl_distance_rbt_obj.setText(str(round(self.controller.obj_distance, 5)))

        # Update time labels
        self.lbl_dt.setText(str(round(self.controller.dt, 6)))
        self.lbl_time.setText(str(round(self.controller.time_elapsed, 6)))

        # Update probe voltage label
        # TODO UNCOMMENT self.lbl_probe_voltage.setText(str(round(self.controller.probe_voltage, 5)))

        # If there are normals, allow for probe flow generation
        self.btn_probe_setup.setEnabled(not self.controller.simulation_instance.normal_point_cloud is None)

        # if theere is a probe plan, allow to execute
        self.btn_start_probing.setEnabled(not self.controller.simulation_instance.cur_probe_flow is None)

        if self.controller.simulation_instance.cur_probe_flow is not None:
            self.probe_progress_bar.setValue(self.controller.simulation_instance.cur_probe_flow.probe_percentage)

    def embed_pysim(self):
        hwnd = win32gui.FindWindowEx(0, 0, None, "Bullet Physics ExampleBrowser using OpenGL3+ [btgl] Release build")
        self.window = QtGui.QWindow.fromWinId(hwnd)
        self.windowcontainer = self.createWindowContainer(self.window, self.widget_pybullet)
        self.windowcontainer.setMinimumSize(1220, 900)

    def plot_slice(self, slice, path, active_idx=0):
        x_values = [point.pos[0] for point in path]
        y_values = [point.pos[1] for point in path]

        self.widget_slice_disp.plot(x=x_values, y=y_values, pen='r', symbol='o', symbolPen='r', symbolBrush=(0, 0, 255), symbolSize=10)

        for i, point in enumerate(path):
            x, y, z = point.pos
            angle = np.arctan2(point.direction[1], point.direction[0])
            x_end = x + 0.025 * np.cos(angle)
            y_end = y + 0.025 * np.sin(angle)
            self.widget_slice_disp.plot([x, x_end], [y, y_end], pen='b')

            text = pg.TextItem(f'{i}', anchor=(0, 1), color=(255, 255, 255), fill=(0, 0, 0, 0))
            text.setPos(x, y)
            self.widget_slice_disp.addItem(text)

    def edit_constants(self):
        pass

    def stop_program(self):
        print("[MAIN] Stopping Program!")
        self.window.setParent(None)
        self.update_thread.stop()
        self.controller.shutdown()
        # self.o3d_visualizer.visualizer.destroy_window()
        exit()

    def begin_probe_flow(self):
        self.controller.simulation_instance.can_execute_flow = True

    def dialog_probe_setup(self):
        _dlg = DialogProbeProfile(self)
        _dlg.exec()

    def dialog_rbt_info(self):
        _dlg = DialogRobotInfo(self)
        _dlg.exec()

    def dialog_normal_generator(self):
        _dlg = DialogNormalGenerator(self)
        _dlg.exec()

    def show_information_box(self, title, content, icon=QMessageBox.Information):
        msg = QMessageBox()
        msg.setText(content)
        msg.setIcon(icon)
        msg.setWindowTitle(title)
        msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        msg.exec()

    def closeEvent(self, a0: QtGui.QCloseEvent):
        self.update_thread.quit()
        self.stop_program()
        a0.accept()


class ObjectWizard(QWizard):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_ObjectWizard()
        self.ui.setupUi(self)

        self.sim_robot = None
        self.sim_platform = None
        self.obj = None

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.update)
        self.gui_timer.start(25)  # start GUI update thread ~ 40 FPS

        self.has_init = False

        # todo if time, configurable for calibration perhaps?
        self.SIM_ROBOT_OFFSET = np.dot(2, [-0.40, 0, 0])
        self.SIM_PLATFORM_OFFSET = np.dot(2, [0, 0, 0])

        self.ui.sbox_rbt_offset_x.valueChanged.connect(self.update_rbt_offset)
        self.ui.sbox_rbt_offset_y.valueChanged.connect(self.update_rbt_offset)
        self.ui.sbox_rbt_offset_z.valueChanged.connect(self.update_rbt_offset)

        self.ui.sbox_offset_x.valueChanged.connect(self.update_obj_offset)
        self.ui.sbox_offset_y.valueChanged.connect(self.update_obj_offset)
        self.ui.sbox_offset_z.valueChanged.connect(self.update_obj_offset)

    def update(self):
        if self.currentId() == 5 and not self.has_init:
            pp.connect(True)
            p.setGravity(0, 0, 0)
            # Add robot into PyBullet environment
            self.sim_robot = pp.load_pybullet(URDF_RBT, fixed_base=True, scale=2)
            p.resetBasePositionAndOrientation(self.sim_robot, self.SIM_ROBOT_OFFSET, p.getQuaternionFromEuler([0, 0, 0]))
            print(f"[SIM] Initialized robot with ID: {self.sim_robot}")

            # Add center platform AND OBJECT into PyBullet environment
            self.sim_platform = pp.load_pybullet(URDF_PLAT_NO_OBJ, fixed_base=True, scale=2)
            p.resetBasePositionAndOrientation(self.sim_platform, self.SIM_PLATFORM_OFFSET,
                                              p.getQuaternionFromEuler([0, 0, 0]))
            print(f"[SIM] Initialized platform with ID: {self.sim_platform}")
            self.obj = pp.load_pybullet(URDF_OBJ, scale=2)
            p.resetBasePositionAndOrientation(self.obj, np.dot(2, [0, 0, .15875]), [0, 0, 0, 1])

            self.obj_joint_offset = [0, 0, .16*2]
            self.obj_const = p.createConstraint(parentBodyUniqueId=self.sim_platform, parentLinkIndex=1, childBodyUniqueId=self.obj,
                               childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                               parentFramePosition=self.obj_joint_offset, childFramePosition=[0, 0, 0])

            pp.set_camera_pose(tuple(np.array((0, 0, 0.25)) + np.array([0.25, -0.25, 0.25])), (0, 0, 0.25))
            self.has_init = True

        if pp.is_connected():
            pp.step_simulation()
            time.sleep(0.01)

    def update_rbt_offset(self):
        self.SIM_ROBOT_OFFSET = np.dot(2, [self.ui.sbox_rbt_offset_x.value()/1000, self.ui.sbox_rbt_offset_y.value()/1000, self.ui.sbox_rbt_offset_z.value()/1000])
        p.resetBasePositionAndOrientation(self.sim_robot, self.SIM_ROBOT_OFFSET, p.getQuaternionFromEuler([0, 0, 0]))

    def update_obj_offset(self):
        self.obj_joint_offset = np.dot(2, [self.ui.sbox_offset_x.value()/1000, self.ui.sbox_offset_y.value()/1000, .16 + self.ui.sbox_offset_z.value()/1000])
        p.removeConstraint(self.obj_const)
        self.obj_const = p.createConstraint(parentBodyUniqueId=self.sim_platform, parentLinkIndex=1,
                                            childBodyUniqueId=self.obj,
                                            childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                                            parentFramePosition=self.obj_joint_offset, childFramePosition=[0, 0, 0])

skip_wiz = False

def show_main_form(data):
    main = MainWindow(data)
    main.show()

def show_setup_wizard():
    if skip_wiz:
        show_main_form(None)
        return
    wiz = ObjectWizard()
    res = wiz.exec_()

    if res == QWizard.Accepted:
        pp.disconnect()

        #overwrite offsets
        (x, y, z) = wiz.obj_joint_offset
        URDF_OFFSET_LINE_NUM = 77

        with open(URDF_PLAT, 'r') as file:
            content = file.readlines()
            str_write = f'    <origin xyz="{x/2} {y/2} {z/2}"/>\n'
            content[URDF_OFFSET_LINE_NUM - 1] = str_write  # replace contents with modified offset.

        with open(URDF_PLAT, 'w') as file:
            file.writelines(content)

        show_main_form([wiz.SIM_ROBOT_OFFSET, wiz.obj_joint_offset])

if __name__ == '__main__':
    # current_dir = os.getcwd()
    # print(current_dir)
    #
    # log_file_name = "console_log.txt"
    # log_file_path = os.path.join(current_dir, log_file_name)
    # sys.stdout = open(log_file_path, "w")

    app = QApplication(sys.argv)
    show_setup_wizard()
    sys.exit(app.exec_())
