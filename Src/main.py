'''

    maingui.py creates the update thread and GUI for the program.
    Run this file to start the program...

'''
import os
import sys
import threading
import time

import numpy as np
import pybullet as p
import pybullet_planning as pp
import pyqtgraph as pg
import win32gui
from PyQt5 import QtGui
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QWizard, QFileDialog, QErrorMessage, QWidget, QVBoxLayout
)
from Src.Controller import Controller
from Src.gui.dialogs.dialog_com_port import DialogComPorts
from Src.gui.dialogs.dialog_normal_generator import DialogNormalGenerator
from Src.gui.dialogs.dialog_probe_profile import DialogProbeProfile
from Src.gui.dialogs.dialog_robot_info import DialogRobotInfo
from Src.gui.main_window import Ui_MainWindow
from Src.gui.object_wizard import Ui_ObjectWizard
from Src.robot.SerialMonitor import StepperHandler
from Src.robot.arm.RobotHandler import RobotHandler
from Src.robot.ports import PortConfiguration
from Src.robot.scanner.Scanner import Scanner, PrimitiveScan
from Src.sim.ObjectVisualizer import ObjectVisualizer

import configparser

DATA_DIR = os.path.join(os.path.abspath('../'), "Data", "sim")

URDF_RBT = os.path.join(DATA_DIR, "urdf", "rx200pantex.urdf")
URDF_PLAT_NO_OBJ = os.path.join(DATA_DIR, "urdf", "actuated_platform_no_obj.urdf")
URDF_PLAT = os.path.join(DATA_DIR, "urdf", "actuated_platform.urdf")
URDF_OBJ = os.path.join(DATA_DIR, "urdf", "object.urdf")

port_config = PortConfiguration()

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
        self.controller = Controller(self, port_config, obj_wiz_data)

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
        self.btn_charge_done.clicked.connect(self.advance_flow)
        self.btn_sim_terminate.clicked.connect(self.sim_stop)
        self.btn.clicked.connect(self.rbt_stop)  # fix name lol

#        self.actionImport_New_Object.triggered.connect(self.new_object_reset)

        self.lbl_charge_warn.setVisible(False)
        self.btn_charge_done.setVisible(False)

        # Set update rate to given value.
        self.lbl_updaterate.setText(str(round(1 / self.controller.update_rate)) + " /s")

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
        self.lbl_probe_potential.setText(str(round(self.controller.probe_voltage, 5)))

        # If there are normals, allow for probe flow generation
        self.btn_probe_setup.setEnabled(not self.controller.simulation_instance.normal_point_cloud is None)

        # if there is a probe plan, allow to execute
        self.btn_start_probing.setEnabled(not self.controller.simulation_instance.cur_probe_flow is None)

        if self.controller.simulation_instance.cur_probe_flow is not None:
            self.probe_progress_bar.setValue(self.controller.simulation_instance.cur_probe_flow.probe_percentage)

    def embed_pysim(self):
        hwnd = win32gui.FindWindowEx(0, 0, None, "Bullet Physics ExampleBrowser using OpenGL3+ [btgl] Release build")
        self.window = QtGui.QWindow.fromWinId(hwnd)
        self.windowcontainer = self.createWindowContainer(self.window, self.widget_pybullet)
        self.windowcontainer.setMinimumSize(1220, 900)

    def plot_slice(self, slice, path):
        self.widget_slice_disp.clear()
        time.sleep(0.1)

        x_values = [point.pos[0] for point in path]
        y_values = [point.pos[1] for point in path]

        self.widget_slice_disp.plot(x=x_values, y=y_values, pen='r', symbol='o', symbolPen='r', symbolBrush=(0, 0, 255),
                                    symbolSize=10)

        for i, point in enumerate(path):
            x, y, z = point.pos
            angle = np.arctan2(point.direction[1], point.direction[0])
            x_end = x + 0.025 * np.cos(angle)
            y_end = y + 0.025 * np.sin(angle)
            self.widget_slice_disp.plot([x, x_end], [y, y_end], pen='b')

            text = pg.TextItem(f'{i}', anchor=(0, 1), color=(255, 255, 255), fill=(0, 0, 0, 0))
            text.setPos(x, y)
            self.widget_slice_disp.addItem(text)

    def advance_flow(self):
        self.controller.simulation_instance.cur_probe_flow.charge_done_flag = True
        self.lbl_charge_warn.setVisible(False)
        self.btn_charge_done.setVisible(False)

    def sim_stop(self):
        # this function could do many things, but I've decided to make it stop probe flow and return the robot home.
        self.controller.simulation_instance.cur_probe_flow = None
        for i in range(100):
            self.controller.simulation_instance.drive_motors_to_home()
            p.stepSimulation()
            time.sleep(1 / 120)

        self.controller.simulation_instance.robot_handler.set_goal_conf(
            pp.get_joint_positions(self.controller.simulation_instance.sim_robot, [1, 2, 3, 4, 5]))

    def rbt_stop(self):
        self.controller.simulation_instance.robot_handler.terminate_robot()

    def edit_constants(self):
        pass

    def stop_program(self):
        print("[MAIN] Stopping Program!")
        self.sim_stop()
        time.sleep(1.5)
        self.update_thread.stop()
        self.controller.shutdown()
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
        msg.exec_()

    def closeEvent(self, a0: QtGui.QCloseEvent):
        self.update_thread.quit()
        self.stop_program()
        a0.accept()


class ObjectWizard(QWizard):

    def __init__(self, show_com=False, parent=None):
        super().__init__(parent)
        self.ui = Ui_ObjectWizard()
        self.ui.setupUi(self)

        self.sim_robot = None
        self.sim_platform = None
        self.obj = None

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.update)
        self.gui_timer.start(25)  # start GUI update thread ~ 40 FPS

        self.rbt = None
        self.has_init = False

        self.o3d_viz = ObjectVisualizer()

        hwnd = win32gui.FindWindowEx(0, 0, None, "Open3D")
        self.window = QtGui.QWindow.fromWinId(hwnd)
        self.window.resize(551, 291)
        self.windowcontainer = self.createWindowContainer(self.window, self.ui.widget_visualize)
        self.windowcontainer.setMinimumSize(551, 291)

        self.SIM_ROBOT_OFFSET = np.dot(2, [0, 0, 0])
        self.SIM_PLATFORM_OFFSET = np.dot(2, [0, 0, 0])
        self.obj_joint_offset = None
        self.obj_const = None
        self.obj_rot = [0, 0, 0]

        self.ui.sbox_rbt_offset_x.valueChanged.connect(self.update_rbt_offset)
        self.ui.sbox_rbt_offset_y.valueChanged.connect(self.update_rbt_offset)
        self.ui.sbox_rbt_offset_z.valueChanged.connect(self.update_rbt_offset)

        self.ui.sbox_offset_x.valueChanged.connect(self.update_obj_offset)
        self.ui.sbox_offset_y.valueChanged.connect(self.update_obj_offset)
        self.ui.sbox_offset_z.valueChanged.connect(self.update_obj_offset)

        self.ui.btn_prompt_path.clicked.connect(self.prompt_path)
        self.ui.rbtn_import_units.clicked.connect(self.update_import)
        self.ui.rbtn_import_scale.clicked.connect(self.update_import)
        self.ui.wiz_page_import_obj.nextId = lambda: 5
        self.update_import()

        self.ui.rbtn_prim_rect.clicked.connect(self.prim_rect_selected)
        self.ui.rbtn_prim_sphere.clicked.connect(self.prim_sphere_selected)
        self.ui.rbtn_prim_cylinder.clicked.connect(self.prim_cylinder_selected)

        self.ui.btn_start_scan.clicked.connect(self.start_scan)
        self.ui.btn_scan_halt.clicked.connect(self.halt_scan)
        self.ui.wiz_page_scan_obj.nextId = self.check_scan
        self.scan_thread = None
        self.scanner = None

        self.ui.wiz_page_create_primitive.nextId = self.prim_creation
        self.prim = 2

        self.ui.wiz_page_visualize_obj.nextId = self.pack_object

        self.ui.wiz_page_method.nextId = self.select_method

        self.button(QWizard.FinishButton).clicked.connect(self.finish_button)

        self.rbt = RobotHandler(port_config, dummy=True)
        self.rbt.stepper_board.home_all()

        if show_com:
            _dlg = DialogComPorts(self)
            _dlg.exec()


    def update(self):
        if self.scan_thread and self.scanner:
            self.ui.lbl_current_points.setText(f"{self.scanner.point_index}/{self.scanner.total_points}")

        if self.currentId() == 5 and not self.has_init:
            pp.connect(True)
            p.setGravity(0, 0, 0)
            # Add robot into PyBullet environment
            self.sim_robot = pp.load_pybullet(URDF_RBT, fixed_base=True, scale=2)
            p.resetBasePositionAndOrientation(self.sim_robot, self.SIM_ROBOT_OFFSET,
                                              p.getQuaternionFromEuler([0, 0, 0]))
            print(f"[SIM] Initialized robot with ID: {self.sim_robot}")

            # Add center platform AND OBJECT into PyBullet environment
            self.sim_platform = pp.load_pybullet(URDF_PLAT_NO_OBJ, fixed_base=True, scale=2)
            p.resetBasePositionAndOrientation(self.sim_platform, self.SIM_PLATFORM_OFFSET,
                                              p.getQuaternionFromEuler([0, 0, 0]))
            print(f"[SIM] Initialized platform with ID: {self.sim_platform}")
            self.obj = pp.load_pybullet(URDF_OBJ, scale=2)
            p.resetBasePositionAndOrientation(self.obj, np.dot(2, [0, 0, .15875]), [0, 0, 0, 1])

            self.obj_joint_offset = [0, 0, .16 * 2]
            self.obj_const = p.createConstraint(parentBodyUniqueId=self.sim_platform, parentLinkIndex=1,
                                                childBodyUniqueId=self.obj,
                                                childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                                                parentFramePosition=self.obj_joint_offset, childFramePosition=[0, 0, 0])

            pp.set_camera_pose(tuple(np.array((0, 0, 0.25)) + np.array([0.25, -0.25, 0.25])), (0, 0, 0.25))
            self.update_rbt_offset()
            # self.rbt.stepper_board.write_b(165)
            self.has_init = True

        if self.currentId() == 4:
            self.o3d_viz.update_visualizer()

        if pp.is_connected():
            pp.step_simulation()
            conf = self.rbt.read_cur_conf()[1]
            if conf is not None: pp.set_joint_positions(self.sim_robot, [1, 2, 3, 4, 5], conf)
            time.sleep(0.01)

    def update_rbt_offset(self):
        self.SIM_ROBOT_OFFSET = np.dot(2, [self.ui.sbox_rbt_offset_x.value() / 1000,
                                           self.ui.sbox_rbt_offset_y.value() / 1000,
                                           self.ui.sbox_rbt_offset_z.value() / 1000])
        p.resetBasePositionAndOrientation(self.sim_robot, self.SIM_ROBOT_OFFSET, p.getQuaternionFromEuler([0, 0, 0]))

    def update_obj_offset(self):
        self.obj_joint_offset = np.dot(2, [self.ui.sbox_offset_x.value() / 1000, self.ui.sbox_offset_y.value() / 1000,
                                           .16 + self.ui.sbox_offset_z.value() / 1000])
        self.obj_rot = [np.deg2rad(self.ui.sbox_offset_rx.value()), np.deg2rad(self.ui.sbox_offset_ry.value()),
                        np.deg2rad(self.ui.sbox_offset_rz.value())]

        p.removeConstraint(self.obj_const)
        if self.obj_rot != [0, 0, 0]: p.resetBasePositionAndOrientation(self.obj, self.obj_joint_offset, p.getQuaternionFromEuler(self.obj_rot))
        self.obj_const = p.createConstraint(parentBodyUniqueId=self.sim_platform, parentLinkIndex=1,
                                            childBodyUniqueId=self.obj,
                                            childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                                            parentFramePosition=self.obj_joint_offset, childFramePosition=[0, 0, 0])

    def update_import(self):
        self.ui.cbox_import_units.setVisible(self.ui.rbtn_import_units.isChecked())
        self.ui.sbox_import_X.setVisible(self.ui.rbtn_import_scale.isChecked())
        self.ui.sbox_import_Y.setVisible(self.ui.rbtn_import_scale.isChecked())
        self.ui.sbox_import_Z.setVisible(self.ui.rbtn_import_scale.isChecked())
        self.ui.lbl_import_X.setVisible(self.ui.rbtn_import_scale.isChecked())
        self.ui.lbl_import_Y.setVisible(self.ui.rbtn_import_scale.isChecked())
        self.ui.lbl_import_Z.setVisible(self.ui.rbtn_import_scale.isChecked())

    def prompt_path(self):
        file_path, _ = QFileDialog.getOpenFileName(None, "Select Object Mesh", "")
        self.ui.txt_path.setText(file_path)
        if not self.o3d_viz.load_mesh_from_path(file_path):
            QMessageBox.critical(None,
                                 "Invalid Filetype Selected!",
                                 "Invalid object filetype was selected, please select an object in .stl or .obj format!",
                                 QMessageBox.Ok)

    def start_scan(self):
        if self.scan_thread:
            self.scan_thread.join()
            del self.scan_thread

        self.scanner = Scanner(self.rbt.stepper_board, port_config, float(self.ui.sbox_scan_x.value()), float(self.ui.sbox_scan_y.value()), float(self.ui.sbox_scan_z.value()),
                    self.ui.prg_scan)

        self.scan_thread = threading.Thread(target=self.scanner.begin_scan)
        self.ui.prg_scan.setValue(0)
        self.scan_thread.start()

    def halt_scan(self):
        self.scanner.stop = True
        self.rbt.stepper_board.home_scan()

    def check_scan(self):
        if self.ui.prg_scan.value() == 100:
            return 4
        else:
            return 3

    def prim_rect_selected(self):
        self.show_all()
        self.ui.lbl_prim_field_A.setText("X")
        self.ui.lbl_prim_field_B.setText("Y")
        self.ui.lbl_prim_field_C.setText("Z")
        self.prim = 2

    def prim_sphere_selected(self):
        self.show_all()
        self.ui.lbl_prim_field_B.setVisible(False)
        self.ui.lbl_prim_field_C.setVisible(False)
        self.ui.lbl_prim_field_units_B.setVisible(False)
        self.ui.lbl_prim_field_units_C.setVisible(False)
        self.ui.sbox_prim_field_B.setVisible(False)
        self.ui.sbox_prim_field_C.setVisible(False)
        self.ui.lbl_prim_field_A.setText("Radius")
        self.prim = 1

    def prim_cylinder_selected(self):
        self.show_all()
        self.ui.lbl_prim_field_C.setVisible(False)
        self.ui.lbl_prim_field_units_C.setVisible(False)
        self.ui.sbox_prim_field_C.setVisible(False)
        self.ui.lbl_prim_field_A.setText("Radius")
        self.ui.lbl_prim_field_B.setText("Height")
        self.prim = 0

    def prim_creation(self):
        if float(self.ui.sbox_prim_field_A.value()) == 1:
            return 2

        self.o3d_viz.display_primitive(self.prim, 8192, float(self.ui.sbox_prim_field_A.text()),
                                       float(self.ui.sbox_prim_field_B.text()), float(self.ui.sbox_prim_field_C.text()))
        self.o3d_viz.disp_cur_mesh()

        # self.prim_scanner = PrimitiveScan(self.rbt.stepper_board, port_config)
        # self.prim_scanner.run_prim_scan(2, float(self.ui.sbox_prim_field_A.text()),
        #                                float(self.ui.sbox_prim_field_B.text()), float(self.ui.sbox_prim_field_C.text()))

        return 4

    def pack_object(self):
        self.o3d_viz.pack_object()
        time.sleep(1)
        return 5

    def select_method(self):
        if self.ui.rbt_primitive.isChecked():
            return 1
        elif self.ui.rbtn_import_obj.isChecked():
            return 2
        elif self.ui.rbt_scan_obj.isChecked():
            return 3
        else:
            return 0

    def show_all(self):
        self.ui.lbl_prim_field_A.setVisible(True)
        self.ui.lbl_prim_field_B.setVisible(True)
        self.ui.lbl_prim_field_C.setVisible(True)
        self.ui.lbl_prim_field_units_A.setVisible(True)
        self.ui.lbl_prim_field_units_B.setVisible(True)
        self.ui.lbl_prim_field_units_C.setVisible(True)
        self.ui.sbox_prim_field_A.setVisible(True)
        self.ui.sbox_prim_field_B.setVisible(True)
        self.ui.sbox_prim_field_C.setVisible(True)

    def finish_button(self):
        pp.disconnect()
        self.rbt.terminate_robot()
        self.o3d_viz.visualizer.destroy_window()
        self.gui_timer.stop()

        # overwrite offsets
        (x, y, z) = self.obj_joint_offset
        URDF_OFFSET_LINE_NUM = 77
        URDF_ROT_LINE_NUM = 82

        (rx, ry, rz) = self.obj_rot

        with open(URDF_PLAT, 'r') as file:
            content = file.readlines()
            str_write = f'    <origin xyz="{x / 2} {y / 2} {z / 2}"/>\n'
            content[URDF_OFFSET_LINE_NUM - 1] = str_write  # replace contents with modified offset.

            # str_write = f'      <origin rpy="{rx} {ry} {rz + np.pi/2}" xyz="0 0 0"/>\n'
            # content[URDF_ROT_LINE_NUM - 1] = str _write

        with open(URDF_PLAT, 'w') as file:
            file.writelines(content)

        time.sleep(1)
        self.destroy()


skip_wiz = False
param = []

if __name__ == '__main__':
    # current_dir = os.getcwd()
    # print(current_dir)
    #
    # log_file_name = "console_log.txt"
    # log_file_path = os.path.join(current_dir, log_file_name)
    # sys.stdout = open(log_file_path, "w")

    if not skip_wiz:
        app1 = QApplication(sys.argv)
        first_window = ObjectWizard()
        first_window.show()
        app1.exec_()
        app1.exit()
        param = [first_window.SIM_ROBOT_OFFSET, first_window.obj_joint_offset]
        del app1
        del first_window

        print(param)

    # After the first window is closed, this part will run

    app1 = QApplication(sys.argv)
    second_window = MainWindow(param)
    second_window.show()
    sys.exit(app1.exec_())