'''

    maingui.py creates the update thread and GUI for the program.
    Run this file to start the program...

'''

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
    QApplication, QMainWindow, QMessageBox
)
from Src.Controller import Controller
from Src.gui.dialogs.dialog_cmd_set_pose import DialogSetProbePosition
from Src.gui.dialogs.dialog_obj_offset import DialogOffsetObject
from Src.gui.dialogs.dialog_robot_info import DialogRobotInfo
from Src.gui.dialogs.dialog_normal_generator import DialogNormalGenerator
from Src.gui.dialogs.dialog_probe_profile import DialogProbeProfile
from Src.gui.dialogs.dialog_charge_object import DialogChargeObject
from Src.gui.main_window import Ui_MainWindow

import pyqtgraph as pg
import numpy as np

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

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setupUi(self)
        self.setWindowTitle("Controller")

        # Update our GUI to keep up with the controller flags
        self._gui_timer = QTimer(self)
        self._gui_timer.timeout.connect(self.update_gui)

        # New controller that is stored within our main window.
        print("[MAIN] Initializing Controller...")
        self.controller = Controller(self)

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
        # self.cb_primitive_select.currentIndexChanged.connect(self.on_primitive_select)  # primitive select combobox
        # self.btn_obj_create.clicked.connect(self.on_primitive_create)  # primitive create button
        # self.btn_obj_send_to_sim.clicked.connect(self.on_send_obj_to_sim)
        # self.btn_scan_start.clicked.connect(self.start_obj_scan)
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

        # If the user is currently on the visualizer, update it.
        if self.widget_window_tabs.currentIndex() == 1:
            self.o3d_visualizer.update_visualizer()

        # If there is no current mesh, disable the 'send to sim' button
        # self.btn_obj_send_to_sim.setEnabled(not self.o3d_visualizer.cur_mesh is None)

        # If there are normals, allow for probe flow generation
        self.btn_probe_setup.setEnabled(not self.controller.simulation_instance.normal_point_cloud is None)

        # if theere is a probe plan, allow to execute
        self.btn_start_probing.setEnabled(not self.controller.simulation_instance.cur_probe_flow is None)

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
        pp.control_joints(0, [1, 2, 3, 4, 5],
                          pp.inverse_kinematics_helper(0, 6, ([0, 0.15, 0.5], p.getQuaternionFromEuler([0, 0, 0]))))
        pass

    def stop_program(self):
        print("[MAIN] Stopping Program!")
        self.window.setParent(None)
        self.update_thread.stop()
        self.controller.shutdown()
        # self.o3d_visualizer.visualizer.destroy_window()
        exit()

    def start_obj_scan(self):
        pass

    def begin_probe_flow(self):
        self.controller.simulation_instance.can_execute_flow = True

    def dialog_probe_setup(self):
        _dlg = DialogProbeProfile(self)
        _dlg.exec()

    def dialog_rbt_info(self):
        _dlg = DialogRobotInfo(self)
        _dlg.exec()

    def dialog_set_probe_pos(self):
        _dlg = DialogSetProbePosition(self)
        _dlg.exec()

    def dialog_normal_generator(self):
        _dlg = DialogNormalGenerator(self)
        _dlg.exec()

    def on_primitive_select(self, index):
        selected_prim = self.cb_primitive_select.itemText(index)

        # Hide the UI and set them visible depending on what primitive is selected.
        self.lbl_info_a.setVisible(False)
        self.lbl_info_b.setVisible(False)
        self.lbl_info_c.setVisible(False)
        self.lbl_info_m1.setVisible(False)
        self.lbl_info_m2.setVisible(False)
        self.lbl_info_m3.setVisible(False)
        self.txt_obj_a.setVisible(False)
        self.txt_obj_b.setVisible(False)
        self.txt_obj_c.setVisible(False)

        # Check what the primitive is and display the proper text boxes and labels.
        if selected_prim.lower() == "sphere":
            self.lbl_info_a.setText("Radius:")
            self.lbl_info_a.setVisible(True)
            self.lbl_info_m1.setVisible(True)
            self.txt_obj_a.setVisible(True)
        elif selected_prim.lower() == "cylinder":
            self.lbl_info_a.setText("Radius:")
            self.lbl_info_b.setText("Height:")
            self.lbl_info_a.setVisible(True)
            self.lbl_info_b.setVisible(True)
            self.lbl_info_m1.setVisible(True)
            self.lbl_info_m2.setVisible(True)
            self.txt_obj_a.setVisible(True)
            self.txt_obj_b.setVisible(True)
        elif selected_prim.lower() == "rectangular prism":
            self.lbl_info_a.setText("Width:")
            self.lbl_info_b.setText("Height:")
            self.lbl_info_c.setText("Depth:")
            self.lbl_info_a.setVisible(True)
            self.lbl_info_b.setVisible(True)
            self.lbl_info_c.setVisible(True)
            self.lbl_info_m1.setVisible(True)
            self.lbl_info_m2.setVisible(True)
            self.lbl_info_m3.setVisible(True)
            self.txt_obj_a.setVisible(True)
            self.txt_obj_b.setVisible(True)
            self.txt_obj_c.setVisible(True)

    def on_primitive_create(self):
        match self.cb_primitive_select.currentIndex():
            case 0:
                self.show_information_box("Select a Primitive!", "Please select a primitive object before creation!")

            case 1:  # sphere was selected.
                if float(self.txt_obj_a.text()) == 0:
                    self.show_information_box("Enter Required Fields!",
                                              "Please enter all required fields before creation!")
                    return
                self.o3d_visualizer.display_primitive(
                    "sphere",
                    200,
                    float(self.txt_obj_a.text())
                )

            case 2:  # prism was selected.
                if float(self.txt_obj_a.text()) == 0 or float(self.txt_obj_b.text()) == 0 or float(
                        self.txt_obj_c.text()) == 0:
                    self.show_information_box("Enter Required Fields!",
                                              "Please enter all required fields before creation!")
                    return
                self.o3d_visualizer.display_primitive(
                    "rectangular prism",
                    200,
                    float(self.txt_obj_a.text()), float(self.txt_obj_b.text()), float(self.txt_obj_c.text())
                )

            case 3:  # cylinder was selected.
                if float(self.txt_obj_a.text()) == 0 or float(self.txt_obj_b.text()) == 0:
                    self.show_information_box("Enter Required Fields!",
                                              "Please enter all required fields before creation!")
                    return
                self.o3d_visualizer.display_primitive(
                    "cylinder",
                    200,
                    float(self.txt_obj_a.text()), float(self.txt_obj_b.text())
                )

    def on_send_obj_to_sim(self):
        _dlg = DialogOffsetObject(self)
        _dlg.exec()

    def send_object_to_sim(self):
        print("[MAIN] Sending object to simulation! Please hold...")
        self.o3d_visualizer.pack_object()
        time.sleep(0.5)
        self.controller.restart_sim()

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


if __name__ == '__main__':
    current_dir = os.getcwd()
    print(current_dir)

    log_file_name = "console_log.txt"
    log_file_path = os.path.join(current_dir, log_file_name)

    sys.stdout = open(log_file_path, "w")

    app = QApplication(sys.argv)
    form = MainWindow()
    form.show()

    sys.stdout.close()
    sys.exit(app.exec_())
