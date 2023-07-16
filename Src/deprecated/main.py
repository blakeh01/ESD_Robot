import sys

import pyqtgraph as pg
import src.sim.Command as cmd

from PyQt5.QtWidgets import (
    QApplication, QMainWindow
)
from PyQt5.QtCore import QTimer

from src.gui.main_window import Ui_MainWindow
from src.gui.dialogs.dialog_cmd_set_pose import DialogSetProbePosition
from src.gui.dialogs.dialog_choose_object import DialogChooseObject  # todo maybe? put into a single dialogs class
from src.gui.dialogs.dialog_robot_info import DialogRobotInfo
from src.Controller import Controller
from src.robot.RobotHandler import RobotHandler

class MainWindow(QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setupUi(self)
        self.setWindowTitle("Controller")

        # Update our GUI to keep up with the controller flags
        self._gui_timer = QTimer()
        self._gui_timer.timeout.connect(self.update_gui)

        # New controller that is stored within our main window.
        print("[MAIN] Initializing Controller...")
        self.controller = Controller(self)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.controller.send_update)
        self._gui_timer.start(25)
        self._gui_frame = [0]

        # Program Button Events:
        self.btn_start_program.clicked.connect(self.start_program)
        self.btn_edit_consts.clicked.connect(lambda: self.send_command("cmd_test"))
        #self.btn_edit_consts.clicked.connect(self.edit_constants)
        self.btn_shutdown_program.clicked.connect(self.stop_program)
        self.btn_rbt_info.clicked.connect(self.dialog_rbt_info)

        # Dialogs:
        self.btn_choose_object.clicked.connect(self.dialog_choose_obj)
        self.btn_cmd_setprobepos.clicked.connect(self.dialog_set_probe_pos)

        # Command Handling:
        self.cmd_timeout_timer = QTimer()
        def chk_timeout():
            if self.controller.current_command is not None:
                print("[CMD] Command Timed Out! This could be dangerous!")
                self.controller.current_command = None
                self.cmd_timeout_timer.stop()
        self.cmd_timeout_timer.timeout.connect(chk_timeout)

        self.command_list = [
            ("cmd_go_to_position", cmd.CmdGoToPosiiton(self.controller, timeout=2000)),
            ("cmd_restart_sim", cmd.CmdRestartSim(self.controller)),
            ("cmd_generate_normals", cmd.CmdGenerateNormals(self.controller)),
            ("cmd_test", cmd.CmdTest(self.controller))
        ]

        self.btn_cmd_home.clicked.connect(lambda: self.send_command("cmd_go_to_position", [0, 0, 1]))
        self.btn_quit_sim.clicked.connect(lambda: self.send_command("cmd_restart_sim"))
        self.btn_cmd_generate_normals.clicked.connect(lambda: self.send_command("cmd_generate_normals"))

        # Graph setup
        pen = pg.mkPen(color=(255, 0, 0))
        #self.widget_graph_dt.setLabel("left", "Process Time (s)", **{"color": "#f00", "font-size": "10px"})
        self.data_line = self.widget_graph_dt.plot([0], [0], pen=pen)
       # self.widget_graph_dt.addLegend()
        self.widget_graph_dt.showGrid(x=False, y=True)

        # Set update rate to given value.
        self.lbl_updaterate.setText(str(round(1/self.controller.update_rate)) + " /s")

        print("[MAIN] Initialized Program! Ready for action...")

    def update_gui(self):
        self._gui_frame.append(self._gui_frame[-1] + 1)

        # Update simulation isRunning label
        if self.controller.simulation_instance.can_run:
            self.lbl_sim_status.setStyleSheet("background-color: lightgreen")
            self.lbl_sim_status.setText("RUNNING")
        else:
            self.lbl_sim_status.setStyleSheet("background-color: red")
            self.lbl_sim_status.setText("OFF")

        # Update collision label
        if self.controller.obj_distance <= 0:
            self.lbl_distance_rbt_obj.setStyleSheet("background-color: red")
        elif 0 < self.controller.obj_distance < 0.01:
            self.lbl_distance_rbt_obj.setStyleSheet("background-color: orange")
        else:
            self.lbl_distance_rbt_obj.setStyleSheet("background-color: lightgreen")
        self.lbl_distance_rbt_obj.setText(str(round(self.controller.obj_distance, 5)))

        # Update time management labels
        self.lbl_dt.setText(str(round(self.controller.dt, 3)))
        self.lbl_time.setText(str(round(self.controller.time_elapsed, 5)))

        y_data = self.controller.cached_dt[-min(len(self.controller.cached_dt), 100):]
        x_data = self._gui_frame[-min(len(self.controller.cached_dt), 100):]

        if len(self._gui_frame) % 2 == 0:
            self.data_line.setData(x_data, y_data)

    def send_command(self, command, *args):
        print(f"[MAIN] Received command: {command}")
        for cmd in self.command_list:
            if cmd[0] == command:
                cmd[1].executeCommand(args)
                if cmd[1].timeout > 0:
                    self.cmd_timeout_timer.start(cmd[1].timeout)

    def start_program(self):
        self.update_timer.start(1)
        self.btn_start_program.setEnabled(False)
        print("[MAIN] Begun Update Timers!")

    def editMode(self):
        self.ctrl.switch_mode()

    def edit_constants(self):
        pass

    def stop_program(self):
        print("[MAIN] Stopping Program!")
        self.send_command("cmd_shutdown")
        self.controller.shutdown()
        exit()

    def dialog_choose_obj(self):
        _dlg = DialogChooseObject(self)
        _dlg.exec()

    def dialog_set_probe_pos(self):
        _dlg = DialogSetProbePosition(self)
        _dlg.exec()

    def dialog_rbt_info(self):
        _dlg = DialogRobotInfo(self)
        _dlg.exec()




# class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
#
#     def __init__(self, *args, **kwargs):
#         super(MainWindow, self).__init__(*args, **kwargs)
#
#         self.initialized = False
#         self.setWindowTitle("Controller")
#         self.setupUi(self)
#
#         # Initialize controller
#         self.ctrl = Controller(self)
#
#         self.gui_timer = QTimer()
#         self.gui_timer.timeout.connect(self.update_gui)
#         self.gui_timer.start(50)
#
#         # Create a QTimer that will run in the background of the GUI
#         # This will be responsible for triggering updates on robot/simulation.
#         self.update_timer = QTimer()
#         self.update_timer.timeout.connect(self.ctrl.send_update)
#
#         # Set button events
#         self.btn_start.clicked.connect(self.start_update_timer)
#         self.btn_shutdown.clicked.connect(self.shutdown)
#         self.btn_probepos.clicked.connect(self.onProbePosClick)
#         self.btn_edit_consts.clicked.connect(self.editMode)
#         #self.btn_rbt_terminate.clicked.connect(self.ctrl.robot_instance.terminateRobot)
#         self.btn_stop_sim.clicked.connect(self.sim_stopstart)
#
#         # Set up graph params. todo move to gui class once complete
#         self.widget_graph_dt.setBackground('w')
#         pen = pg.mkPen(color=(255, 0, 0))
#         self.widget_graph_dt.setLabel("left", "Process Time (s)", **{"color": "#f00", "font-size": "10px"})
#         self.data_line = self.widget_graph_dt.plot([0], [0], pen=pen)
#         self.widget_graph_dt.addLegend()
#         self.widget_graph_dt.showGrid(x=False, y=True)
#
#         # Set update rate to given value.
#         self.lbl_updaterate.setText(str(round(1/self.ctrl.update_rate)) + " /s")
#
#         self.initialized = True
#
#     def update_gui(self):
#         # Update robot status label
#         # if self.ctrl.robot_instance.status == RBT_STATUS_OKAY:
#         #     self.lbl_rbt_status.setStyleSheet("background-color: lightgreen")
#         # elif self.ctrl.robot_instance.status == RBT_STATUS_WARN:
#         #     self.lbl_rbt_status.setStyleSheet("background-color: orange")
#         # else:
#         #     self.lbl_rbt_status.setStyleSheet("background-color: red")
#         # self.lbl_rbt_status.setText(self.ctrl.robot_instance.status)
#
#         # Update simulation isRunning label
#         if self.ctrl.simulation_instance.can_run:
#             self.lbl_sim_status.setStyleSheet("background-color: lightgreen")
#             self.lbl_sim_status.setText("RUNNING")
#         else:
#             self.lbl_sim_status.setStyleSheet("background-color: red")
#             self.lbl_sim_status.setText("OFF")
#
#         # Update simulation isMoving label
#         if self.ctrl.move_flag:
#             self.lbl_ismoving.setStyleSheet("background-color: lightgreen")
#             self.lbl_ismoving.setText("TRUE")
#         else:
#             self.lbl_ismoving.setStyleSheet("background-color: red")
#             self.lbl_ismoving.setText("FALSE")
#
#         # Update program mode
#         if self.ctrl.shadow == 0:
#             self.lbl_mode.setText("MIMIC SIMULATION")
#         elif self.ctrl.shadow == 1:
#             self.lbl_mode.setText("MIMIC ROBOT")
#         else:
#             self.lbl_mode.setText("COMMAND MODE")
#
#         # Update collision label
#         if self.ctrl.obj_distance <= 0:
#             self.lbl_distance_rbt_obj.setStyleSheet("background-color: red")
#         elif 0 < self.ctrl.obj_distance < 0.01:
#             self.lbl_distance_rbt_obj.setStyleSheet("background-color: orange")
#         else:
#             self.lbl_distance_rbt_obj.setStyleSheet("background-color: lightgreen")
#         self.lbl_distance_rbt_obj.setText(str(round(self.ctrl.obj_distance, 5)))
#
#         # Update time management labels
#         self.lbl_dt.setText(str(round(self.ctrl.dt, 3)))
#         self.lbl_time.setText(str(round(self.ctrl.time_elapsed, 5)))
#
#         # Graph process time.
#         x_size = 100
#         x = np.arange(1, min(len(self.ctrl.cached_dt), x_size+1))
#         y = self.ctrl.cached_dt[-x_size:]
#         if 0 < len(y) < x_size:
#             y.pop()  # hacky asf, fix
#
#         self.data_line.setData(x, y)
#
#     def start_update_timer(self):
#         self.ctrl.simulation_instance.can_run = True
#         self.update_timer.start(1)
#         self.btn_start.setEnabled(False)
#
#     def onProbePosClick(self):
#         dlg = ProbeDlg(self)
#         dlg.exec()
#
#     # TODO TEMP FOR DEMO
#     def editMode(self):
#         self.ctrl.switch_mode()
#
#     def shutdown(self):
#         self.ctrl.shutdown()
#         time.sleep(0.5)
#         exit()
#
#     def sim_stopstart(self):
#         pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    form = MainWindow()
    form.show()
    sys.exit(app.exec_())
