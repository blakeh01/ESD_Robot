# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1720, 967)
        MainWindow.setMinimumSize(QtCore.QSize(865, 600))
        MainWindow.setMaximumSize(QtCore.QSize(99999, 99999))
        MainWindow.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.grp_robot = QtWidgets.QGroupBox(self.centralwidget)
        self.grp_robot.setGeometry(QtCore.QRect(10, 100, 201, 191))
        self.grp_robot.setObjectName("grp_robot")
        self.lbl_info_rbtstatus = QtWidgets.QLabel(self.grp_robot)
        self.lbl_info_rbtstatus.setGeometry(QtCore.QRect(20, 20, 71, 16))
        self.lbl_info_rbtstatus.setObjectName("lbl_info_rbtstatus")
        self.lbl_rbt_status = QtWidgets.QLabel(self.grp_robot)
        self.lbl_rbt_status.setGeometry(QtCore.QRect(100, 22, 91, 16))
        self.lbl_rbt_status.setStyleSheet("background-color: red")
        self.lbl_rbt_status.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_rbt_status.setObjectName("lbl_rbt_status")
        self.btn_rbt_terminate = QtWidgets.QPushButton(self.grp_robot)
        self.btn_rbt_terminate.setGeometry(QtCore.QRect(10, 160, 71, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.btn_rbt_terminate.setFont(font)
        self.btn_rbt_terminate.setStyleSheet("background-color: red")
        self.btn_rbt_terminate.setObjectName("btn_rbt_terminate")
        self.btn_rbt_info = QtWidgets.QPushButton(self.grp_robot)
        self.btn_rbt_info.setGeometry(QtCore.QRect(160, 160, 31, 23))
        self.btn_rbt_info.setObjectName("btn_rbt_info")
        self.lbl_ismoving_rail = QtWidgets.QLabel(self.grp_robot)
        self.lbl_ismoving_rail.setGeometry(QtCore.QRect(112, 50, 16, 16))
        self.lbl_ismoving_rail.setStyleSheet("background-color: red")
        self.lbl_ismoving_rail.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_ismoving_rail.setObjectName("lbl_ismoving_rail")
        self.lbl_ismoving_wrist = QtWidgets.QLabel(self.grp_robot)
        self.lbl_ismoving_wrist.setGeometry(QtCore.QRect(160, 50, 16, 16))
        self.lbl_ismoving_wrist.setStyleSheet("background-color: red")
        self.lbl_ismoving_wrist.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_ismoving_wrist.setObjectName("lbl_ismoving_wrist")
        self.lbl_ismoving_shoulder = QtWidgets.QLabel(self.grp_robot)
        self.lbl_ismoving_shoulder.setGeometry(QtCore.QRect(128, 50, 16, 16))
        self.lbl_ismoving_shoulder.setStyleSheet("background-color: red")
        self.lbl_ismoving_shoulder.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_ismoving_shoulder.setObjectName("lbl_ismoving_shoulder")
        self.lbl_ismoving_elbow = QtWidgets.QLabel(self.grp_robot)
        self.lbl_ismoving_elbow.setGeometry(QtCore.QRect(144, 50, 16, 16))
        self.lbl_ismoving_elbow.setStyleSheet("background-color: red")
        self.lbl_ismoving_elbow.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_ismoving_elbow.setObjectName("lbl_ismoving_elbow")
        self.lbl_info_activejoints = QtWidgets.QLabel(self.grp_robot)
        self.lbl_info_activejoints.setGeometry(QtCore.QRect(20, 50, 71, 16))
        self.lbl_info_activejoints.setObjectName("lbl_info_activejoints")
        self.lbl_info_currentgoal = QtWidgets.QLabel(self.grp_robot)
        self.lbl_info_currentgoal.setGeometry(QtCore.QRect(20, 65, 71, 16))
        self.lbl_info_currentgoal.setObjectName("lbl_info_currentgoal")
        self.lbl_info_currentgoal_2 = QtWidgets.QLabel(self.grp_robot)
        self.lbl_info_currentgoal_2.setGeometry(QtCore.QRect(20, 110, 71, 16))
        self.lbl_info_currentgoal_2.setObjectName("lbl_info_currentgoal_2")
        self.lbl_rbt_status_2 = QtWidgets.QLabel(self.grp_robot)
        self.lbl_rbt_status_2.setGeometry(QtCore.QRect(90, 65, 111, 16))
        self.lbl_rbt_status_2.setStyleSheet("")
        self.lbl_rbt_status_2.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_rbt_status_2.setObjectName("lbl_rbt_status_2")
        self.lbl_rbt_status_3 = QtWidgets.QLabel(self.grp_robot)
        self.lbl_rbt_status_3.setGeometry(QtCore.QRect(90, 110, 111, 16))
        self.lbl_rbt_status_3.setStyleSheet("")
        self.lbl_rbt_status_3.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_rbt_status_3.setObjectName("lbl_rbt_status_3")
        self.lbl_info_distance = QtWidgets.QLabel(self.grp_robot)
        self.lbl_info_distance.setGeometry(QtCore.QRect(20, 94, 91, 16))
        self.lbl_info_distance.setObjectName("lbl_info_distance")
        self.lbl_distance_rbt_obj = QtWidgets.QLabel(self.grp_robot)
        self.lbl_distance_rbt_obj.setGeometry(QtCore.QRect(120, 97, 46, 13))
        self.lbl_distance_rbt_obj.setStyleSheet("background-color: red")
        self.lbl_distance_rbt_obj.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_distance_rbt_obj.setObjectName("lbl_distance_rbt_obj")
        self.grp_simulation = QtWidgets.QGroupBox(self.centralwidget)
        self.grp_simulation.setGeometry(QtCore.QRect(230, 100, 211, 191))
        self.grp_simulation.setObjectName("grp_simulation")
        self.lbl_info_simstatus = QtWidgets.QLabel(self.grp_simulation)
        self.lbl_info_simstatus.setGeometry(QtCore.QRect(20, 20, 91, 16))
        self.lbl_info_simstatus.setObjectName("lbl_info_simstatus")
        self.lbl_sim_status = QtWidgets.QLabel(self.grp_simulation)
        self.lbl_sim_status.setGeometry(QtCore.QRect(120, 20, 51, 16))
        self.lbl_sim_status.setStyleSheet("background-color: red")
        self.lbl_sim_status.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_sim_status.setObjectName("lbl_sim_status")
        self.btn_rbt_terminate_2 = QtWidgets.QPushButton(self.grp_simulation)
        self.btn_rbt_terminate_2.setGeometry(QtCore.QRect(130, 160, 71, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.btn_rbt_terminate_2.setFont(font)
        self.btn_rbt_terminate_2.setStyleSheet("background-color: red")
        self.btn_rbt_terminate_2.setObjectName("btn_rbt_terminate_2")
        self.lbl_info_simstatus_2 = QtWidgets.QLabel(self.grp_simulation)
        self.lbl_info_simstatus_2.setGeometry(QtCore.QRect(20, 40, 91, 16))
        self.lbl_info_simstatus_2.setObjectName("lbl_info_simstatus_2")
        self.lbl_sim_status_2 = QtWidgets.QLabel(self.grp_simulation)
        self.lbl_sim_status_2.setGeometry(QtCore.QRect(120, 42, 51, 16))
        self.lbl_sim_status_2.setStyleSheet("background-color: red")
        self.lbl_sim_status_2.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_sim_status_2.setObjectName("lbl_sim_status_2")
        self.pushButton = QtWidgets.QPushButton(self.grp_simulation)
        self.pushButton.setEnabled(True)
        self.pushButton.setGeometry(QtCore.QRect(20, 70, 171, 23))
        self.pushButton.setObjectName("pushButton")
        self.grp_proginfo = QtWidgets.QGroupBox(self.centralwidget)
        self.grp_proginfo.setGeometry(QtCore.QRect(10, 10, 431, 80))
        self.grp_proginfo.setObjectName("grp_proginfo")
        self.lbl_info_time = QtWidgets.QLabel(self.grp_proginfo)
        self.lbl_info_time.setGeometry(QtCore.QRect(10, 20, 71, 16))
        self.lbl_info_time.setObjectName("lbl_info_time")
        self.lbl_info_processtime = QtWidgets.QLabel(self.grp_proginfo)
        self.lbl_info_processtime.setGeometry(QtCore.QRect(10, 40, 71, 16))
        self.lbl_info_processtime.setObjectName("lbl_info_processtime")
        self.lbl_time = QtWidgets.QLabel(self.grp_proginfo)
        self.lbl_time.setGeometry(QtCore.QRect(80, 20, 91, 16))
        self.lbl_time.setObjectName("lbl_time")
        self.lbl_dt = QtWidgets.QLabel(self.grp_proginfo)
        self.lbl_dt.setGeometry(QtCore.QRect(80, 40, 61, 16))
        self.lbl_dt.setObjectName("lbl_dt")
        self.btn_edit_consts = QtWidgets.QPushButton(self.grp_proginfo)
        self.btn_edit_consts.setGeometry(QtCore.QRect(336, 52, 91, 23))
        self.btn_edit_consts.setObjectName("btn_edit_consts")
        self.lbl_info_updaterate = QtWidgets.QLabel(self.grp_proginfo)
        self.lbl_info_updaterate.setGeometry(QtCore.QRect(10, 57, 71, 20))
        self.lbl_info_updaterate.setObjectName("lbl_info_updaterate")
        self.lbl_updaterate = QtWidgets.QLabel(self.grp_proginfo)
        self.lbl_updaterate.setGeometry(QtCore.QRect(80, 57, 41, 20))
        self.lbl_updaterate.setObjectName("lbl_updaterate")
        self.btn_shutdown_program = QtWidgets.QPushButton(self.centralwidget)
        self.btn_shutdown_program.setGeometry(QtCore.QRect(10, 300, 431, 30))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.btn_shutdown_program.setFont(font)
        self.btn_shutdown_program.setStyleSheet("background-color: red")
        self.btn_shutdown_program.setObjectName("btn_shutdown_program")
        self.widget_window_tabs = QtWidgets.QTabWidget(self.centralwidget)
        self.widget_window_tabs.setGeometry(QtCore.QRect(460, 10, 1251, 941))
        self.widget_window_tabs.setTabShape(QtWidgets.QTabWidget.Triangular)
        self.widget_window_tabs.setObjectName("widget_window_tabs")
        self.tab_pybullet = QtWidgets.QWidget()
        self.tab_pybullet.setObjectName("tab_pybullet")
        self.widget_open3d_2 = QtWidgets.QOpenGLWidget(self.tab_pybullet)
        self.widget_open3d_2.setGeometry(QtCore.QRect(10, 160, 1221, 761))
        self.widget_open3d_2.setObjectName("widget_open3d_2")
        self.widget_window_tabs.addTab(self.tab_pybullet, "")
        self.tab_open3d = QtWidgets.QWidget()
        self.tab_open3d.setObjectName("tab_open3d")
        self.widget_open3d = QtWidgets.QOpenGLWidget(self.tab_open3d)
        self.widget_open3d.setGeometry(QtCore.QRect(10, 160, 1221, 761))
        self.widget_open3d.setObjectName("widget_open3d")
        self.tabObjectMode = QtWidgets.QTabWidget(self.tab_open3d)
        self.tabObjectMode.setGeometry(QtCore.QRect(10, 10, 511, 141))
        self.tabObjectMode.setTabPosition(QtWidgets.QTabWidget.South)
        self.tabObjectMode.setDocumentMode(False)
        self.tabObjectMode.setObjectName("tabObjectMode")
        self.tab_obj_primitives = QtWidgets.QWidget()
        self.tab_obj_primitives.setObjectName("tab_obj_primitives")
        self.cb_primitive_select = QtWidgets.QComboBox(self.tab_obj_primitives)
        self.cb_primitive_select.setGeometry(QtCore.QRect(10, 10, 121, 22))
        self.cb_primitive_select.setObjectName("cb_primitive_select")
        self.cb_primitive_select.addItem("")
        self.cb_primitive_select.addItem("")
        self.cb_primitive_select.addItem("")
        self.cb_primitive_select.addItem("")
        self.btn_obj_create = QtWidgets.QPushButton(self.tab_obj_primitives)
        self.btn_obj_create.setGeometry(QtCore.QRect(420, 90, 75, 23))
        self.btn_obj_create.setObjectName("btn_obj_create")
        self.txt_obj_a = QtWidgets.QDoubleSpinBox(self.tab_obj_primitives)
        self.txt_obj_a.setEnabled(True)
        self.txt_obj_a.setGeometry(QtCore.QRect(58, 48, 62, 22))
        self.txt_obj_a.setObjectName("txt_obj_a")
        self.txt_obj_b = QtWidgets.QDoubleSpinBox(self.tab_obj_primitives)
        self.txt_obj_b.setEnabled(True)
        self.txt_obj_b.setGeometry(QtCore.QRect(238, 48, 62, 22))
        self.txt_obj_b.setObjectName("txt_obj_b")
        self.txt_obj_c = QtWidgets.QDoubleSpinBox(self.tab_obj_primitives)
        self.txt_obj_c.setEnabled(True)
        self.txt_obj_c.setGeometry(QtCore.QRect(415, 48, 62, 22))
        self.txt_obj_c.setObjectName("txt_obj_c")
        self.lbl_info_m1 = QtWidgets.QLabel(self.tab_obj_primitives)
        self.lbl_info_m1.setEnabled(True)
        self.lbl_info_m1.setGeometry(QtCore.QRect(124, 51, 31, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_m1.setFont(font)
        self.lbl_info_m1.setObjectName("lbl_info_m1")
        self.lbl_info_m2 = QtWidgets.QLabel(self.tab_obj_primitives)
        self.lbl_info_m2.setEnabled(True)
        self.lbl_info_m2.setGeometry(QtCore.QRect(304, 51, 31, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_m2.setFont(font)
        self.lbl_info_m2.setObjectName("lbl_info_m2")
        self.lbl_info_m3 = QtWidgets.QLabel(self.tab_obj_primitives)
        self.lbl_info_m3.setEnabled(True)
        self.lbl_info_m3.setGeometry(QtCore.QRect(481, 51, 31, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_m3.setFont(font)
        self.lbl_info_m3.setObjectName("lbl_info_m3")
        self.lbl_info_a = QtWidgets.QLabel(self.tab_obj_primitives)
        self.lbl_info_a.setEnabled(True)
        self.lbl_info_a.setGeometry(QtCore.QRect(10, 49, 47, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_a.setFont(font)
        self.lbl_info_a.setObjectName("lbl_info_a")
        self.lbl_info_b = QtWidgets.QLabel(self.tab_obj_primitives)
        self.lbl_info_b.setEnabled(True)
        self.lbl_info_b.setGeometry(QtCore.QRect(174, 49, 61, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_b.setFont(font)
        self.lbl_info_b.setObjectName("lbl_info_b")
        self.lbl_info_c = QtWidgets.QLabel(self.tab_obj_primitives)
        self.lbl_info_c.setEnabled(True)
        self.lbl_info_c.setGeometry(QtCore.QRect(352, 49, 61, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_c.setFont(font)
        self.lbl_info_c.setObjectName("lbl_info_c")
        self.tabObjectMode.addTab(self.tab_obj_primitives, "")
        self.tab_obj_import = QtWidgets.QWidget()
        self.tab_obj_import.setObjectName("tab_obj_import")
        self.txt_obj_import_path = QtWidgets.QLineEdit(self.tab_obj_import)
        self.txt_obj_import_path.setGeometry(QtCore.QRect(95, 10, 371, 20))
        self.txt_obj_import_path.setObjectName("txt_obj_import_path")
        self.btn_obj_import_lookup = QtWidgets.QPushButton(self.tab_obj_import)
        self.btn_obj_import_lookup.setGeometry(QtCore.QRect(470, 10, 21, 21))
        self.btn_obj_import_lookup.setObjectName("btn_obj_import_lookup")
        self.lbl_info_obj_import_path = QtWidgets.QLabel(self.tab_obj_import)
        self.lbl_info_obj_import_path.setGeometry(QtCore.QRect(21, 12, 71, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_obj_import_path.setFont(font)
        self.lbl_info_obj_import_path.setObjectName("lbl_info_obj_import_path")
        self.btn_obj_import = QtWidgets.QPushButton(self.tab_obj_import)
        self.btn_obj_import.setEnabled(False)
        self.btn_obj_import.setGeometry(QtCore.QRect(420, 90, 75, 23))
        self.btn_obj_import.setObjectName("btn_obj_import")
        self.txt_obj_import_scale = QtWidgets.QDoubleSpinBox(self.tab_obj_import)
        self.txt_obj_import_scale.setGeometry(QtCore.QRect(96, 40, 61, 22))
        self.txt_obj_import_scale.setProperty("value", 1.0)
        self.txt_obj_import_scale.setObjectName("txt_obj_import_scale")
        self.lbl_info_obj_import_scale = QtWidgets.QLabel(self.tab_obj_import)
        self.lbl_info_obj_import_scale.setGeometry(QtCore.QRect(20, 43, 81, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_obj_import_scale.setFont(font)
        self.lbl_info_obj_import_scale.setObjectName("lbl_info_obj_import_scale")
        self.tabObjectMode.addTab(self.tab_obj_import, "")
        self.tab_obj_scan = QtWidgets.QWidget()
        self.tab_obj_scan.setObjectName("tab_obj_scan")
        self.btn_scan_start = QtWidgets.QPushButton(self.tab_obj_scan)
        self.btn_scan_start.setGeometry(QtCore.QRect(310, 90, 75, 23))
        self.btn_scan_start.setObjectName("btn_scan_start")
        self.pbar_scan = QtWidgets.QProgressBar(self.tab_obj_scan)
        self.pbar_scan.setGeometry(QtCore.QRect(390, 90, 118, 23))
        self.pbar_scan.setProperty("value", 42)
        self.pbar_scan.setObjectName("pbar_scan")
        self.grp_scan_dims = QtWidgets.QGroupBox(self.tab_obj_scan)
        self.grp_scan_dims.setGeometry(QtCore.QRect(10, 10, 481, 51))
        self.grp_scan_dims.setObjectName("grp_scan_dims")
        self.lbl_info_m1_3 = QtWidgets.QLabel(self.grp_scan_dims)
        self.lbl_info_m1_3.setEnabled(True)
        self.lbl_info_m1_3.setGeometry(QtCore.QRect(94, 20, 31, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_m1_3.setFont(font)
        self.lbl_info_m1_3.setObjectName("lbl_info_m1_3")
        self.txt_obj_scan_x = QtWidgets.QDoubleSpinBox(self.grp_scan_dims)
        self.txt_obj_scan_x.setEnabled(True)
        self.txt_obj_scan_x.setGeometry(QtCore.QRect(28, 17, 62, 22))
        self.txt_obj_scan_x.setObjectName("txt_obj_scan_x")
        self.lbl_info_m2_3 = QtWidgets.QLabel(self.grp_scan_dims)
        self.lbl_info_m2_3.setEnabled(True)
        self.lbl_info_m2_3.setGeometry(QtCore.QRect(258, 20, 31, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_m2_3.setFont(font)
        self.lbl_info_m2_3.setObjectName("lbl_info_m2_3")
        self.txt_obj_scan_y = QtWidgets.QDoubleSpinBox(self.grp_scan_dims)
        self.txt_obj_scan_y.setEnabled(True)
        self.txt_obj_scan_y.setGeometry(QtCore.QRect(192, 17, 62, 22))
        self.txt_obj_scan_y.setObjectName("txt_obj_scan_y")
        self.lbl_info_m3_3 = QtWidgets.QLabel(self.grp_scan_dims)
        self.lbl_info_m3_3.setEnabled(True)
        self.lbl_info_m3_3.setGeometry(QtCore.QRect(448, 20, 31, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_m3_3.setFont(font)
        self.lbl_info_m3_3.setObjectName("lbl_info_m3_3")
        self.txt_obj_scan_z = QtWidgets.QDoubleSpinBox(self.grp_scan_dims)
        self.txt_obj_scan_z.setEnabled(True)
        self.txt_obj_scan_z.setGeometry(QtCore.QRect(382, 17, 62, 22))
        self.txt_obj_scan_z.setObjectName("txt_obj_scan_z")
        self.label_2 = QtWidgets.QLabel(self.grp_scan_dims)
        self.label_2.setGeometry(QtCore.QRect(9, 20, 16, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.grp_scan_dims)
        self.label_3.setGeometry(QtCore.QRect(173, 20, 16, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.grp_scan_dims)
        self.label_4.setGeometry(QtCore.QRect(363, 20, 16, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.tabObjectMode.addTab(self.tab_obj_scan, "")
        self.btn_obj_send_to_sim = QtWidgets.QPushButton(self.tab_open3d)
        self.btn_obj_send_to_sim.setEnabled(False)
        self.btn_obj_send_to_sim.setGeometry(QtCore.QRect(530, 10, 121, 121))
        self.btn_obj_send_to_sim.setObjectName("btn_obj_send_to_sim")
        self.widget_window_tabs.addTab(self.tab_open3d, "")
        self.tab_data_visualizer = QtWidgets.QWidget()
        self.tab_data_visualizer.setObjectName("tab_data_visualizer")
        self.widget_window_tabs.addTab(self.tab_data_visualizer, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.widget_window_tabs.setCurrentIndex(1)
        self.tabObjectMode.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.lbl_info_a.setVisible(False)
        self.lbl_info_b.setVisible(False)
        self.lbl_info_c.setVisible(False)
        self.lbl_info_m1.setVisible(False)
        self.lbl_info_m2.setVisible(False)
        self.lbl_info_m3.setVisible(False)
        self.txt_obj_a.setVisible(False)
        self.txt_obj_b.setVisible(False)
        self.txt_obj_c.setVisible(False)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Command Window"))
        self.grp_robot.setTitle(_translate("MainWindow", "Robot Information"))
        self.lbl_info_rbtstatus.setText(_translate("MainWindow", "Robot Status: "))
        self.lbl_rbt_status.setText(_translate("MainWindow", "OFF"))
        self.btn_rbt_terminate.setText(_translate("MainWindow", "Terminate"))
        self.btn_rbt_info.setText(_translate("MainWindow", "Info"))
        self.lbl_ismoving_rail.setText(_translate("MainWindow", "L"))
        self.lbl_ismoving_wrist.setText(_translate("MainWindow", "W"))
        self.lbl_ismoving_shoulder.setText(_translate("MainWindow", "S"))
        self.lbl_ismoving_elbow.setText(_translate("MainWindow", "E"))
        self.lbl_info_activejoints.setText(_translate("MainWindow", "Active Joints:"))
        self.lbl_info_currentgoal.setText(_translate("MainWindow", "Goal State:"))
        self.lbl_info_currentgoal_2.setText(_translate("MainWindow", "Probing Time:"))
        self.lbl_rbt_status_2.setText(_translate("MainWindow", "[2, 1, 0, -1]"))
        self.lbl_rbt_status_3.setText(_translate("MainWindow", "0000.0000"))
        self.lbl_info_distance.setText(_translate("MainWindow", "Probe Distance:"))
        self.lbl_distance_rbt_obj.setText(_translate("MainWindow", "0.0000"))
        self.grp_simulation.setTitle(_translate("MainWindow", "Simulation Information"))
        self.lbl_info_simstatus.setText(_translate("MainWindow", "Simulation Status:"))
        self.lbl_sim_status.setText(_translate("MainWindow", "OFF"))
        self.btn_rbt_terminate_2.setText(_translate("MainWindow", "Terminate"))
        self.lbl_info_simstatus_2.setText(_translate("MainWindow", "Object Data:"))
        self.lbl_sim_status_2.setText(_translate("MainWindow", "NONE"))
        self.pushButton.setText(_translate("MainWindow", "Probing Helper"))
        self.grp_proginfo.setTitle(_translate("MainWindow", "Program Information"))
        self.lbl_info_time.setText(_translate("MainWindow", "Time Elapsed:"))
        self.lbl_info_processtime.setText(_translate("MainWindow", "Process Time:"))
        self.lbl_time.setText(_translate("MainWindow", "00000.0000"))
        self.lbl_dt.setText(_translate("MainWindow", "00000.0000"))
        self.btn_edit_consts.setText(_translate("MainWindow", "Constraint Editor"))
        self.lbl_info_updaterate.setText(_translate("MainWindow", "Update Rate:"))
        self.lbl_updaterate.setText(_translate("MainWindow", "120 /s"))
        self.btn_shutdown_program.setText(_translate("MainWindow", "Shutdown"))
        self.widget_window_tabs.setTabText(self.widget_window_tabs.indexOf(self.tab_pybullet), _translate("MainWindow", "Simulation"))
        self.cb_primitive_select.setItemText(0, _translate("MainWindow", "Select Primitive..."))
        self.cb_primitive_select.setItemText(1, _translate("MainWindow", "Sphere"))
        self.cb_primitive_select.setItemText(2, _translate("MainWindow", "Rectangular Prism"))
        self.cb_primitive_select.setItemText(3, _translate("MainWindow", "Cylinder"))
        self.btn_obj_create.setText(_translate("MainWindow", "Create"))
        self.lbl_info_m1.setText(_translate("MainWindow", "mm"))
        self.lbl_info_m2.setText(_translate("MainWindow", "mm"))
        self.lbl_info_m3.setText(_translate("MainWindow", "mm"))
        self.lbl_info_a.setText(_translate("MainWindow", "Radius:"))
        self.lbl_info_b.setText(_translate("MainWindow", "Height: (Z)"))
        self.lbl_info_c.setText(_translate("MainWindow", "Height: (Z)"))
        self.tabObjectMode.setTabText(self.tabObjectMode.indexOf(self.tab_obj_primitives), _translate("MainWindow", "Primitives"))
        self.btn_obj_import_lookup.setText(_translate("MainWindow", "..."))
        self.lbl_info_obj_import_path.setText(_translate("MainWindow", "Object Path:"))
        self.btn_obj_import.setText(_translate("MainWindow", "Import"))
        self.lbl_info_obj_import_scale.setText(_translate("MainWindow", "Object Scale:"))
        self.tabObjectMode.setTabText(self.tabObjectMode.indexOf(self.tab_obj_import), _translate("MainWindow", "Import"))
        self.btn_scan_start.setText(_translate("MainWindow", "Start Scan"))
        self.grp_scan_dims.setTitle(_translate("MainWindow", "Object Dimensions"))
        self.lbl_info_m1_3.setText(_translate("MainWindow", "mm"))
        self.lbl_info_m2_3.setText(_translate("MainWindow", "mm"))
        self.lbl_info_m3_3.setText(_translate("MainWindow", "mm"))
        self.label_2.setText(_translate("MainWindow", "X"))
        self.label_3.setText(_translate("MainWindow", "Y"))
        self.label_4.setText(_translate("MainWindow", "Z"))
        self.tabObjectMode.setTabText(self.tabObjectMode.indexOf(self.tab_obj_scan), _translate("MainWindow", "Scan"))
        self.btn_obj_send_to_sim.setText(_translate("MainWindow", "Send To Simulation"))
        self.widget_window_tabs.setTabText(self.widget_window_tabs.indexOf(self.tab_open3d), _translate("MainWindow", "Object Visualizer"))
        self.widget_window_tabs.setTabText(self.widget_window_tabs.indexOf(self.tab_data_visualizer), _translate("MainWindow", "Data Visualizer"))
