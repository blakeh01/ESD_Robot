from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QDialog, QInputDialog, QMessageBox

from src.Controller import Controller
from src.sim.scan_algo.ProbingFlowManager import Discharge, Charge, Wait, Probe, RectangularPrism, RotationallySymmetric


class Ui_CreateProbeProfile(object):
    def setupUi(self, CreateProbeProfile):
        CreateProbeProfile.setObjectName("CreateProbeProfile")
        CreateProbeProfile.resize(450, 300)
        CreateProbeProfile.setMinimumSize(QtCore.QSize(450, 300))
        CreateProbeProfile.setMaximumSize(QtCore.QSize(450, 300))
        self.formLayoutWidget = QtWidgets.QWidget(CreateProbeProfile)
        self.formLayoutWidget.setGeometry(QtCore.QRect(10, 10, 251, 22))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setObjectName("formLayout")
        self.lbl_object_profile = QtWidgets.QLabel(self.formLayoutWidget)
        self.lbl_object_profile.setObjectName("lbl_object_profile")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lbl_object_profile)
        self.cmb_object_profile = QtWidgets.QComboBox(self.formLayoutWidget)
        self.cmb_object_profile.setObjectName("cmb_object_profile")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.cmb_object_profile)
        self.grp_probing_param = QtWidgets.QGroupBox(CreateProbeProfile)
        self.grp_probing_param.setGeometry(QtCore.QRect(10, 40, 251, 221))
        self.grp_probing_param.setObjectName("grp_probing_param")
        self.formLayoutWidget_2 = QtWidgets.QWidget(self.grp_probing_param)
        self.formLayoutWidget_2.setGeometry(QtCore.QRect(10, 19, 231, 191))
        self.formLayoutWidget_2.setObjectName("formLayoutWidget_2")
        self.formLayout_2 = QtWidgets.QFormLayout(self.formLayoutWidget_2)
        self.formLayout_2.setContentsMargins(0, 0, 0, 0)
        self.formLayout_2.setObjectName("formLayout_2")
        self.lbl_max_speed = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.lbl_max_speed.setObjectName("lbl_max_speed")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lbl_max_speed)
        self.txt_max_speed = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.txt_max_speed.setObjectName("txt_max_speed")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txt_max_speed)
        self.lbl_plat_feedrate = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.lbl_plat_feedrate.setObjectName("lbl_plat_feedrate")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.lbl_plat_feedrate)
        self.txt_rotator_feedrate = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.txt_rotator_feedrate.setObjectName("txt_rotator_feedrate")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.txt_rotator_feedrate)
        self.lbl_grouding_interval = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.lbl_grouding_interval.setObjectName("lbl_grouding_interval")
        self.formLayout_2.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.lbl_grouding_interval)
        self.txt_grounding_interval = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.txt_grounding_interval.setObjectName("txt_grounding_interval")
        self.formLayout_2.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.txt_grounding_interval)
        self.lbl_measuring_time = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.lbl_measuring_time.setObjectName("lbl_measuring_time")
        self.formLayout_2.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.lbl_measuring_time)
        self.txt_measuring_time = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.txt_measuring_time.setObjectName("txt_measuring_time")
        self.formLayout_2.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.txt_measuring_time)
        self.btn_done = QtWidgets.QPushButton(CreateProbeProfile)
        self.btn_done.setGeometry(QtCore.QRect(370, 270, 71, 23))
        self.btn_done.setObjectName("btn_done")
        self.verticalLayoutWidget = QtWidgets.QWidget(CreateProbeProfile)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(270, 10, 101, 251))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setUnderline(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.list_probe_flow = QtWidgets.QListWidget(self.verticalLayoutWidget)
        self.list_probe_flow.setObjectName("list_probe_flow")
        self.verticalLayout.addWidget(self.list_probe_flow)
        self.btn_charge = QtWidgets.QPushButton(CreateProbeProfile)
        self.btn_charge.setGeometry(QtCore.QRect(380, 30, 50, 50))
        font = QtGui.QFont()
        font.setPointSize(7)
        self.btn_charge.setFont(font)
        self.btn_charge.setObjectName("btn_charge")
        self.btn_discharge = QtWidgets.QPushButton(CreateProbeProfile)
        self.btn_discharge.setGeometry(QtCore.QRect(380, 90, 50, 50))
        font = QtGui.QFont()
        font.setPointSize(7)
        self.btn_discharge.setFont(font)
        self.btn_discharge.setObjectName("btn_discharge")
        self.btn_probe = QtWidgets.QPushButton(CreateProbeProfile)
        self.btn_probe.setGeometry(QtCore.QRect(380, 150, 50, 50))
        font = QtGui.QFont()
        font.setPointSize(7)
        self.btn_probe.setFont(font)
        self.btn_probe.setObjectName("btn_probe")
        self.btn_wait = QtWidgets.QPushButton(CreateProbeProfile)
        self.btn_wait.setGeometry(QtCore.QRect(380, 210, 50, 50))
        font = QtGui.QFont()
        font.setPointSize(7)
        self.btn_wait.setFont(font)
        self.btn_wait.setObjectName("btn_wait")

        self.retranslateUi(CreateProbeProfile)
        QtCore.QMetaObject.connectSlotsByName(CreateProbeProfile)

    def retranslateUi(self, CreateProbeProfile):
        _translate = QtCore.QCoreApplication.translate
        CreateProbeProfile.setWindowTitle(_translate("CreateProbeProfile", "Create Probing Profile"))
        self.lbl_object_profile.setText(_translate("CreateProbeProfile", "Object Profile:"))
        self.grp_probing_param.setTitle(_translate("CreateProbeProfile", "Probing Parameters"))
        self.lbl_max_speed.setText(_translate("CreateProbeProfile", "Probe Max Speed (mm/s):"))
        self.lbl_plat_feedrate.setText(_translate("CreateProbeProfile", "Rotator Feedrate (deg/min):"))
        self.lbl_grouding_interval.setText(_translate("CreateProbeProfile", "Grounding Interval (s):"))
        self.lbl_measuring_time.setText(_translate("CreateProbeProfile", "Measuring Time (ms):"))
        self.btn_done.setText(_translate("CreateProbeProfile", "Done"))
        self.label.setText(_translate("CreateProbeProfile", "PROBE FLOW"))
        self.btn_charge.setText(_translate("CreateProbeProfile", "Charge"))
        self.btn_discharge.setText(_translate("CreateProbeProfile", "Discharge"))
        self.btn_probe.setText(_translate("CreateProbeProfile", "Probe"))
        self.btn_wait.setText(_translate("CreateProbeProfile", "Wait"))

        self.txt_max_speed.setText("20")
        self.txt_rotator_feedrate.setText("3000")
        self.txt_measuring_time.setText("250")
        self.txt_grounding_interval.setText("10")


class DialogProbeProfile(QDialog):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_CreateProbeProfile()
        self.ui.setupUi(self)

        self.parent_instance = parent
        self.controller_instance: Controller = parent.controller
        self.sim_instance = self.controller_instance.simulation_instance
        self.robot_instance = self.controller_instance.robot_instance
        self.stepper_instance = self.controller_instance.simulation_instance
        self.feather = self.controller_instance.feather_instance


        # add the supported object profiles
        self.ui.cmb_object_profile.addItem("Rot. Symmetric Geometry")
        self.ui.cmb_object_profile.addItem("Rectangular Geometry")

        self.ui.btn_probe.clicked.connect(self.add_probe)
        self.ui.btn_wait.clicked.connect(self.add_wait)
        self.ui.btn_charge.clicked.connect(self.add_charge)
        self.ui.btn_discharge.clicked.connect(self.add_discharge)

        self.ui.list_probe_flow.doubleClicked.connect(self.remove_arg)

        self.ui.btn_done.clicked.connect(self.set_flow)

        self.flow_list = []

    def add_item(self, item_text):
        self.ui.list_probe_flow.addItem(item_text)

    def remove_arg(self):
        items = self.ui.list_probe_flow.selectedIndexes()

        for i in items:
            self.ui.list_probe_flow.takeItem(i.row())
            self.flow_list.pop(i.row())

    def add_wait(self):
        value, ok = QInputDialog.getText(self, 'Enter Wait Time', 'Wait Time (s):')

        if ok:
            try:
                wait_time = abs(int(value))
                self.add_item(f"Wait: {wait_time} s")
                self.flow_list.append(Wait(wait_time))
            except ValueError:
                self.handle_input_error()

    def add_discharge(self):
        value, ok = QInputDialog.getText(self, 'Enter CVR Probe Length', 'Length (mm):')

        if ok:
            try:
                cvr_len = abs(int(value))
                self.add_item(f"Discharge")
                self.flow_list.append(Discharge(self.controller_instance.stepper_controller,
                                                self.controller_instance.lds_instance,
                                                self.controller_instance.obj_height,
                                                cvr_len))
            except ValueError:
                self.handle_input_error()

    def add_charge(self):
        value, ok = QInputDialog.getText(self, 'Enter Charge Duration (0 for manual)', 'Duration (s):')

        if ok:
            try:
                duration = abs(int(value))
                is_manual = duration > 0

                self.add_item("Charge" + ((': ' + str(duration)) if not is_manual else ''))
                self.flow_list.append(Charge(duration))

            except ValueError:
                self.handle_input_error()

    def add_probe(self):
        self.add_item(f"Probe")
        self.flow_list.append(Probe())

    def set_flow(self):
        if len(self.flow_list) == 0:
            self.handle_input_error("Empty Flow List!", "Please add action arguments to this program!")

        try:
            flow_args = [
                int(self.ui.txt_max_speed.text()),
                int(self.ui.txt_rotator_feedrate.text()),
                int(self.ui.txt_grounding_interval.text()),
                int(self.ui.txt_measuring_time.text()) / 1000
            ]
        except ValueError:
            self.handle_input_error()
            return

        match self.ui.cmb_object_profile.currentIndex():

            case 0:  # rot symm
                self.controller_instance.simulation_instance.cur_probe_flow = RotationallySymmetric(
                    self.sim_instance, self.robot_instance, self.feather, self.flow_list,
                    flow_args
                )

            case 1:  # rect
                self.controller_instance.simulation_instance.cur_probe_flow = RectangularPrism(
                    self.sim_instance, self.robot_instance, self.feather, self.flow_list,
                    flow_args
                )

        self.close()

    def handle_input_error(self, title="Input Error", text="Invalid input. Please enter a valid value."):
        error_box = QMessageBox()
        error_box.setIcon(QMessageBox.Warning)
        error_box.setWindowTitle(title)
        error_box.setText(text)
        error_box.exec_()
