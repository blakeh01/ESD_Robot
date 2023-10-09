from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QDialog, QInputDialog

from Src.sim.scan_algo.ObjectProfile import *
from Src.sim.scan_algo.ObjectProfile import Discharge, Charge, Wait, Probe


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
        self.controller_instance = parent.controller

        self.ui.cmb_object_profile.addItem("Rot. Symmetric Geometry")
        self.ui.cmb_object_profile.addItem("Rectangular Geometry")

        self.ui.btn_probe.clicked.connect(lambda: self.ui.list_probe_flow.addItem("Probe"))
        self.ui.btn_wait.clicked.connect(self.add_wait)
        self.ui.btn_charge.clicked.connect(lambda: self.ui.list_probe_flow.addItem("Charge"))
        self.ui.btn_discharge.clicked.connect(self.add_discharge)

        self.ui.list_probe_flow.doubleClicked.connect(self.remove_arg)

        self.ui.btn_done.clicked.connect(self.parse_flow)

    def remove_arg(self):
        items = self.ui.list_probe_flow.selectedIndexes()

        for i in items:
            self.ui.list_probe_flow.takeItem(i.row())

    def add_wait(self):
        value, ok = QInputDialog.getText(self, 'Enter Wait Time', 'Wait Time (s):')

        if ok:
            try:
                self.ui.list_probe_flow.addItem("Wait: " + str(abs(int(value))) + " s")
            except:
                return  # todo error handling?

    def add_discharge(self):
        value, ok = QInputDialog.getText(self, "Enter Object Height", 'Object Height (mm)')

        if ok:
            try:
                self.ui.list_probe_flow.addItem("Discharge: " + str(abs(int(value))) + " mm")
            except:
                return

    def parse_flow(self):
        if self.ui.list_probe_flow.count() <= 0:
            return

        try:
            if int(self.ui.txt_max_speed.text()) <= 0 or int(self.ui.txt_measuring_time.text()) < 0 \
                    or int(self.ui.txt_rotator_feedrate.text()) <= 0 or int(self.ui.txt_grounding_interval.text()) <= 0:
                return
        except:
            return  # error handling for all this. PLEASE

        flow_arr = []

        for i in range(self.ui.list_probe_flow.count()):
            item = self.ui.list_probe_flow.item(i).text()

            data = item.split(':')

            # yikes... works but yikess...
            if len(data) > 1:
                d = data[1].split(' ')

                if d[2] == 'mm':
                    flow_arr.append(Discharge(self.controller_instance.simulation_instance.port_config, int(d[1])))
                else:
                    flow_arr.append(Wait(int(d[1])))
            else:
                if (len(item) == 9): flow_arr.append(Discharge())
                if (len(item) == 6): flow_arr.append(Charge())
                if (len(item) == 5): flow_arr.append(Probe())

        match self.ui.cmb_object_profile.currentIndex():
            case 0:  # rot symm
                self.controller_instance.simulation_instance.cur_probe_flow = RotationallySymmetric(
                    self.controller_instance.simulation_instance, flow_arr,
                    [self.ui.txt_max_speed.text(), self.ui.txt_rotator_feedrate.text(),
                     self.ui.txt_grounding_interval.text(), self.ui.txt_measuring_time.text()]
                )

            case 1:  # rect
                self.controller_instance.simulation_instance.cur_probe_flow = RectangularPrisms(
                    self.controller_instance.simulation_instance, flow_arr,
                    [self.ui.txt_max_speed.text(), self.ui.txt_rotator_feedrate.text(),
                     self.ui.txt_grounding_interval.text(), self.ui.txt_measuring_time.text()]
                )

        self.close()
