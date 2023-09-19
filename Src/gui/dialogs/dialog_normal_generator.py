# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dialog_probe_helper.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QDialog


class UiDialogNormalGenerator(object):
    def setupUi(self, NormalGenerator):
        NormalGenerator.setObjectName("NormalGenerator")
        NormalGenerator.resize(280, 185)
        NormalGenerator.setMinimumSize(QtCore.QSize(280, 185))
        NormalGenerator.setMaximumSize(QtCore.QSize(280, 185))
        self.formLayoutWidget_2 = QtWidgets.QWidget(NormalGenerator)
        self.formLayoutWidget_2.setGeometry(QtCore.QRect(10, 10, 241, 21))
        self.formLayoutWidget_2.setObjectName("formLayoutWidget_2")
        self.formLayout_2 = QtWidgets.QFormLayout(self.formLayoutWidget_2)
        self.formLayout_2.setContentsMargins(0, 0, 0, 0)
        self.formLayout_2.setObjectName("formLayout_2")
        self.lbl_info_probe_points = QtWidgets.QLabel(self.formLayoutWidget_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_probe_points.setFont(font)
        self.lbl_info_probe_points.setObjectName("lbl_info_probe_points")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lbl_info_probe_points)
        self.lbl_probe_points = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.lbl_probe_points.setObjectName("lbl_probe_points")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.lbl_probe_points)
        self.grp_parameters = QtWidgets.QGroupBox(NormalGenerator)
        self.grp_parameters.setGeometry(QtCore.QRect(10, 40, 261, 111))
        self.grp_parameters.setObjectName("grp_parameters")
        self.formLayoutWidget = QtWidgets.QWidget(self.grp_parameters)
        self.formLayoutWidget.setGeometry(QtCore.QRect(10, 20, 241, 80))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setObjectName("formLayout")
        self.lbl_info_scan_res = QtWidgets.QLabel(self.formLayoutWidget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_scan_res.setFont(font)
        self.lbl_info_scan_res.setObjectName("lbl_info_scan_res")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lbl_info_scan_res)
        self.lbl_info_scan_z = QtWidgets.QLabel(self.formLayoutWidget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_scan_z.setFont(font)
        self.lbl_info_scan_z.setObjectName("lbl_info_scan_z")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.lbl_info_scan_z)
        self.lbl_info_probe_distance = QtWidgets.QLabel(self.formLayoutWidget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_probe_distance.setFont(font)
        self.lbl_info_probe_distance.setObjectName("lbl_info_probe_distance")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.lbl_info_probe_distance)
        self.sbox_scan_resolution = QtWidgets.QDoubleSpinBox(self.formLayoutWidget)
        self.sbox_scan_resolution.setMaximum(720.0)
        self.sbox_scan_resolution.setProperty("value", 180.0)
        self.sbox_scan_resolution.setObjectName("sbox_scan_resolution")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.sbox_scan_resolution)
        self.sbox_scan_z = QtWidgets.QDoubleSpinBox(self.formLayoutWidget)
        self.sbox_scan_z.setProperty("value", 1.0)
        self.sbox_scan_z.setObjectName("sbox_scan_z")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.sbox_scan_z)
        self.sbox_probe_distance_2 = QtWidgets.QDoubleSpinBox(self.formLayoutWidget)
        self.sbox_probe_distance_2.setProperty("value", 10.0)
        self.sbox_probe_distance_2.setObjectName("sbox_probe_distance_2")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.sbox_probe_distance_2)
        self.btn_generate_normals = QtWidgets.QPushButton(NormalGenerator)
        self.btn_generate_normals.setGeometry(QtCore.QRect(10, 150, 261, 23))
        self.btn_generate_normals.setObjectName("btn_generate_normals")

        self.retranslateUi(NormalGenerator)
        QtCore.QMetaObject.connectSlotsByName(NormalGenerator)

    def retranslateUi(self, NormalGenerator):
        _translate = QtCore.QCoreApplication.translate
        NormalGenerator.setWindowTitle(_translate("NormalGenerator", "Normal Generator"))
        self.lbl_info_probe_points.setText(_translate("NormalGenerator", "Generated Normal Points:"))
        self.lbl_probe_points.setText(_translate("NormalGenerator", "0"))
        self.grp_parameters.setTitle(_translate("NormalGenerator", "Normal Generation Parameters"))
        self.lbl_info_scan_res.setText(_translate("NormalGenerator", "Normal Resoultion (Deg):"))
        self.lbl_info_scan_z.setText(_translate("NormalGenerator", "Normal Z Density (mm):"))
        self.lbl_info_probe_distance.setText(_translate("NormalGenerator", "Probe Distance (mm):"))
        self.btn_generate_normals.setText(_translate("NormalGenerator", "Generate Normals"))

class DialogNormalGenerator(QDialog):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiDialogNormalGenerator()
        self.ui.setupUi(self)

        self.parent_instance = parent
        self.controller_instance: Controller = parent.controller

        self.ui.btn_generate_normals.clicked.connect(self.generate_normals)

    def generate_normals(self):
        cloud = simhelper.get_object_point_cloud(
            probe_dist=(float(self.ui.sbox_probe_distance.text()) / 1000) * sim_constants.SIM_SCALE,
            resolution=(int(float(self.ui.sbox_scan_resolution.text()))),
            z_density=(int(float(self.ui.sbox_scan_z.text())) * sim_constants.SIM_SCALE) * 10
        )

        if self.controller_instance.simulation_instance.current_point_cloud is not None:
            self.controller_instance.simulation_instance.current_point_cloud.delete_cloud()
            self.controller_instance.simulation_instance.current_point_cloud = None

        self.controller_instance.simulation_instance.current_point_cloud = cloud

        self.ui.lbl_probe_points.setText(str(cloud.get_num_points()))
        self.close()