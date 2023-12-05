# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'object_wizard.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_ObjectWizard(object):
    def setupUi(self, ObjectWizard):
        ObjectWizard.setObjectName("ObjectWizard")
        ObjectWizard.resize(575, 400)
        ObjectWizard.setWizardStyle(QtWidgets.QWizard.ModernStyle)
        ObjectWizard.setOptions(QtWidgets.QWizard.HelpButtonOnRight|QtWidgets.QWizard.IndependentPages|QtWidgets.QWizard.NoCancelButton)
        self.wiz_page_method = QtWidgets.QWizardPage()
        self.wiz_page_method.setObjectName("wiz_page_method")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.wiz_page_method)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(30, 30, 441, 81))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.rbt_primitive = QtWidgets.QRadioButton(self.verticalLayoutWidget)
        self.rbt_primitive.setObjectName("rbt_primitive")
        self.verticalLayout.addWidget(self.rbt_primitive)
        self.rbtn_import_obj = QtWidgets.QRadioButton(self.verticalLayoutWidget)
        self.rbtn_import_obj.setObjectName("rbtn_import_obj")
        self.verticalLayout.addWidget(self.rbtn_import_obj)
        self.rbt_scan_obj = QtWidgets.QRadioButton(self.verticalLayoutWidget)
        self.rbt_scan_obj.setObjectName("rbt_scan_obj")
        self.verticalLayout.addWidget(self.rbt_scan_obj)
        ObjectWizard.addPage(self.wiz_page_method)
        self.wiz_page_create_primitive = QtWidgets.QWizardPage()
        self.wiz_page_create_primitive.setObjectName("wiz_page_create_primitive")
        self.gridLayoutWidget = QtWidgets.QWidget(self.wiz_page_create_primitive)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(40, 10, 461, 270))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.groupBox = QtWidgets.QGroupBox(self.gridLayoutWidget)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.groupBox)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(20, 20, 160, 80))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.rbtn_prim_rect = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.rbtn_prim_rect.setChecked(True)
        self.rbtn_prim_rect.setObjectName("rbtn_prim_rect")
        self.verticalLayout_2.addWidget(self.rbtn_prim_rect)
        self.rbtn_prim_cylinder = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.rbtn_prim_cylinder.setObjectName("rbtn_prim_cylinder")
        self.verticalLayout_2.addWidget(self.rbtn_prim_cylinder)
        self.rbtn_prim_sphere = QtWidgets.QRadioButton(self.verticalLayoutWidget_2)
        self.rbtn_prim_sphere.setObjectName("rbtn_prim_sphere")
        self.verticalLayout_2.addWidget(self.rbtn_prim_sphere)
        self.gridLayout.addWidget(self.groupBox, 0, 0, 1, 1)
        self.groupBox_2 = QtWidgets.QGroupBox(self.gridLayoutWidget)
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_2)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(10, 20, 201, 141))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.lbl_prim_field_A = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.lbl_prim_field_A.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_prim_field_A.setObjectName("lbl_prim_field_A")
        self.gridLayout_2.addWidget(self.lbl_prim_field_A, 0, 0, 1, 1)
        self.sbox_prim_field_B = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_2)
        self.sbox_prim_field_B.setDecimals(2)
        self.sbox_prim_field_B.setMinimum(1.0)
        self.sbox_prim_field_B.setMaximum(9999.0)
        self.sbox_prim_field_B.setObjectName("sbox_prim_field_B")
        self.gridLayout_2.addWidget(self.sbox_prim_field_B, 1, 1, 1, 1)
        self.sbox_prim_field_A = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_2)
        self.sbox_prim_field_A.setDecimals(2)
        self.sbox_prim_field_A.setMinimum(1.0)
        self.sbox_prim_field_A.setMaximum(9999.0)
        self.sbox_prim_field_A.setObjectName("sbox_prim_field_A")
        self.gridLayout_2.addWidget(self.sbox_prim_field_A, 0, 1, 1, 1)
        self.lbl_prim_field_units_B = QtWidgets.QLabel(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_prim_field_units_B.setFont(font)
        self.lbl_prim_field_units_B.setObjectName("lbl_prim_field_units_B")
        self.gridLayout_2.addWidget(self.lbl_prim_field_units_B, 1, 2, 1, 1)
        self.lbl_prim_field_units_A = QtWidgets.QLabel(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_prim_field_units_A.setFont(font)
        self.lbl_prim_field_units_A.setObjectName("lbl_prim_field_units_A")
        self.gridLayout_2.addWidget(self.lbl_prim_field_units_A, 0, 2, 1, 1)
        self.lbl_prim_field_B = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.lbl_prim_field_B.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_prim_field_B.setObjectName("lbl_prim_field_B")
        self.gridLayout_2.addWidget(self.lbl_prim_field_B, 1, 0, 1, 1)
        self.lbl_prim_field_C = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.lbl_prim_field_C.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_prim_field_C.setObjectName("lbl_prim_field_C")
        self.gridLayout_2.addWidget(self.lbl_prim_field_C, 2, 0, 1, 1)
        self.sbox_prim_field_C = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_2)
        self.sbox_prim_field_C.setDecimals(2)
        self.sbox_prim_field_C.setMinimum(1.0)
        self.sbox_prim_field_C.setMaximum(9999.0)
        self.sbox_prim_field_C.setObjectName("sbox_prim_field_C")
        self.gridLayout_2.addWidget(self.sbox_prim_field_C, 2, 1, 1, 1)
        self.lbl_prim_field_units_C = QtWidgets.QLabel(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_prim_field_units_C.setFont(font)
        self.lbl_prim_field_units_C.setObjectName("lbl_prim_field_units_C")
        self.gridLayout_2.addWidget(self.lbl_prim_field_units_C, 2, 2, 1, 1)
        self.gridLayout.addWidget(self.groupBox_2, 0, 1, 1, 1)
        ObjectWizard.addPage(self.wiz_page_create_primitive)
        self.wiz_page_import_obj = QtWidgets.QWizardPage()
        self.wiz_page_import_obj.setObjectName("wiz_page_import_obj")
        self.gridLayoutWidget_3 = QtWidgets.QWidget(self.wiz_page_import_obj)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(30, 30, 481, 80))
        self.gridLayoutWidget_3.setObjectName("gridLayoutWidget_3")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.txt_path = QtWidgets.QLineEdit(self.gridLayoutWidget_3)
        self.txt_path.setObjectName("txt_path")
        self.gridLayout_3.addWidget(self.txt_path, 0, 1, 1, 1)
        self.btn_prompt_path = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        self.btn_prompt_path.setObjectName("btn_prompt_path")
        self.gridLayout_3.addWidget(self.btn_prompt_path, 0, 2, 1, 1)
        self.lbl_import_path = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.lbl_import_path.setObjectName("lbl_import_path")
        self.gridLayout_3.addWidget(self.lbl_import_path, 0, 0, 1, 1)
        self.groupBox_3 = QtWidgets.QGroupBox(self.wiz_page_import_obj)
        self.groupBox_3.setGeometry(QtCore.QRect(40, 130, 471, 111))
        self.groupBox_3.setObjectName("groupBox_3")
        self.cbox_import_units = QtWidgets.QComboBox(self.groupBox_3)
        self.cbox_import_units.setGeometry(QtCore.QRect(220, 30, 61, 22))
        self.cbox_import_units.setObjectName("cbox_import_units")
        self.cbox_import_units.addItem("")
        self.cbox_import_units.addItem("")
        self.cbox_import_units.addItem("")
        self.cbox_import_units.addItem("")
        self.cbox_import_units.addItem("")
        self.sbox_import_Y = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.sbox_import_Y.setGeometry(QtCore.QRect(310, 30, 62, 22))
        self.sbox_import_Y.setDecimals(3)
        self.sbox_import_Y.setMaximum(100.0)
        self.sbox_import_Y.setProperty("value", 1.0)
        self.sbox_import_Y.setObjectName("sbox_import_Y")
        self.sbox_import_Z = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.sbox_import_Z.setGeometry(QtCore.QRect(400, 30, 62, 22))
        self.sbox_import_Z.setDecimals(3)
        self.sbox_import_Z.setMaximum(100.0)
        self.sbox_import_Z.setProperty("value", 1.0)
        self.sbox_import_Z.setObjectName("sbox_import_Z")
        self.sbox_import_X = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.sbox_import_X.setGeometry(QtCore.QRect(220, 30, 62, 22))
        self.sbox_import_X.setDecimals(3)
        self.sbox_import_X.setMaximum(100.0)
        self.sbox_import_X.setProperty("value", 1.0)
        self.sbox_import_X.setObjectName("sbox_import_X")
        self.gridLayoutWidget_4 = QtWidgets.QWidget(self.groupBox_3)
        self.gridLayoutWidget_4.setGeometry(QtCore.QRect(10, 20, 131, 80))
        self.gridLayoutWidget_4.setObjectName("gridLayoutWidget_4")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.gridLayoutWidget_4)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.rbtn_import_units = QtWidgets.QRadioButton(self.gridLayoutWidget_4)
        self.rbtn_import_units.setChecked(True)
        self.rbtn_import_units.setObjectName("rbtn_import_units")
        self.gridLayout_4.addWidget(self.rbtn_import_units, 0, 0, 1, 1)
        self.rbtn_import_scale = QtWidgets.QRadioButton(self.gridLayoutWidget_4)
        self.rbtn_import_scale.setObjectName("rbtn_import_scale")
        self.gridLayout_4.addWidget(self.rbtn_import_scale, 1, 0, 1, 1)
        self.lbl_import_X = QtWidgets.QLabel(self.groupBox_3)
        self.lbl_import_X.setGeometry(QtCore.QRect(165, 30, 51, 20))
        self.lbl_import_X.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_import_X.setObjectName("lbl_import_X")
        self.lbl_import_Y = QtWidgets.QLabel(self.groupBox_3)
        self.lbl_import_Y.setGeometry(QtCore.QRect(300, 30, 16, 20))
        self.lbl_import_Y.setObjectName("lbl_import_Y")
        self.lbl_import_Z = QtWidgets.QLabel(self.groupBox_3)
        self.lbl_import_Z.setGeometry(QtCore.QRect(390, 30, 16, 20))
        self.lbl_import_Z.setObjectName("lbl_import_Z")
        ObjectWizard.addPage(self.wiz_page_import_obj)
        self.wiz_page_scan_obj = QtWidgets.QWizardPage()
        self.wiz_page_scan_obj.setObjectName("wiz_page_scan_obj")
        self.groupBox_4 = QtWidgets.QGroupBox(self.wiz_page_scan_obj)
        self.groupBox_4.setGeometry(QtCore.QRect(171, 80, 211, 111))
        self.groupBox_4.setObjectName("groupBox_4")
        self.gridLayoutWidget_5 = QtWidgets.QWidget(self.groupBox_4)
        self.gridLayoutWidget_5.setGeometry(QtCore.QRect(-50, 20, 241, 80))
        self.gridLayoutWidget_5.setObjectName("gridLayoutWidget_5")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.gridLayoutWidget_5)
        self.gridLayout_5.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.lbl_scan_x = QtWidgets.QLabel(self.gridLayoutWidget_5)
        self.lbl_scan_x.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_scan_x.setObjectName("lbl_scan_x")
        self.gridLayout_5.addWidget(self.lbl_scan_x, 0, 0, 1, 1)
        self.lbl_scan_z = QtWidgets.QLabel(self.gridLayoutWidget_5)
        self.lbl_scan_z.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_scan_z.setObjectName("lbl_scan_z")
        self.gridLayout_5.addWidget(self.lbl_scan_z, 2, 0, 1, 1)
        self.lbl_scan_y = QtWidgets.QLabel(self.gridLayoutWidget_5)
        self.lbl_scan_y.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_scan_y.setObjectName("lbl_scan_y")
        self.gridLayout_5.addWidget(self.lbl_scan_y, 1, 0, 1, 1)
        self.sbox_scan_y = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_5)
        self.sbox_scan_y.setDecimals(2)
        self.sbox_scan_y.setMinimum(1.0)
        self.sbox_scan_y.setMaximum(150.0)
        self.sbox_scan_y.setObjectName("sbox_scan_y")
        self.gridLayout_5.addWidget(self.sbox_scan_y, 1, 1, 1, 1)
        self.sbox_scan_z = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_5)
        self.sbox_scan_z.setDecimals(2)
        self.sbox_scan_z.setMinimum(1.0)
        self.sbox_scan_z.setMaximum(300.0)
        self.sbox_scan_z.setObjectName("sbox_scan_z")
        self.gridLayout_5.addWidget(self.sbox_scan_z, 2, 1, 1, 1)
        self.sbox_scan_x = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_5)
        self.sbox_scan_x.setDecimals(2)
        self.sbox_scan_x.setMinimum(1.0)
        self.sbox_scan_x.setMaximum(150.0)
        self.sbox_scan_x.setObjectName("sbox_scan_x")
        self.gridLayout_5.addWidget(self.sbox_scan_x, 0, 1, 1, 1)
        self.lbl_scan_x_units = QtWidgets.QLabel(self.gridLayoutWidget_5)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_scan_x_units.setFont(font)
        self.lbl_scan_x_units.setObjectName("lbl_scan_x_units")
        self.gridLayout_5.addWidget(self.lbl_scan_x_units, 0, 2, 1, 1)
        self.lbl_scan_y_units = QtWidgets.QLabel(self.gridLayoutWidget_5)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_scan_y_units.setFont(font)
        self.lbl_scan_y_units.setObjectName("lbl_scan_y_units")
        self.gridLayout_5.addWidget(self.lbl_scan_y_units, 1, 2, 1, 1)
        self.lbl_scan_z_units = QtWidgets.QLabel(self.gridLayoutWidget_5)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_scan_z_units.setFont(font)
        self.lbl_scan_z_units.setObjectName("lbl_scan_z_units")
        self.gridLayout_5.addWidget(self.lbl_scan_z_units, 2, 2, 1, 1)
        self.btn_start_scan = QtWidgets.QPushButton(self.wiz_page_scan_obj)
        self.btn_start_scan.setGeometry(QtCore.QRect(171, 190, 211, 23))
        self.btn_start_scan.setObjectName("btn_start_scan")
        self.prg_scan = QtWidgets.QProgressBar(self.wiz_page_scan_obj)
        self.prg_scan.setGeometry(QtCore.QRect(206, 260, 161, 23))
        self.prg_scan.setProperty("value", 0)
        self.prg_scan.setTextVisible(False)
        self.prg_scan.setObjectName("prg_scan")
        self.btn_scan_halt = QtWidgets.QPushButton(self.wiz_page_scan_obj)
        self.btn_scan_halt.setGeometry(QtCore.QRect(160, 260, 41, 23))
        self.btn_scan_halt.setObjectName("btn_scan_halt")
        self.lbl_current_points = QtWidgets.QLabel(self.wiz_page_scan_obj)
        self.lbl_current_points.setGeometry(QtCore.QRect(370, 260, 101, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_current_points.setFont(font)
        self.lbl_current_points.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.lbl_current_points.setObjectName("lbl_current_points")
        ObjectWizard.addPage(self.wiz_page_scan_obj)
        self.wiz_page_visualize_obj = QtWidgets.QWizardPage()
        self.wiz_page_visualize_obj.setObjectName("wiz_page_visualize_obj")
        self.widget_visualize = QtWidgets.QOpenGLWidget(self.wiz_page_visualize_obj)
        self.widget_visualize.setGeometry(QtCore.QRect(10, 10, 540, 280))
        self.widget_visualize.setObjectName("widget_visualize")
        ObjectWizard.addPage(self.wiz_page_visualize_obj)
        self.wiz_page_sim_offsets = QtWidgets.QWizardPage()
        self.wiz_page_sim_offsets.setObjectName("wiz_page_sim_offsets")
        self.btn_find = QtWidgets.QPushButton(self.wiz_page_sim_offsets)
        self.btn_find.setEnabled(False)
        self.btn_find.setGeometry(QtCore.QRect(250, 270, 75, 23))
        self.btn_find.setObjectName("btn_find")
        self.tabWidget = QtWidgets.QTabWidget(self.wiz_page_sim_offsets)
        self.tabWidget.setGeometry(QtCore.QRect(20, 50, 231, 241))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_object = QtWidgets.QWidget()
        self.tab_object.setObjectName("tab_object")
        self.grp_object_offsets = QtWidgets.QGroupBox(self.tab_object)
        self.grp_object_offsets.setGeometry(QtCore.QRect(10, 10, 211, 201))
        self.grp_object_offsets.setObjectName("grp_object_offsets")
        self.gridLayoutWidget_6 = QtWidgets.QWidget(self.grp_object_offsets)
        self.gridLayoutWidget_6.setGeometry(QtCore.QRect(-50, 20, 241, 80))
        self.gridLayoutWidget_6.setObjectName("gridLayoutWidget_6")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.gridLayoutWidget_6)
        self.gridLayout_6.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.lbl_offset_y = QtWidgets.QLabel(self.gridLayoutWidget_6)
        self.lbl_offset_y.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_y.setObjectName("lbl_offset_y")
        self.gridLayout_6.addWidget(self.lbl_offset_y, 1, 0, 1, 1)
        self.lbl_offset_y_units = QtWidgets.QLabel(self.gridLayoutWidget_6)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_y_units.setFont(font)
        self.lbl_offset_y_units.setObjectName("lbl_offset_y_units")
        self.gridLayout_6.addWidget(self.lbl_offset_y_units, 1, 2, 1, 1)
        self.sbox_offset_x = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_6)
        self.sbox_offset_x.setDecimals(2)
        self.sbox_offset_x.setMinimum(-150.0)
        self.sbox_offset_x.setMaximum(150.0)
        self.sbox_offset_x.setSingleStep(0.05)
        self.sbox_offset_x.setObjectName("sbox_offset_x")
        self.gridLayout_6.addWidget(self.sbox_offset_x, 0, 1, 1, 1)
        self.sbox_offset_y = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_6)
        self.sbox_offset_y.setDecimals(2)
        self.sbox_offset_y.setMinimum(-150.0)
        self.sbox_offset_y.setMaximum(150.0)
        self.sbox_offset_y.setSingleStep(0.05)
        self.sbox_offset_y.setObjectName("sbox_offset_y")
        self.gridLayout_6.addWidget(self.sbox_offset_y, 1, 1, 1, 1)
        self.lbl_offset_x_units = QtWidgets.QLabel(self.gridLayoutWidget_6)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_x_units.setFont(font)
        self.lbl_offset_x_units.setObjectName("lbl_offset_x_units")
        self.gridLayout_6.addWidget(self.lbl_offset_x_units, 0, 2, 1, 1)
        self.lbl_offset_z = QtWidgets.QLabel(self.gridLayoutWidget_6)
        self.lbl_offset_z.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_z.setObjectName("lbl_offset_z")
        self.gridLayout_6.addWidget(self.lbl_offset_z, 2, 0, 1, 1)
        self.lbl_offset_z_units = QtWidgets.QLabel(self.gridLayoutWidget_6)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_z_units.setFont(font)
        self.lbl_offset_z_units.setObjectName("lbl_offset_z_units")
        self.gridLayout_6.addWidget(self.lbl_offset_z_units, 2, 2, 1, 1)
        self.lbl_offset_x = QtWidgets.QLabel(self.gridLayoutWidget_6)
        self.lbl_offset_x.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_x.setObjectName("lbl_offset_x")
        self.gridLayout_6.addWidget(self.lbl_offset_x, 0, 0, 1, 1)
        self.sbox_offset_z = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_6)
        self.sbox_offset_z.setDecimals(2)
        self.sbox_offset_z.setMinimum(-150.0)
        self.sbox_offset_z.setMaximum(150.0)
        self.sbox_offset_z.setSingleStep(0.05)
        self.sbox_offset_z.setObjectName("sbox_offset_z")
        self.gridLayout_6.addWidget(self.sbox_offset_z, 2, 1, 1, 1)
        self.gridLayoutWidget_7 = QtWidgets.QWidget(self.grp_object_offsets)
        self.gridLayoutWidget_7.setGeometry(QtCore.QRect(-50, 120, 241, 80))
        self.gridLayoutWidget_7.setObjectName("gridLayoutWidget_7")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.gridLayoutWidget_7)
        self.gridLayout_7.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.lbl_offset_ry = QtWidgets.QLabel(self.gridLayoutWidget_7)
        self.lbl_offset_ry.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_ry.setObjectName("lbl_offset_ry")
        self.gridLayout_7.addWidget(self.lbl_offset_ry, 1, 0, 1, 1)
        self.lbl_offset_ry_units = QtWidgets.QLabel(self.gridLayoutWidget_7)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_ry_units.setFont(font)
        self.lbl_offset_ry_units.setObjectName("lbl_offset_ry_units")
        self.gridLayout_7.addWidget(self.lbl_offset_ry_units, 1, 2, 1, 1)
        self.sbox_offset_rx = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_7)
        self.sbox_offset_rx.setDecimals(2)
        self.sbox_offset_rx.setMinimum(-360.0)
        self.sbox_offset_rx.setMaximum(360.0)
        self.sbox_offset_rx.setSingleStep(0.05)
        self.sbox_offset_rx.setObjectName("sbox_offset_rx")
        self.gridLayout_7.addWidget(self.sbox_offset_rx, 0, 1, 1, 1)
        self.sbox_offset_ry = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_7)
        self.sbox_offset_ry.setDecimals(2)
        self.sbox_offset_ry.setMinimum(-360.0)
        self.sbox_offset_ry.setMaximum(360.0)
        self.sbox_offset_ry.setSingleStep(0.05)
        self.sbox_offset_ry.setObjectName("sbox_offset_ry")
        self.gridLayout_7.addWidget(self.sbox_offset_ry, 1, 1, 1, 1)
        self.lbl_offset_rx_units = QtWidgets.QLabel(self.gridLayoutWidget_7)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_rx_units.setFont(font)
        self.lbl_offset_rx_units.setObjectName("lbl_offset_rx_units")
        self.gridLayout_7.addWidget(self.lbl_offset_rx_units, 0, 2, 1, 1)
        self.lbl_offset_rz = QtWidgets.QLabel(self.gridLayoutWidget_7)
        self.lbl_offset_rz.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_rz.setObjectName("lbl_offset_rz")
        self.gridLayout_7.addWidget(self.lbl_offset_rz, 2, 0, 1, 1)
        self.lbl_offset_rz_units = QtWidgets.QLabel(self.gridLayoutWidget_7)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_rz_units.setFont(font)
        self.lbl_offset_rz_units.setObjectName("lbl_offset_rz_units")
        self.gridLayout_7.addWidget(self.lbl_offset_rz_units, 2, 2, 1, 1)
        self.lbl_offset_rx = QtWidgets.QLabel(self.gridLayoutWidget_7)
        self.lbl_offset_rx.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_rx.setObjectName("lbl_offset_rx")
        self.gridLayout_7.addWidget(self.lbl_offset_rx, 0, 0, 1, 1)
        self.sbox_offset_rz = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_7)
        self.sbox_offset_rz.setDecimals(2)
        self.sbox_offset_rz.setMinimum(-360.0)
        self.sbox_offset_rz.setMaximum(360.0)
        self.sbox_offset_rz.setSingleStep(0.05)
        self.sbox_offset_rz.setObjectName("sbox_offset_rz")
        self.gridLayout_7.addWidget(self.sbox_offset_rz, 2, 1, 1, 1)
        self.tabWidget.addTab(self.tab_object, "")
        self.tab_robot = QtWidgets.QWidget()
        self.tab_robot.setObjectName("tab_robot")
        self.grp_robot_offsets = QtWidgets.QGroupBox(self.tab_robot)
        self.grp_robot_offsets.setGeometry(QtCore.QRect(10, 10, 211, 111))
        self.grp_robot_offsets.setObjectName("grp_robot_offsets")
        self.gridLayoutWidget_8 = QtWidgets.QWidget(self.grp_robot_offsets)
        self.gridLayoutWidget_8.setGeometry(QtCore.QRect(-50, 20, 241, 80))
        self.gridLayoutWidget_8.setObjectName("gridLayoutWidget_8")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.gridLayoutWidget_8)
        self.gridLayout_8.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.lbl_offset_y_3 = QtWidgets.QLabel(self.gridLayoutWidget_8)
        self.lbl_offset_y_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_y_3.setObjectName("lbl_offset_y_3")
        self.gridLayout_8.addWidget(self.lbl_offset_y_3, 1, 0, 1, 1)
        self.lbl_offset_y_units_3 = QtWidgets.QLabel(self.gridLayoutWidget_8)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_y_units_3.setFont(font)
        self.lbl_offset_y_units_3.setObjectName("lbl_offset_y_units_3")
        self.gridLayout_8.addWidget(self.lbl_offset_y_units_3, 1, 2, 1, 1)
        self.sbox_rbt_offset_x = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_8)
        self.sbox_rbt_offset_x.setDecimals(2)
        self.sbox_rbt_offset_x.setMinimum(-99999.0)
        self.sbox_rbt_offset_x.setMaximum(99999.0)
        self.sbox_rbt_offset_x.setSingleStep(0.05)
        self.sbox_rbt_offset_x.setProperty("value", -340.0)
        self.sbox_rbt_offset_x.setObjectName("sbox_rbt_offset_x")
        self.gridLayout_8.addWidget(self.sbox_rbt_offset_x, 0, 1, 1, 1)
        self.sbox_rbt_offset_y = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_8)
        self.sbox_rbt_offset_y.setDecimals(2)
        self.sbox_rbt_offset_y.setMinimum(-99999.0)
        self.sbox_rbt_offset_y.setMaximum(99999.0)
        self.sbox_rbt_offset_y.setSingleStep(0.05)
        self.sbox_rbt_offset_y.setObjectName("sbox_rbt_offset_y")
        self.gridLayout_8.addWidget(self.sbox_rbt_offset_y, 1, 1, 1, 1)
        self.lbl_offset_x_units_3 = QtWidgets.QLabel(self.gridLayoutWidget_8)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_x_units_3.setFont(font)
        self.lbl_offset_x_units_3.setObjectName("lbl_offset_x_units_3")
        self.gridLayout_8.addWidget(self.lbl_offset_x_units_3, 0, 2, 1, 1)
        self.lbl_offset_z_3 = QtWidgets.QLabel(self.gridLayoutWidget_8)
        self.lbl_offset_z_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_z_3.setObjectName("lbl_offset_z_3")
        self.gridLayout_8.addWidget(self.lbl_offset_z_3, 2, 0, 1, 1)
        self.lbl_offset_z_units_3 = QtWidgets.QLabel(self.gridLayoutWidget_8)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_offset_z_units_3.setFont(font)
        self.lbl_offset_z_units_3.setObjectName("lbl_offset_z_units_3")
        self.gridLayout_8.addWidget(self.lbl_offset_z_units_3, 2, 2, 1, 1)
        self.lbl_offset_x_3 = QtWidgets.QLabel(self.gridLayoutWidget_8)
        self.lbl_offset_x_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.lbl_offset_x_3.setObjectName("lbl_offset_x_3")
        self.gridLayout_8.addWidget(self.lbl_offset_x_3, 0, 0, 1, 1)
        self.sbox_rbt_offset_z = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_8)
        self.sbox_rbt_offset_z.setDecimals(2)
        self.sbox_rbt_offset_z.setMinimum(-99999.0)
        self.sbox_rbt_offset_z.setMaximum(99999.0)
        self.sbox_rbt_offset_z.setSingleStep(0.05)
        self.sbox_rbt_offset_z.setObjectName("sbox_rbt_offset_z")
        self.gridLayout_8.addWidget(self.sbox_rbt_offset_z, 2, 1, 1, 1)
        self.tabWidget.addTab(self.tab_robot, "")
        ObjectWizard.addPage(self.wiz_page_sim_offsets)

        self.retranslateUi(ObjectWizard)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(ObjectWizard)
        ObjectWizard.setTabOrder(self.sbox_rbt_offset_x, self.sbox_rbt_offset_y)
        ObjectWizard.setTabOrder(self.sbox_rbt_offset_y, self.sbox_rbt_offset_z)
        ObjectWizard.setTabOrder(self.sbox_rbt_offset_z, self.sbox_offset_x)
        ObjectWizard.setTabOrder(self.sbox_offset_x, self.sbox_offset_y)
        ObjectWizard.setTabOrder(self.sbox_offset_y, self.sbox_offset_z)
        ObjectWizard.setTabOrder(self.sbox_offset_z, self.sbox_import_X)
        ObjectWizard.setTabOrder(self.sbox_import_X, self.cbox_import_units)
        ObjectWizard.setTabOrder(self.cbox_import_units, self.sbox_import_Y)
        ObjectWizard.setTabOrder(self.sbox_import_Y, self.sbox_import_Z)
        ObjectWizard.setTabOrder(self.sbox_import_Z, self.txt_path)
        ObjectWizard.setTabOrder(self.txt_path, self.btn_prompt_path)
        ObjectWizard.setTabOrder(self.btn_prompt_path, self.rbtn_prim_rect)
        ObjectWizard.setTabOrder(self.rbtn_prim_rect, self.rbtn_prim_cylinder)
        ObjectWizard.setTabOrder(self.rbtn_prim_cylinder, self.rbtn_prim_sphere)
        ObjectWizard.setTabOrder(self.rbtn_prim_sphere, self.sbox_prim_field_A)
        ObjectWizard.setTabOrder(self.sbox_prim_field_A, self.sbox_prim_field_B)
        ObjectWizard.setTabOrder(self.sbox_prim_field_B, self.sbox_prim_field_C)
        ObjectWizard.setTabOrder(self.sbox_prim_field_C, self.rbt_primitive)
        ObjectWizard.setTabOrder(self.rbt_primitive, self.rbtn_import_obj)
        ObjectWizard.setTabOrder(self.rbtn_import_obj, self.rbt_scan_obj)
        ObjectWizard.setTabOrder(self.rbt_scan_obj, self.rbtn_import_scale)
        ObjectWizard.setTabOrder(self.rbtn_import_scale, self.tabWidget)
        ObjectWizard.setTabOrder(self.tabWidget, self.btn_find)
        ObjectWizard.setTabOrder(self.btn_find, self.rbtn_import_units)
        ObjectWizard.setTabOrder(self.rbtn_import_units, self.sbox_scan_x)
        ObjectWizard.setTabOrder(self.sbox_scan_x, self.sbox_scan_y)
        ObjectWizard.setTabOrder(self.sbox_scan_y, self.sbox_scan_z)
        ObjectWizard.setTabOrder(self.sbox_scan_z, self.btn_start_scan)

    def retranslateUi(self, ObjectWizard):
        _translate = QtCore.QCoreApplication.translate
        ObjectWizard.setWindowTitle(_translate("ObjectWizard", "Object Processing Wizard"))
        self.wiz_page_method.setTitle(_translate("ObjectWizard", "Choose Import Method"))
        self.wiz_page_method.setSubTitle(_translate("ObjectWizard", "Select a method to import object for ESD scan..."))
        self.rbt_primitive.setText(_translate("ObjectWizard", "Create Primitive Object"))
        self.rbtn_import_obj.setText(_translate("ObjectWizard", "Import Object from File (.stl, .obj)"))
        self.rbt_scan_obj.setText(_translate("ObjectWizard", "Scan and Generate Object"))
        self.wiz_page_create_primitive.setTitle(_translate("ObjectWizard", "Create Primitive Object"))
        self.wiz_page_create_primitive.setSubTitle(_translate("ObjectWizard", "Select and create a primitive shape..."))
        self.groupBox.setTitle(_translate("ObjectWizard", "Primitive Type"))
        self.rbtn_prim_rect.setText(_translate("ObjectWizard", "Rectangular Prism"))
        self.rbtn_prim_cylinder.setText(_translate("ObjectWizard", "Cylinder"))
        self.rbtn_prim_sphere.setText(_translate("ObjectWizard", "Sphere"))
        self.groupBox_2.setTitle(_translate("ObjectWizard", "Primitive Properties"))
        self.lbl_prim_field_A.setText(_translate("ObjectWizard", "X"))
        self.lbl_prim_field_units_B.setText(_translate("ObjectWizard", "mm"))
        self.lbl_prim_field_units_A.setText(_translate("ObjectWizard", "mm"))
        self.lbl_prim_field_B.setText(_translate("ObjectWizard", "Y"))
        self.lbl_prim_field_C.setText(_translate("ObjectWizard", "Z"))
        self.lbl_prim_field_units_C.setText(_translate("ObjectWizard", "mm"))
        self.wiz_page_import_obj.setTitle(_translate("ObjectWizard", "Import Object From File"))
        self.wiz_page_import_obj.setSubTitle(_translate("ObjectWizard", "Select a path to an object file (.stl/.obj)..."))
        self.btn_prompt_path.setText(_translate("ObjectWizard", "..."))
        self.lbl_import_path.setText(_translate("ObjectWizard", "Object Path:"))
        self.groupBox_3.setTitle(_translate("ObjectWizard", "Object Properties"))
        self.cbox_import_units.setItemText(0, _translate("ObjectWizard", "m"))
        self.cbox_import_units.setItemText(1, _translate("ObjectWizard", "cm"))
        self.cbox_import_units.setItemText(2, _translate("ObjectWizard", "mm"))
        self.cbox_import_units.setItemText(3, _translate("ObjectWizard", "ft"))
        self.cbox_import_units.setItemText(4, _translate("ObjectWizard", "in"))
        self.rbtn_import_units.setText(_translate("ObjectWizard", "Units"))
        self.rbtn_import_scale.setText(_translate("ObjectWizard", "Absolute Scale"))
        self.lbl_import_X.setText(_translate("ObjectWizard", "X"))
        self.lbl_import_Y.setText(_translate("ObjectWizard", "Y"))
        self.lbl_import_Z.setText(_translate("ObjectWizard", "Z"))
        self.wiz_page_scan_obj.setTitle(_translate("ObjectWizard", "Scan Object"))
        self.wiz_page_scan_obj.setSubTitle(_translate("ObjectWizard", "Enter object dimensions to scan object... NOTE: this takes a long time!"))
        self.groupBox_4.setTitle(_translate("ObjectWizard", "Object Dimensions"))
        self.lbl_scan_x.setText(_translate("ObjectWizard", "X"))
        self.lbl_scan_z.setText(_translate("ObjectWizard", "Z"))
        self.lbl_scan_y.setText(_translate("ObjectWizard", "Y"))
        self.lbl_scan_x_units.setText(_translate("ObjectWizard", "mm"))
        self.lbl_scan_y_units.setText(_translate("ObjectWizard", "mm"))
        self.lbl_scan_z_units.setText(_translate("ObjectWizard", "mm"))
        self.btn_start_scan.setText(_translate("ObjectWizard", "Begin Scan"))
        self.btn_scan_halt.setText(_translate("ObjectWizard", "Halt"))
        self.lbl_current_points.setText(_translate("ObjectWizard", "0/1"))
        self.wiz_page_visualize_obj.setTitle(_translate("ObjectWizard", "Visualize Object"))
        self.wiz_page_visualize_obj.setSubTitle(_translate("ObjectWizard", "Ensure object has been properly imported into Open3D enviroment..."))
        self.wiz_page_sim_offsets.setTitle(_translate("ObjectWizard", "Set Object Offests"))
        self.wiz_page_sim_offsets.setSubTitle(_translate("ObjectWizard", "Input offets to align object in simulated enviroment. You can also modify the robot offsets, but\n"
"                    this is not recommneded unless you know what you are doing.\n"
"                "))
        self.btn_find.setText(_translate("ObjectWizard", "Find"))
        self.grp_object_offsets.setTitle(_translate("ObjectWizard", "Object Offset"))
        self.lbl_offset_y.setText(_translate("ObjectWizard", "Y"))
        self.lbl_offset_y_units.setText(_translate("ObjectWizard", "mm"))
        self.lbl_offset_x_units.setText(_translate("ObjectWizard", "mm"))
        self.lbl_offset_z.setText(_translate("ObjectWizard", "Z"))
        self.lbl_offset_z_units.setText(_translate("ObjectWizard", "mm"))
        self.lbl_offset_x.setText(_translate("ObjectWizard", "X"))
        self.lbl_offset_ry.setText(_translate("ObjectWizard", "rY"))
        self.lbl_offset_ry_units.setText(_translate("ObjectWizard", "deg"))
        self.lbl_offset_rx_units.setText(_translate("ObjectWizard", "deg"))
        self.lbl_offset_rz.setText(_translate("ObjectWizard", "rZ"))
        self.lbl_offset_rz_units.setText(_translate("ObjectWizard", "deg"))
        self.lbl_offset_rx.setText(_translate("ObjectWizard", "rX"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_object), _translate("ObjectWizard", "Object Offsets"))
        self.grp_robot_offsets.setTitle(_translate("ObjectWizard", "Robot Offset"))
        self.lbl_offset_y_3.setText(_translate("ObjectWizard", "Y"))
        self.lbl_offset_y_units_3.setText(_translate("ObjectWizard", "mm"))
        self.lbl_offset_x_units_3.setText(_translate("ObjectWizard", "mm"))
        self.lbl_offset_z_3.setText(_translate("ObjectWizard", "Z"))
        self.lbl_offset_z_units_3.setText(_translate("ObjectWizard", "mm"))
        self.lbl_offset_x_3.setText(_translate("ObjectWizard", "X"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_robot), _translate("ObjectWizard", "Robot Offsets"))